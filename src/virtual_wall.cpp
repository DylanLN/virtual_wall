#include <pluginlib/class_list_macros.h>

#include <virtual_wall/virtual_wall.h>

PLUGINLIB_EXPORT_CLASS(virtual_wall::VirtualWall, costmap_2d::Layer)

using namespace std;

namespace virtual_wall{

VirtualWall::VirtualWall()
{
  cout << "VirtualWall()" << endl;
    // ros::NodeHandle n;
    wallmax_x = 0.0;
    wallmax_y = 0.0;
    wallmin_x = 0.0;
    wallmin_y = 0.0;
}

VirtualWall::~VirtualWall()
{
  cout << "~VirtualWall()" << endl;
}

void VirtualWall::onInitialize()
{
    ros::NodeHandle g_nh;
    nh = ros::NodeHandle("~/" + name_);
    matchSize();
    current_ = true;

    add_wall_sub_ = g_nh.subscribe("/clicked_point", 1, &VirtualWall::AddWallCallback, this);
    wall_list_pub = g_nh.advertise<std_msgs::Int32>("/virtual_wall_list", 1);
    delete_wall_sub = g_nh.subscribe("/delete_wall", 1, &VirtualWall::DeleteWallCallback, this);
}

void VirtualWall::matchSize()
{
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    resolution = master->getResolution();
    // resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
    //           master->getOriginX(), master->getOriginY());
}

/* *********************************************************************
 * updateBounds
 *
 * update obstacles and bounds
 */
void VirtualWall::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
    *min_x = std::min(wallmin_x, *min_x);
    *min_y = std::min(wallmin_y, *min_y);
    *max_x = std::max(wallmax_x, *max_x);
    *max_y = std::max(wallmax_y, *max_y);
}

/* *********************************************************************
 * updateCosts
 *
 * updates the master grid in the area defined in updateBounds
 */
void VirtualWall::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  // cout << "size : " << wallPoint.size() << endl;
    for (size_t i = 0; i < wallPoint.size(); i++)
    {
      std_msgs::Int32 msg;
      msg.data = wallPoint[i].id;
      wall_list_pub.publish(msg);
      for (size_t j = 0; j < wallPoint[i].polygon.points.size(); j++)
      {
          // cout << "(" << wallPoint[i].polygon.points[j].x << ", " << wallPoint[i].polygon.points[j].y << ")";
          unsigned int pixle_x;
          unsigned int pixle_y;
          bool ret = master_grid.worldToMap(wallPoint[i].polygon.points[j].x, wallPoint[i].polygon.points[j].y, pixle_x, pixle_y);
          if (ret)
          {
            // cout << " (" << pixle_x << ", " << pixle_y << ") ";
            master_grid.setCost(pixle_x, pixle_y, costmap_2d::LETHAL_OBSTACLE);
          }
          // else
            // cout << "( ret )";
      }
    }
}

//虚拟墙插补运算
bool VirtualWall::WallInterpolation()
{
  double pixle_x[2];
  double pixle_y[2];
  double k,b;
  for (size_t i = 0; i < 2; i++)
  {
    if (fabs(wallPoint.back().polygon.points[0].x - wallPoint.back().polygon.points[1].x) > fabs(wallPoint.back().polygon.points[0].y - wallPoint.back().polygon.points[1].y))
    {
      pixle_x[i] = wallPoint.back().polygon.points[i].x;
      pixle_y[i] = wallPoint.back().polygon.points[i].y;
    }else{
      pixle_x[i] = wallPoint.back().polygon.points[i].y;
      pixle_y[i] = wallPoint.back().polygon.points[i].x;
    }
  }

  k = (pixle_y[0] - pixle_y[1]) / (pixle_x[0] - pixle_x[1]);
  b = pixle_y[0] - k * pixle_x[0];
  // cout << "k : " << k << "b " << b << endl;

  wallPoint.back().polygon.points.clear();
  for (double i = std::min(pixle_x[0], pixle_x[1]); i < std::max(pixle_x[0], pixle_x[1]); i += resolution)
  {
    // cout << "i " << i << " , " << std::max(pixle_x[0], pixle_x[1]) << endl;
    geometry_msgs::Point32 point;
    if (fabs(v_wall.back().polygon.points[0].x - v_wall.back().polygon.points[1].x) > fabs(v_wall.back().polygon.points[0].y - v_wall.back().polygon.points[1].y))
    {
      point.x = i;
      point.y = k * i + b;
    }else{
      point.x = k * i + b;
      point.y = i;
    }
    // cout << i << "(" << point.x << " , " << point.y << ") ";
    wallPoint.back().polygon.points.push_back(point);
  }
  // cout << endl;
  return true;
}

bool VirtualWall::updateWall(costmap_2d::Costmap2D& master_grid)
{
    for (size_t i = 0; i < wallPoint.size(); i++)
    {
      std_msgs::Int32 msg;
      msg.data = wallPoint[i].id;
      wall_list_pub.publish(msg);
      for (size_t j = 0; j < wallPoint[i].polygon.points.size(); j++)
      {
          // cout << "(" << wallPoint[i].polygon.points[j].x << ", " << wallPoint[i].polygon.points[j].y << ")";
          unsigned int pixle_x;
          unsigned int pixle_y;
          bool ret = master_grid.worldToMap(wallPoint[i].polygon.points[j].x, wallPoint[i].polygon.points[j].y, pixle_x, pixle_y);
          if (ret)
          {
            // cout << " (" << pixle_x << ", " << pixle_y << ") ";
            master_grid.setCost(pixle_x, pixle_y, costmap_2d::LETHAL_OBSTACLE);
          }
          // else
            // cout << "( ret )";
      }
    }
}

//添加虚拟墙
void VirtualWall::AddWallCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  geometry_msgs::Point32 point;
  point.x = msg->point.x;
  point.y = msg->point.y;
  point.z = msg->point.z;
  wallmax_x = std::max(wallmax_x, msg->point.x);
  wallmax_y = std::max(wallmax_y, msg->point.y);
  wallmin_x = std::min(wallmin_x, msg->point.x);
  wallmin_y = std::min(wallmin_y, msg->point.y);

  if (v_wall.size() == 0)
  {
    virtual_wall::Wall wall;
    wall.id = 0;
    // wall.name = "s";
    wall.polygon.points.push_back(point);
    v_wall.push_back(wall);
  }else{
    if (v_wall.back().polygon.points.size() == 1)
    {
      if (v_wall.size() == 1)
      {
        v_wall.back().id = 0;
      }else{
        v_wall.back().id = v_wall[v_wall.size() - 2].id + 1;
      }
      v_wall.back().polygon.points.push_back(point);
      wallPoint.push_back(v_wall.back());
      //对虚拟墙插值
      WallInterpolation();
      // updateWall();
    }else if(v_wall.back().polygon.points.size() == 2)
    {
      virtual_wall::Wall wall;
      wall.id = v_wall.size();
      // wall.name = "s";
      wall.polygon.points.push_back(point);
      v_wall.push_back(wall);
    }
  }
}

//删除虚拟墙
void VirtualWall::DeleteWallCallback(const std_msgs::Int32ConstPtr& msg)
{
  for (size_t i = 0; i < v_wall.size(); i++)
  {
    if(v_wall[i].id == msg->data){
      v_wall.erase(v_wall.begin() + i);
      wallPoint.erase(wallPoint.begin() + i);
    }
  }
}


}