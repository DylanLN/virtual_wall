#ifndef VIRTUAL_WALL_H_
#define VIRTUAL_WALL_H_

// std lib includes
#include <iostream>
#include <string>
#include <vector>
#include <map>

// ros includes
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <virtual_wall/Wall.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Int32.h>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>


namespace virtual_wall
{

class VirtualWall : public costmap_2d::Layer
{
private:
    /* data */
    // msg system
    ros::NodeHandle nh;
    ros::Subscriber sub;
    double resolution;
    double wallmax_x,wallmax_y,wallmin_x,wallmin_y;
    //costmap框架
    std::string global_frame_;
    //地图框架
    std::string map_frame_;
    //添加地图。
    ros::Subscriber add_wall_sub_;
    //地图列表
    ros::Publisher wall_list_pub;
    //发布虚拟墙标记
    ros::Publisher wall_maker_pub;
    // //删除地图。
    ros::Subscriber delete_wall_sub;
    // ros::ServiceServer delete_wall_server_;
    // //get地图
    // ros::ServiceServer get_wall_server_;

    boost::recursive_mutex data_access_;
    bool rolling_window_;

    std::vector<virtual_wall::Wall> v_wall;
    std::vector<virtual_wall::Wall> wallPoint;

    void AddWallCallback(const geometry_msgs::PointStampedConstPtr& msg);
    void DeleteWallCallback(const std_msgs::Int32ConstPtr& msg);

    bool WallInterpolation();

    bool updateWall(costmap_2d::Costmap2D& master_grid);

public:
    VirtualWall(/* args */);
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void matchSize();

    bool isDiscretized()
    {
        return true;
    }

    virtual void reset() { onInitialize(); }
    virtual ~VirtualWall();
};
}

#endif
