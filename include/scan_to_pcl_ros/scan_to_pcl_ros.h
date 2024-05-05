/**
 * @file scan_to_pcl_ros.hpp
 * @author Toshiki Nakamura
 * @brief Convert a sensor_msgs/LaserScan to a sensor_msgs/PointCloud
 * @copyright Copyright (c) 2024
 */

#ifndef SCAN_TO_PCL_ROS_HPP
#define SCAN_TO_PCL_ROS_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>

class ScanToPcl
{
public:
    ScanToPcl();
    void process();
private:
    void hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    int hz_;
    std::string frame_id_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber hokuyo_sub_;
    ros::Publisher  pcl_from_scan_pub_;
};

#endif
