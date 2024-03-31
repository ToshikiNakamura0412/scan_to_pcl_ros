#ifndef SCAN_TO_PCL_ROS_HPP
#define SCAN_TO_PCL_ROS_HPP

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <laser_geometry/laser_geometry.hpp>


class ScanToPcl : public rclcpp::Node
{
public:
    ScanToPcl();

private:
    void hokuyo_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    std::string frame_id_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr hokuyo_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_from_scan_pub_;
};

#endif
