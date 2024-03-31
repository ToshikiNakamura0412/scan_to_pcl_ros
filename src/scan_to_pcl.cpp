#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "scan_to_pcl_ros/scan_to_pcl_ros.hpp"

using namespace std::chrono_literals;

ScanToPcl::ScanToPcl() : Node("laserScan_to_pointcloud")
{
    frame_id_ = this->declare_parameter<std::string>("frame_id", "base_link");

    hokuyo_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(1).reliable(),
        std::bind(&ScanToPcl::hokuyo_callback, this, std::placeholders::_1));
    pcl_from_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/pcl_from_scan", rclcpp::QoS(1).reliable());

    RCLCPP_INFO_STREAM(this->get_logger(), "=== " << rclcpp::Node::get_name() << " ===");
    RCLCPP_INFO_STREAM(this->get_logger(), "frame_id: " << frame_id_);
}

void ScanToPcl::hokuyo_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    laser_geometry::LaserProjection projector;
    projector.projectLaser(*scan_msg, *cloud_msg); // convert a sensor_msgs/msg/LaserScan to a sensor_msgs/msg/PointCloud2

    // Publish the new point cloud.
    cloud_msg->header.frame_id = frame_id_;
    cloud_msg->header.stamp = scan_msg->header.stamp;
    pcl_from_scan_pub_->publish(std::move(cloud_msg));
}