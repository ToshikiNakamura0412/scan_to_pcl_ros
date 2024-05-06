/**
 * @file scan_to_pcl.cpp
 * @author Toshiki Nakamura
 * @brief Convert a sensor_msgs/LaserScan to a sensor_msgs/PointCloud
 * @copyright Copyright (c) 2024
 */

#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_listener.h>

class ScanToPcl
{
public:
  ScanToPcl() : private_nh_("~")
  {
    private_nh_.param<std::string>("frame_id", frame_id_, std::string("base_scan"));

    cloud_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/cloud", 1);
    scan_sub_ = nh_.subscribe("/scan", 1, &ScanToPcl::scan_callback, this);

    tf_listener_.setExtrapolationLimit(ros::Duration(0.1));

    ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
    ROS_INFO_STREAM("frame_id: " << frame_id_);
  }

private:
  void scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.transformLaserScanToPointCloud(frame_id_, *msg, cloud, tf_listener_);
    cloud.header.frame_id = frame_id_;
    cloud.header.stamp = ros::Time::now();
    cloud_pub_.publish(cloud);
  }

  std::string frame_id_;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
  ros::Subscriber scan_sub_;

  tf::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_pcl");
  ScanToPcl scan_to_pcl;
  ros::spin();

  return 0;
}
