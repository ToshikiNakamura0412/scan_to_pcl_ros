/**
 * @file scan_to_pcl.cpp
 * @author Toshiki Nakamura
 * @brief Convert a sensor_msgs/LaserScan to a sensor_msgs/PointCloud
 * @copyright Copyright (c) 2024
 */

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_listener.h>

/**
 * @class ScanToPcl
 * @brief Class for converting a sensor_msgs/LaserScan to a sensor_msgs/PointCloud
 */
class ScanToPcl
{
public:
  /**
   * @brief Construct a new Scan To Pcl object
   */
  ScanToPcl(void) : private_nh_("~")
  {
    private_nh_.param<std::string>("frame_id", frame_id_, std::string("base_scan"));

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    laser_scan_sub_ =
        nh_.subscribe("/scan", 1, &ScanToPcl::laser_scan_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    ROS_INFO_STREAM(ros::this_node::getName() << " node has started..");
    ROS_INFO_STREAM("frame_id: " << frame_id_);
  }

private:
  /**
   * @brief Callback function for the subscriber of the scan topic
   * @details Convert a sensor_msgs/LaserScan to a sensor_msgs/PointCloud
   *
   * @param msg laser scan message
   */
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
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
  ros::Subscriber laser_scan_sub_;

  tf::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scan_to_pcl");
  ScanToPcl scan_to_pcl;
  ros::spin();

  return 0;
}
