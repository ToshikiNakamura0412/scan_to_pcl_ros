/**
 * @file scan_to_pcl_node.cpp
 * @author Toshiki Nakamura
 * @brief Convert a sensor_msgs/LaserScan to a sensor_msgs/PointCloud
 * @copyright Copyright (c) 2024
 */

#include "scan_to_pcl_ros/scan_to_pcl_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserScan_to_pointcloud");
  ScanToPcl laserScan_to_pointcloud;
  laserScan_to_pointcloud.process();

  return 0;
}
