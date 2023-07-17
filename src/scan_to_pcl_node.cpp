#include "scan_to_pcl_ros/scan_to_pcl.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserScan_to_pointcloud");
    ScanToPcl laserScan_to_pointcloud;
    laserScan_to_pointcloud.process();

    return 0;
}
