#include "scan_to_pcl_ros/scan_to_pcl_ros.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ScanToPcl::ScanToPcl():private_nh_("~")
{
    private_nh_.param<int>("hz", hz_, 10);
    hokuyo_sub_ = nh_.subscribe("/scan", 1, &ScanToPcl::hokuyo_callback, this);
    pcl_from_scan_pub_ = nh_.advertise<PointCloud>("/pcl_from_scan", 1);
}

void ScanToPcl::hokuyo_callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_in, cloud); // convert a sensor_msgs/LaserScan to a sensor_msgs/PointCloud

    // Publish the new point cloud.
    cloud.header.frame_id = "base_link";
    cloud.header.stamp = scan_in->header.stamp;
    pcl_from_scan_pub_.publish(cloud);
}

void ScanToPcl::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
