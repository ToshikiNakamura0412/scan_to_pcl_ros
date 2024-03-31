#include <rclcpp/rclcpp.hpp>

#include "scan_to_pcl_ros/scan_to_pcl_ros.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToPcl>());
    rclcpp::shutdown();

    return 0;
}
