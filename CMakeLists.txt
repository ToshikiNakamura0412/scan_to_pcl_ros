cmake_minimum_required(VERSION 3.5)
project(scan_to_pcl_ros)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_executable(${PROJECT_NAME}_node
  src/scan_to_pcl.cpp
  src/scan_to_pcl_node.cpp)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)
ament_auto_package()
