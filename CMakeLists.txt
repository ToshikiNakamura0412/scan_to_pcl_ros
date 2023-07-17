cmake_minimum_required(VERSION 3.0.2)
project(scan_to_pcl_ros)

add_compile_options(-std=c++14 -O2 -g)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    pcl_conversions
    laser_geometry
    sensor_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES scan_to_pcl_ros
    # CATKIN_DEPENDS other_catkin_pkg
    # DEPENDS system_lib
)


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
## Declare a C++ library
add_library(${PROJECT_NAME} src/scan_to_pcl.cpp)
## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/scan_to_pcl_node.cpp)
## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
