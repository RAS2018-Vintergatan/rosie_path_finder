cmake_minimum_required(VERSION 2.8.3)
project(rosie_path_finder)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs phidgets nav_msgs geometry_msgs sensor_msgs tf visualization_msgs pcl_ros)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets geometry_msgs sensor_msgs nav_msgs tf  pcl_ros visualization_msgs
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_rrt src/rosie_rrt.cpp)

target_link_libraries(rosie_rrt ${catkin_LIBRARIES})

add_dependencies( rosie_rrt ${catkin_EXPORTED_TARGETS})
#std_msgs nav_msgs phidgets geometry_msgs sensor_msgs tf pcl_ros visualization_msgs
