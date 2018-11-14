cmake_minimum_required(VERSION 2.8.3)
project(rosie_map_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs phidgets geometry_msgs sensor_msgs tf message_filters laser_geometry pcl_ros)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets geometry_msgs sensor_msgs tf message_filters laser_geometry pcl_ros
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_laser_publisher src/rosie_laser_publisher.cpp)
add_executable(rosie_laser_broadcaster src/tf_laser_broadcaster.cpp)
target_link_libraries(rosie_laser_publisher ${catkin_LIBRARIES})
target_link_libraries(rosie_laser_broadcaster ${catkin_LIBRARIES})
add_dependencies(rosie_laser_publisher rosie_laser_broadcaster std_msgs phidgets geometry_msgs sensor_msgs tf message_filters laser_geometry pcl_ros)


