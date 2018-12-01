cmake_minimum_required(VERSION 2.8.3)
project(rosie_path_finder)

find_package(catkin REQUIRED COMPONENTS
	 message_generation
	 roscpp
	 std_msgs
	 phidgets
	 nav_msgs
	 geometry_msgs
	 sensor_msgs
	 tf
	 visualization_msgs
	 pcl_ros
	)


add_message_files(
  FILES
  request_rerun.msg
  commission_rerun.msg
)

add_service_files(
  FILES
  CommissionRerun.srv
  RequestRerun.srv
  StartRRT.srv 
  rrtService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets geometry_msgs sensor_msgs nav_msgs tf  pcl_ros visualization_msgs message_runtime
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_rrt src/rosie_rrt.cpp)
add_executable(rosie_rrt_service src/rosie_rrt_service.cpp)
target_link_libraries(rosie_rrt ${catkin_LIBRARIES})
target_link_libraries(rosie_rrt_service ${catkin_LIBRARIES})
add_dependencies(rosie_rrt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(rosie_rrt_service ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#std_msgs nav_msgs phidgets geometry_msgs sensor_msgs tf pcl_ros visualization_msgs
