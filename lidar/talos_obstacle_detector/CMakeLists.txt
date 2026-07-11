cmake_minimum_required(VERSION 3.0.2)
project(talos_obstacle_detector)

add_compile_options(-std=c++17 -O2 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  jsk_recognition_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
    tf2_ros
    tf2_sensor_msgs
    jsk_recognition_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(map_publisher_node src/map_publisher_node.cpp)
target_link_libraries(map_publisher_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
target_link_libraries(obstacle_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS map_publisher_node obstacle_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
