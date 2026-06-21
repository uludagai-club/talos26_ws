cmake_minimum_required(VERSION 3.0.2)
project(talos_ground_removal)

add_compile_options(-std=c++17 -O2 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf
  jsk_recognition_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions tf jsk_recognition_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# ground_removal (Patchwork++ zemin ayiklama; cekirdek vendor: include/patchworkpp)
add_executable(ground_removal src/ground_removal.cpp)
target_link_libraries(ground_removal ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS ground_removal
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
