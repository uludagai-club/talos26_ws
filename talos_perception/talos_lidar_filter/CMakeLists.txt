cmake_minimum_required(VERSION 3.0.2)
project(talos_lidar_filter)

add_compile_options(-std=c++17 -O2 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# pass_filter (pass-through Z filtresi)
add_executable(pass_filter src/pass_filter.cpp)
target_link_libraries(pass_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# voxel_filter (voxel grid downsampling)
add_executable(voxel_filter src/voxel_filter.cpp)
target_link_libraries(voxel_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# outlier_remove (statistical outlier removal)
add_executable(outlier_remove src/outlier_remove.cpp)
target_link_libraries(outlier_remove ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS pass_filter voxel_filter outlier_remove
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
