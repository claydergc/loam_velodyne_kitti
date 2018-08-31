cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne_kitti)

if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(
  include
	"./include"
	${PCL_INCLUDE_DIRS})

## Compile as C++14, supported in ROS Kinetic and newer
# set_property(TARGET invz_player PROPERTY CXX_STANDARD 17)
#add_compile_options(-std=c++14)

add_definitions( -march=native )

add_subdirectory(src/lib)

add_executable(main src/main.cpp)
target_link_libraries(main ${PCL_LIBRARIES} loam )

#add_executable(multiScanRegistration src/multi_scan_registration_node.cpp)
#target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#add_executable(laserOdometry src/laser_odometry_node.cpp)
#target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#add_executable(laserMapping src/laser_mapping_node.cpp)
#target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

#add_executable(transformMaintenance src/transform_maintenance_node.cpp)
#target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )
