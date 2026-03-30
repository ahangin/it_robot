cmake_minimum_required(VERSION 2.8.3)
project(cloud_deskewed)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  message_generation
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Projection
# RS激光雷达去畸变节点
add_executable(rs_lidar_deskew src/rs_lidar_deskew.cpp)
add_dependencies(rs_lidar_deskew ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rs_lidar_deskew 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)
# Velodyne激光雷达去畸变节点
add_executable(velodyne_lidar_deskew src/velodyne_lidar_deskew.cpp)
add_dependencies(velodyne_lidar_deskew ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(velodyne_lidar_deskew 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenMP_CXX_LIBRARIES}
)
# 安装可执行文件（同时安装两个节点）
install(TARGETS rs_lidar_deskew velodyne_lidar_deskew
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# 安装launch文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# 安装配置文件
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)