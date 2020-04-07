cmake_minimum_required(VERSION 2.8.3)
project(hw4_0513336)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  visualization_msgs
  robot_pose_ekf
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_odom
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs visualization_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(hw4_node src/ekf_fusion.cpp)
target_link_libraries(hw4_node ${catkin_LIBRARIES})
