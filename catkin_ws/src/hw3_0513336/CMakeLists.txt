cmake_minimum_required(VERSION 2.8.3)
project(hw3_0513336)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  visualization_msgs
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
add_executable(hw3_node src/imu_odom.cpp)
target_link_libraries(hw3_node ${catkin_LIBRARIES})
