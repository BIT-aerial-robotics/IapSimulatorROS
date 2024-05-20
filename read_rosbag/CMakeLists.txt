cmake_minimum_required(VERSION 3.0.2)
project(read_rosbag)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  rosbag
  nav_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

AUX_SOURCE_DIRECTORY(. DIR_SRCS)
ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}) 
