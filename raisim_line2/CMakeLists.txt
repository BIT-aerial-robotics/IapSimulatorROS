cmake_minimum_required(VERSION 3.10)
project(line2single)

set(ENV_VAR /home/siris)
list(APPEND CMAKE_PREFIX_PATH ~/raisimlib/raisim/linux)
list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  message_generation
  rosbag
)

find_package(raisim CONFIG REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

add_message_files(
  FILES
  AvaSlamData.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

aux_source_directory(src CPP_LIST)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  ${CPP_LIST}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#find cppzmq wrapper, installed by make of cppzmq
find_package(cppzmq)
target_link_libraries(${PROJECT_NAME} cppzmq)

target_link_libraries(${PROJECT_NAME} raisim::raisim pthread)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
