cmake_minimum_required(VERSION 3.10)
project(iap3spherical)

set(ENV_VAR /home/siris) # path to home directory
list(APPEND CMAKE_PREFIX_PATH ~/raisimlib/raisim/linux) # path to installed raisim packages
list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic) # path to ros

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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# **************************** line3 **************************** #
add_executable(line3single
  src/line3single.cpp
  src/BasicTypes.cpp
  src/StatesSender.cpp
  src/CommandsReceiver.cpp
)

target_link_libraries(line3single
  ${catkin_LIBRARIES}
)

find_package(cppzmq)
target_link_libraries(line3single cppzmq)

target_link_libraries(line3single raisim::raisim pthread)
target_include_directories(line3single PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# **************************** line3 **************************** #

# **************************** square3 **************************** #
add_executable(square3single
  src/square3single.cpp
  src/BasicTypes.cpp
  src/StatesSender.cpp
  src/CommandsReceiver.cpp
)

target_link_libraries(square3single
  ${catkin_LIBRARIES}
)

find_package(cppzmq)
target_link_libraries(square3single cppzmq)

target_link_libraries(square3single raisim::raisim pthread)
target_include_directories(square3single PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# **************************** square3 **************************** #

# **************************** star3 **************************** #
add_executable(star3single
src/star3single.cpp
src/BasicTypes.cpp
src/StatesSender.cpp
src/CommandsReceiver.cpp
)

target_link_libraries(star3single
${catkin_LIBRARIES}
)

find_package(cppzmq)
target_link_libraries(star3single cppzmq)

target_link_libraries(star3single raisim::raisim pthread)
target_include_directories(star3single PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# **************************** star3 **************************** #

# **************************** star3 admittance **************************** #
add_executable(star3admittance
src/star3admittance.cpp
src/BasicTypes.cpp
src/StatesSender.cpp
src/CommandsReceiver.cpp
)

target_link_libraries(star3admittance
${catkin_LIBRARIES}
)

find_package(cppzmq)
target_link_libraries(star3admittance cppzmq)

target_link_libraries(star3admittance raisim::raisim pthread)
target_include_directories(star3admittance PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# **************************** star3 admittance **************************** #

# **************************** star3 admittance **************************** #
add_executable(star3combined
src/star3combined.cpp
src/BasicTypes.cpp
src/StatesSender.cpp
src/CommandsReceiver.cpp
)

target_link_libraries(star3combined
${catkin_LIBRARIES}
)

find_package(cppzmq)
target_link_libraries(star3combined cppzmq)

target_link_libraries(star3combined raisim::raisim pthread)
target_include_directories(star3combined PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# **************************** star3 admittance **************************** #
