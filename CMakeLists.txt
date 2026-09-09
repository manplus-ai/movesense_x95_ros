cmake_minimum_required(VERSION 3.10)
project(movesense_x95_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  vision_msgs
  geometry_msgs
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)
find_package(MoveSense_X95_SDK REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs vision_msgs geometry_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/movesense_x95_ros.cpp
  src/camera_driver.cpp
  src/stereo_calib.cpp
  src/stereo_receiver.cpp
  src/imu_receiver.cpp
  src/stereo_publisher.cpp
  src/imu_publisher.cpp
  src/tf_publisher.cpp
)

target_link_libraries(${PROJECT_NAME}
  MoveSense::X95_SDK
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
