cmake_minimum_required(VERSION 3.5)
project(prob_rob_vision)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(prob_rob_msgs REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${prob_rob_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_processor src/main.cpp src/video_processor.cpp)

ament_target_dependencies(video_processor
  rclcpp
  sensor_msgs
  std_msgs
  prob_rob_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS
  video_processor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
