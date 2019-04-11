cmake_minimum_required(VERSION 2.8.3)
project(rm_control_demo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robot_protocol_msgs
)
catkin_package(
)
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/main.cpp src/key.cpp)
add_dependencies(${PROJECT_NAME}_node robot_protocol_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)