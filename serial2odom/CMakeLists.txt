cmake_minimum_required(VERSION 2.8.3)
project(serial2odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  tf
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    tf
    nav_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial2odom_node src/serial2odom_node.cpp)

target_link_libraries(serial2odom_node
  ${catkin_LIBRARIES}
)

