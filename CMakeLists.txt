cmake_minimum_required(VERSION 3.0.2)
project(uwarl_test_suite)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_2_pathing src/test_2_pathing.cpp)
add_executable(test_echo src/test_echo.cpp)

target_link_libraries(test_2_pathing ${catkin_LIBRARIES})
target_link_libraries(test_echo ${catkin_LIBRARIES})
