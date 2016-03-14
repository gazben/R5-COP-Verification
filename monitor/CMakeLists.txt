cmake_minimum_required(VERSION 2.8.3)
project(--monitor_name--)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS roscpp turtlesim)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  /opt/ros/jade/include
)

add_executable(
  mission_cntrl_groovy
  src/mission_cntrl_groovy.cpp
)


add_executable(
  --monitor_name--
  src/main.cpp
  src/monitor_test.cpp
  src/monitor/events.h
  src/monitor/output_state.h
  src/monitor/output_state.cpp
  src/monitor/property.h
  src/monitor/property.cpp
  src/monitor/state_register.h
  src/monitor/state_register.cpp
)

target_link_libraries(mission_cntrl_groovy ${catkin_LIBRARIES})
target_link_libraries(--monitor_name-- ${catkin_LIBRARIES})



