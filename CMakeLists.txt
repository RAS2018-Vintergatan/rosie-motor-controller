cmake_minimum_required(VERSION 2.8.3)
project(rosie_motor_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs phidgets geometry_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp std_msgs phidgets geometry_msgs
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_motor_controller src/motor_controller_node.cpp)
target_link_libraries(rosie_motor_controller ${catkin_LIBRARIES})
add_dependencies(rosie_motor_controller ${catkin_EXPORTED_TARGETS})
#std_msgs phidgets geometry_msgs
