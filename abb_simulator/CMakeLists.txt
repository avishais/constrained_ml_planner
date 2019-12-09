cmake_minimum_required(VERSION 2.8.3)
project(abb_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

include_directories(
  include/
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  /home/avishai/Documents/PQP_v1.3/include
)

link_directories(/home/avishai/Documents/PQP_v1.3/lib)

add_executable(abb_simulator src/simulator_node.cpp src/simulator.cpp)

target_link_libraries(abb_simulator
  ${catkin_LIBRARIES}
  PQP
)