cmake_minimum_required(VERSION 2.8.3)
project(abb_projector)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
)

#find_package(orocos-kdl REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  project_srv.srv
  sample_srv.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

include_directories(
  include/
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  /home/avishai/Documents/eigen
)

link_directories(/usr/local/include
/home/avishai/Documents/eigen
)

add_executable(abb_projector src/projector_node.cpp src/kdl_class_abb.cpp src/projector.cpp)
add_executable(test src/test.cpp src/kdl_class_abb.cpp)

target_link_libraries(abb_projector
  ${catkin_LIBRARIES}
  orocos-kdl
  /home/avishai/Documents/eigen
)

target_link_libraries(test
  ${catkin_LIBRARIES}
  orocos-kdl
  /home/avishai/Documents/eigen
)