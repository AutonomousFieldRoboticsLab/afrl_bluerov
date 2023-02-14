cmake_minimum_required(VERSION 3.0)

project(bluerov_control_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-march=native -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS 
            roscpp
            sensor_msgs
            mavros)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs mavros joy)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(motor_control/include)
add_subdirectory(motor_control)

add_library(bluerov_control src/Primitives.cpp src/VelocityControl.cpp)

add_executable(bluerov_primitive_node src/bluerov_primitives_node.cpp)
TARGET_LINK_LIBRARIES(bluerov_primitive_node bluerov_control motor_control ${catkin_LIBRARIES})
#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/bluerov_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
   scripts/bluerov_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
