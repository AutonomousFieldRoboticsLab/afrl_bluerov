cmake_minimum_required(VERSION 3.0)

project(bluerov_control_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS 
            roscpp
            sensor_msgs
            mavros
            geometry_msgs
            tf2_ros
            tf)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs mavros joy tf2_ros tf)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

find_package(Eigen3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# include_directories(motor_control/include)
# add_subdirectory(motor_control)

add_library(bluerov_control 
            src/Primitives.cpp
            src/Utils.cpp 
            src/MotorControl.cpp)

# target_link_libraries(bluerov_control motor_control)

add_executable(bluerov_primitive_node src/bluerov_primitives_node.cpp)
target_link_libraries(bluerov_primitive_node bluerov_control ${catkin_LIBRARIES})

add_executable(bluerov_tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(bluerov_tf_broadcaster bluerov_control ${catkin_LIBRARIES})

add_executable(bluerov_transect_node src/bluerov_transect_node.cpp)
target_link_libraries(bluerov_transect_node bluerov_control ${catkin_LIBRARIES})

add_executable(bluerov_square_node src/bluerov_square_node.cpp)
target_link_libraries(bluerov_square_node bluerov_control ${catkin_LIBRARIES})

add_executable(bluerov_lawnmower_node src/bluerov_lawnmower_node.cpp)
target_link_libraries(bluerov_lawnmower_node bluerov_control ${catkin_LIBRARIES})

add_executable(test_node src/test_absolute_orientation.cpp)
target_link_libraries(test_node bluerov_control ${catkin_LIBRARIES})
#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/bluerov_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
   scripts/bluerov_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
