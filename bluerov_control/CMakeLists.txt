cmake_minimum_required(VERSION 3.0)

project(bluerov_control)

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
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############


catkin_install_python(PROGRAMS scripts/bluerov_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
   scripts/bluerov_teleop.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
