cmake_minimum_required(VERSION 3.0)

project(bluerov_mavros)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs mavros)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs mavros)