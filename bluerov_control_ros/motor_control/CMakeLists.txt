cmake_minimum_required(VERSION 3.0)
project(motor_control)

# build the library 

add_library(${PROJECT_NAME} 
  src/MotorControl.cpp
)

# and link it
# target_link_libraries(${PROJECT_NAME} 
#   PUBLIC okvis_util
#   PUBLIC okvis_cv 
#   PUBLIC okvis_common
#   PRIVATE ${CERES_LIBRARIES} 
#   PRIVATE ${OpenCV_LIBRARIES} 
