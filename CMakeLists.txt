cmake_minimum_required(VERSION 2.8.3)
project(ekf_slam)
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_generation
  falkotest
  roscpp
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Pose2DWithCovariance.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ekf_slam
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(predict src/predict.cpp)
target_link_libraries(predict 
    ${catkin_LIBRARIES}
    )

add_executable(update src/update.cpp)
target_link_libraries(update 
    ${catkin_LIBRARIES}
    )

# This is to include the eigen library for matrix algebra. This might differ depending on the 
# system.
include_directories(
    "/usr/include/eigen3/"
    )
