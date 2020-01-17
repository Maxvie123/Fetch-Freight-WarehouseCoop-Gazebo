cmake_minimum_required(VERSION 2.8.3)
project(warehousetest)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    rospy
    message_generation
)


## Generate services in the 'srv' folder
add_service_files(
   FILES
   GetGoal.srv
   GetGripperGoal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES warehousetest
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

