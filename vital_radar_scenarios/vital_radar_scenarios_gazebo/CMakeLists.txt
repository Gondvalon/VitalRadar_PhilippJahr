cmake_minimum_required(VERSION 3.0.2)
project(vital_radar_scenarios_gazebo)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_onboard_launch
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

