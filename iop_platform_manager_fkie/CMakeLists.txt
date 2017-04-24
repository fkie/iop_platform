cmake_minimum_required(VERSION 2.8.3)
project(iop_platform_manager_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package()

install(
   FILES launch/platform_manager.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)