cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_platform_manager)

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