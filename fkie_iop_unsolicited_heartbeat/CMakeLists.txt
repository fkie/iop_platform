cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_unsolicited_heartbeat)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  fkie_iop_liveness
)

iop_init(COMPONENT_ID 0)
iop_export_service(urn_jaus_jss_iop_UnsolicitedHeartbeat)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS fkie_iop_liveness
)

iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/Liveness.xml
    urn.jaus.jss.core-v1.1/Events.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.iop/UnsolicitedHeartbeat.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_iop_UnsolicitedHeartbeat/UnsolicitedHeartbeat_ReceiveFSM.h
    src/urn_jaus_jss_iop_UnsolicitedHeartbeat/UnsolicitedHeartbeat_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_Liveness
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
            src/UnsolicitedHeartbeatPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

