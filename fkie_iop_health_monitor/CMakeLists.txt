cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_health_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
                    fkie_iop_accesscontrol
                    roscpp)
iop_init(COMPONENT_ID 0)
iop_export_service(
    urn_jaus_jss_exp_aeodrs_HealthMonitor
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        fkie_iop_accesscontrol
)

###########
## Build ##
###########
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/Events.xml
    urn.jaus.jss.core-v1.1/EventsClient.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.core-v1.1/AccessControl.xml
    urn.jaus.jss.core-v1.1/AccessControlClient.xml
    urn.jaus.jss.iop/HealthMonitor_V1.4.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_exp_aeodrs_HealthMonitor/HealthMonitor_ReceiveFSM.h
    src/urn_jaus_jss_exp_aeodrs_HealthMonitor/HealthMonitor_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
      urn_jaus_jss_core_AccessControl
      urn_jaus_jss_core_AccessControlClient
      urn_jaus_jss_core_Events
      urn_jaus_jss_core_EventsClient
      urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
            src/HealthMonitorPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
)

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
