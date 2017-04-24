cmake_minimum_required(VERSION 2.8.3)
project(iop_health_monitor_fkie)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
                    iop_accesscontrol_1_0_fkie
                    iop_builder_fkie
                    iop_component_fkie
                    roscpp)
iop_init(COMPONENT_ID 0)
iop_export_service(
    urn_jaus_jss_exp_aeodrs_HealthMonitor_1_2
    urn_jaus_jss_exp_aeodrs_HealthMonitorClient_1_2
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include/${PROJECT_NAME}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        iop_accesscontrol_1_0_fkie
        iop_builder_fkie
        iop_component_fkie
)

###########
## Build ##
###########
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.0/Events.xml
    urn.jaus.jss.core-v1.0/EventsClient.xml
    urn.jaus.jss.core-v1.0/Transport.xml
    urn.jaus.jss.core-v1.0/AccessControl.xml
    urn.jaus.jss.core-v1.0/AccessControlClient.xml
    urn.jaus.jss.iop/HealthMonitor_V1.xml
    urn.jaus.jss.iop/HealthMonitorClient_V1.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_exp_aeodrs_HealthMonitor_1_2/HealthMonitor_ReceiveFSM.h
    src/urn_jaus_jss_exp_aeodrs_HealthMonitor_1_2/HealthMonitor_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
      urn_jaus_jss_core_AccessControl_1_0
      urn_jaus_jss_core_AccessControlClient_1_0
      urn_jaus_jss_core_Events_1_0
      urn_jaus_jss_core_EventsClient_1_0
      urn_jaus_jss_core_Transport_1_0
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${IOP_INCLUDE_DIRS}
                    include/${PROJECT_NAME}
)
add_library(${PROJECT_NAME}
            src/HealthMonitorPlugin_1_2.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${IOP_LIBRARIES}
                      ${Boost_LIBRARIES}
)

install(
  DIRECTORY include/${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
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
