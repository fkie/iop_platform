cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_digital_resource_discovery)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
                    fkie_iop_events
                    fkie_iop_msgs
                    roscpp)
iop_init(COMPONENT_ID 0)
iop_export_service(urn_jaus_jss_iop_DigitalResourceDiscovery
                   urn_jaus_jss_iop_DigitalResourceDiscoveryClient)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include/public
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        fkie_iop_events
        fkie_iop_msgs
        roscpp
)

###########
## Build ##
###########
iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/Events.xml
    urn.jaus.jss.core-v1.1/EventsClient.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.iop/DigitalResourceDiscovery_v1.xml
    urn.jaus.jss.iop/DigitalResourceDiscoveryClient_v1.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_iop_DigitalResourceDiscovery/DigitalResourceDiscovery_ReceiveFSM.h
    include/urn_jaus_jss_iop_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClient_ReceiveFSM.h
    src/urn_jaus_jss_iop_DigitalResourceDiscovery/DigitalResourceDiscovery_ReceiveFSM.cpp
    src/urn_jaus_jss_iop_DigitalResourceDiscoveryClient/DigitalResourceDiscoveryClient_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_EventsClient
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(${catkin_INCLUDE_DIRS}
                    include/public
)
add_library(${PROJECT_NAME} 
            src/DigitalResourceDiscoveryClientPlugin.cpp
            src/DigitalResourceDiscoveryPlugin.cpp
            src/DigitalResourceEndpoint.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(
  DIRECTORY ${IOP_INSTALL_INCLUDE_DIRS} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN "*.old" EXCLUDE
  PATTERN "*.gen" EXCLUDE
)

install(
  DIRECTORY include/public/${PROJECT_NAME} DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
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
