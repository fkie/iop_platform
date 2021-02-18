cmake_minimum_required(VERSION 3.5)
project(fkie_iop_platform_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-ignored-qualifiers)
endif()

find_package(ament_cmake REQUIRED)
find_package(fkie_iop_builder REQUIRED)
find_package(fkie_iop_digital_resource_discovery REQUIRED)
find_package(fkie_iop_discovery REQUIRED)
find_package(fkie_iop_health_monitor REQUIRED)
find_package(fkie_iop_liveness REQUIRED)
find_package(fkie_iop_management REQUIRED)
find_package(fkie_iop_platform_mode REQUIRED)
find_package(fkie_iop_platform_state REQUIRED)
find_package(fkie_iop_unsolicited_heartbeat REQUIRED)


install(
   DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

ament_package()