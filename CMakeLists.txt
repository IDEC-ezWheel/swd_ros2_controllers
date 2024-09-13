cmake_minimum_required(VERSION 3.5)
project(swd_ros2_controllers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyFunctions.msg"
  "msg/WheelOdometry.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ------------------------------------------------------------------------------
# Project definition, variable and dependencies
# ------------------------------------------------------------------------------
set(CMAKE_PROJECT_NAME ${PROJECT_NAME})
set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_PROJECT_BRIEF "ez-Wheel SWD ROS2 Controllers package")

set(PKG_CONFIG_PATH /opt/ezw/usr/lib/pkgconfig CACHE PATH "")
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${PKG_CONFIG_PATH}")
message(STATUS "Pkgconfig extra path are: $ENV{PKG_CONFIG_PATH} ")

# ------------------------------------------------------------------------------
# pkgconfig
# ------------------------------------------------------------------------------
find_package(PkgConfig REQUIRED)

# ############
# # Clang ##
# ############

# Allow Clang-tidy
# set(CMAKE_CXX_CLANG_TIDY clang-tidy-12 --config-file=${CMAKE_SOURCE_DIR}/.clang-tidy --use-color --header-filter=${CMAKE_SOURCE_DIR}/include/)
# set(DCMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------------------------
# Project configuration
# ------------------------------------------------------------------------------
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/msg
  ${CMAKE_SOURCE_DIR}/msg/${PROJECT_NAME}
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Use project pkg config
pkg_check_modules(EZW_TYPES REQUIRED ezw-types)
pkg_check_modules(EZW_LOG REQUIRED ezwLog)
pkg_check_modules(EZW_SMC_SERVICE_CLIENT REQUIRED ezw-smc-service-client)

# Add include dirs. The case not configured will be empty
include_directories(
  ${EZW_TYPES_INCLUDE_DIRS}
  ${EZW_LOG_INCLUDE_DIRS}
  ${EZW_SMC_SERVICE_CLIENT_INCLUDE_DIRS}
)

# Add library dirs. The case not configured will be empty
link_directories(
  ${EZW_LOG_LIBRARY_DIRS}
  ${EZW_SMC_SERVICE_CLIENT_LIBRARY_DIRS}
)

# Add required definitions from pkgconfig
add_definitions(
  ${EZW_LOG_CFLAGS}
  ${EZW_LOG_LDFLAGS}
  ${EZW_SMC_SERVICE_CLIENT_CFLAGS}
  ${EZW_SMC_SERVICE_CLIENT_LDFLAGS}
)

# add_definitions( -DVERBOSE_OUTPUT )
file(GLOB_RECURSE SOURCES_DIFF_DRIVE_CRTL src/diff_drive_controller/*.cpp)

add_executable(swd_diff_drive_controller ${SOURCES_DIFF_DRIVE_CRTL})

ament_target_dependencies(swd_diff_drive_controller
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_msgs
  tf2_ros
  lifecycle_msgs
  rclcpp_lifecycle
)

# Get typesupport targets
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(
  swd_diff_drive_controller
  ${cpp_typesupport_target}
  ${EZW_LOG_LIBRARIES}
  ${EZW_SMC_SERVICE_CLIENT_LIBRARIES}
)

install(TARGETS
  swd_diff_drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
