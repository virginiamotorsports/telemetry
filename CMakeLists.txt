cmake_minimum_required(VERSION 3.5)
project(uva_telemetry)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(uva_iac_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory( proto )
set(PROTO_INCLUDE_DIR ${CMAKE_BINARY_DIR}/proto)
include_directories(
  include
  ${PROTO_INCLUDE_DIR}
)

add_executable(telemetry_bs src/telemetry_base_station.cpp)
ament_target_dependencies(telemetry_bs rclcpp deep_orange_msgs geometry_msgs uva_iac_msgs sensor_msgs)
target_link_libraries(telemetry_bs ${PROJECT_NAME}_proto)
add_executable(telemetry_rc src/telemetry_race_car.cpp)
ament_target_dependencies(telemetry_rc rclcpp deep_orange_msgs novatel_oem7_msgs nav_msgs raptor_dbw_msgs geometry_msgs sensor_msgs)
target_link_libraries(telemetry_rc ${PROJECT_NAME}_proto)

install(TARGETS
  telemetry_bs
  telemetry_rc
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
