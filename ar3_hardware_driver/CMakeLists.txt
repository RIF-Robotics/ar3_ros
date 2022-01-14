cmake_minimum_required(VERSION 3.8)
project(ar3_hardware_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/ar3_encoder_switch_motor_serial_comm.cpp
  )
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  )
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  )

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
  )

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

## EXPORTS
ament_export_include_directories(
  include
  )
ament_export_libraries(
  ${PROJECT_NAME}
  )
ament_export_dependencies(
  rclcpp
  )

ament_package()
