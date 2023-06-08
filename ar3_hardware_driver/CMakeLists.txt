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

# Serial Library
add_library(
  timeout_serial
  SHARED
  src/timeout_serial.cpp
)
target_include_directories(timeout_serial
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  timeout_serial
  rclcpp
)

# Comm protocol to Arduino
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
target_link_libraries(
  ${PROJECT_NAME}
  timeout_serial
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
)

# INSTALL
install(
  TARGETS timeout_serial
  EXPORT timeout_serialTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format()

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8()

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257()

  find_package(ament_cmake_mypy REQUIRED)
  ament_mypy()

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
endif()

# Serial Library Exports
ament_export_targets(timeout_serialTargets HAS_LIBRARY_TARGET)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

## EXPORTS
ament_export_include_directories(
  include
)
#ament_export_libraries(
#  ${PROJECT_NAME}
#  )
ament_export_dependencies(
  rclcpp
)

ament_package()
