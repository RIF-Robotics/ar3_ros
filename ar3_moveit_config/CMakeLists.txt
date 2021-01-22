cmake_minimum_required(VERSION 3.10.2)
project(ar3_moveit_config)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
  )
