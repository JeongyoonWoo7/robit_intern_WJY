cmake_minimum_required(VERSION 3.10)
project(joysim)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(joysim src/joysim.cpp)
ament_target_dependencies(joysim rclcpp geometry_msgs tf2 tf2_ros)

install(TARGETS
  joysim
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
