cmake_minimum_required(VERSION 3.10)
project(ds4)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ds4 src/ds4.cpp)
ament_target_dependencies(ds4 rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  ds4
  DESTINATION lib/${PROJECT_NAME})
  
  
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
