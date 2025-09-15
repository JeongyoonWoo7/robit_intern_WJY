cmake_minimum_required(VERSION 3.8)
project(vector_pubsub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp custom_interfaces)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp custom_interfaces)

install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
