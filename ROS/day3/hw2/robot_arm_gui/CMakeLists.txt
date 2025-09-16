cmake_minimum_required(VERSION 3.16)
project(robot_arm_gui LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 COMPONENTS Widgets REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_arm_gui
    src/main.cpp
    src/mainwindow.cpp
    src/mainwindow.ui
    include/robot_arm_gui/mainwindow.h
)

target_include_directories(robot_arm_gui PRIVATE
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(robot_arm_gui Qt6::Widgets)

ament_target_dependencies(robot_arm_gui rclcpp std_msgs)

install(TARGETS robot_arm_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()

