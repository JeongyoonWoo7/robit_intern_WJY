cmake_minimum_required(VERSION 3.5)

project(3dof_robot_arm LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(3dof_robot_arm
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(3dof_robot_arm PRIVATE Qt5::Widgets)
