cmake_minimum_required(VERSION 3.5)
project(number_queue LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)   # .ui 자동 처리
set(CMAKE_AUTORCC ON)

add_executable(callcenter_queue_qt5
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

target_link_libraries(callcenter_queue_qt5 PRIVATE Qt5::Widgets)
