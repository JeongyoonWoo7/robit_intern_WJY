cmake_minimum_required(VERSION 3.10)
project(turtle_gui)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)


set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC ON)


set(UI_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/resource/mainwindow.ui
)
qt5_wrap_ui(UI_HEADERS ${UI_FILES})


set(MOC_HEADERS
  include/turtle_gui/mainwindow.hpp
)
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})


add_executable(turtle_gui
  src/main.cpp
  src/mainwindow.cpp
  ${UI_HEADERS}
  ${MOC_SOURCES}
)


target_include_directories(turtle_gui PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
)


ament_target_dependencies(turtle_gui
  rclcpp
  geometry_msgs
  std_srvs
)


target_link_libraries(turtle_gui
  Qt5::Widgets
)


install(TARGETS turtle_gui
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

