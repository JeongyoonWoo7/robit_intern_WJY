cmake_minimum_required(VERSION 3.10)
project(communicator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC ON)


set(UI_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/resource/mainwindow.ui
)
qt5_wrap_ui(UI_HEADERS ${UI_FILES})


set(MOC_HEADERS
  include/communicator/mainwindow.hpp
  include/communicator/qnode.hpp
)
qt5_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})


add_executable(${PROJECT_NAME}
  src/main.cpp
  src/mainwindow.cpp
  src/qnode.cpp
  ${UI_HEADERS}
  ${MOC_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets Qt5::Core Qt5::Gui
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

