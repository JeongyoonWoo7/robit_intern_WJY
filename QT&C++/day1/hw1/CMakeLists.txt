cmake_minimum_required(VERSION 3.10)
project(day1_hw1)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(test
    src/main.cpp
    src/Calculation.cpp
)
