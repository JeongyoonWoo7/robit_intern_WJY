cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름
project(PointDistance)

# C++17 표준 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# include 디렉토리 등록
include_directories(include)

# 실행 파일 생성 
add_executable(test
    src/main.cpp
    src/Points.cpp
)
