cmake_minimum_required(VERSION 3.10)

project(CLab1_All_Tasks)

set(CMAKE_CXX_STANDARD 17)


find_package(Eigen3 REQUIRED)

add_executable(wall-e wall-e.cpp)
target_link_libraries(wall-e Eigen3::Eigen)