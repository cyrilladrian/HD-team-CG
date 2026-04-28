cmake_minimum_required(VERSION 3.10)
project(CG_Animation)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Original static-export program (unchanged)
add_executable(wall-e wall-e.cpp)
target_link_libraries(wall-e Eigen3::Eigen)

# New animation program
add_executable(walking_robot walking_robot.cpp)
target_link_libraries(walking_robot
    Eigen3::Eigen
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
)
