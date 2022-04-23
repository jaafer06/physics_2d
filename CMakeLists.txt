cmake_minimum_required(VERSION 3.15)

project(physics_2d)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(external/olcPixelGameEngine)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen olcPixelGameEngine)
