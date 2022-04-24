cmake_minimum_required(VERSION 3.15)

project(physics_2d)
set(CMAKE_CXX_STANDARD 20)

add_subdirectory(external/glad)
add_subdirectory(external/glfw)
add_subdirectory(external/imgui)
find_package(Eigen3 REQUIRED NO_MODULE)

target_link_libraries(imgui PRIVATE glfw)

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src)
target_link_libraries(${PROJECT_NAME} PUBLIC glad glfw Eigen3::Eigen imgui)

# imguiexamples
add_executable(imguiexamples src/imguiexamples.cpp)
target_include_directories(imguiexamples PUBLIC src)
target_link_libraries(imguiexamples PUBLIC glad glfw Eigen3::Eigen imgui)