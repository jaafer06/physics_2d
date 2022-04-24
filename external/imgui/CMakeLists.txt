cmake_minimum_required(VERSION 3.15)
project(imgui)
file(GLOB source ./*.cpp)

add_library(imgui ${source})
target_include_directories(imgui PUBLIC ./)