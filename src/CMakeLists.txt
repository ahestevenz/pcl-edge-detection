cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pclVisualizer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pclVisualizer main.cpp)
target_link_libraries (pclVisualizer ${PCL_LIBRARIES}) 
