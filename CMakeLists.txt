cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(tape_on_registration)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SOURCE_FILES main.cpp)
add_executable(tape_on_registration ${SOURCE_FILES})

target_link_libraries(tape_on_registration ${PCL_LIBRARIES})