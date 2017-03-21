cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(tape_on_registration)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_STANDARD 11)
set(SOURCE_FILES main.cpp)
add_executable(tape_on_registration ${SOURCE_FILES})

target_link_libraries(tape_on_registration ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})