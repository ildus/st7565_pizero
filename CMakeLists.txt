cmake_minimum_required(VERSION 3.16)
project(optrex)

set (CMAKE_CXX_STANDARD 20)
set (CMAKE_CXX_EXTENSIONS ON) # Same as gnu++2a (ON) vs c++2a (OFF): https://cmake.org/cmake/help/latest/prop_tgt/CXX_EXTENSIONS.html
set (CMAKE_CXX_STANDARD_REQUIRED ON)

set (CMAKE_C_STANDARD 11)
set (CMAKE_C_EXTENSIONS ON)
set (CMAKE_C_STANDARD_REQUIRED ON)

set (CMAKE_EXPORT_COMPILE_COMMANDS 1) # Write compile_commands.json

add_subdirectory(BCM2835)
add_subdirectory(ST7565)

add_executable(optrex
    optrex.cpp
)
target_link_libraries(optrex PRIVATE bcm2835 st7565)
target_include_directories(optrex PRIVATE ${st7565_SOURCE_DIR})
