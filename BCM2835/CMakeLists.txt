cmake_minimum_required(VERSION 3.16)
project(bcm2835 LANGUAGES C)

set (CMAKE_C_STANDARD 11)
set (CMAKE_C_EXTENSIONS ON)
set (CMAKE_C_STANDARD_REQUIRED ON)

set (CMAKE_EXPORT_COMPILE_COMMANDS 1) # Write compile_commands.json

add_library(bcm2835 STATIC
    bcm2835.c
)
target_include_directories(bcm2835 PUBLIC ${bcm2835_SOURCE_DIR})
