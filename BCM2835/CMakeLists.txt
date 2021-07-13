cmake_minimum_required(VERSION 3.16)
project(bcm2835 LANGUAGES C)

set (CMAKE_C_STANDARD 11)
set (CMAKE_C_EXTENSIONS ON)
set (CMAKE_C_STANDARD_REQUIRED ON)

set (CMAKE_EXPORT_COMPILE_COMMANDS 1) # Write compile_commands.json

find_package(PkgConfig REQUIRED)
pkg_check_modules(CAP REQUIRED IMPORTED_TARGET libcap)

add_library(bcm2835 STATIC
    bcm2835.c
)
target_include_directories(bcm2835 PUBLIC ${bcm2835_SOURCE_DIR})
target_compile_options(bcm2835 PRIVATE -DBCM2835_HAVE_LIBCAP)
target_link_libraries(bcm2835 PRIVATE PkgConfig::CAP)
