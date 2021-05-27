cmake_minimum_required(VERSION 3.20)
project(st7565)

add_library(st7565 STATIC
	ST7565.cpp
    glcdfont.c
)
target_link_libraries(st7565 PRIVATE bcm2835)
target_include_directories(st7565 PUBLIC ${ST7565_SOURCE_DIR})
