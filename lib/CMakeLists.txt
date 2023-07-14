cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

project(WEH001602A)

add_library(WEH001602A STATIC WEH001602A.cpp)

target_link_libraries(WEH001602A pico_stdlib hardware_i2c)
