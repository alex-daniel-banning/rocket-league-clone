cmake_minimum_required(VERSION 3.10)

project(rocket-league-clone)

add_library(engine STATIC
    src/engine/Camera.cpp
)

target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(render-car)
