cmake_minimum_required(VERSION 3.10)
project(render-car)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find GLFW if installed locally
# set(CMAKE_PREFIX_PATH="$HOME/.local")   # uncomment if you installed in home

# Find packages
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

# Add your executable
add_executable(render-car
    src/main.cpp
    src/glad.c
    include/Shader.hpp
)

target_include_directories(render-car PRIVATE include)

# Link GLFW and OpenGL
target_link_libraries(render-car PRIVATE glfw OpenGL::GL)

# Copy the entire resources folder (recursively) into the build folder
file(COPY "${PROJECT_SOURCE_DIR}/resources"
     DESTINATION "${CMAKE_BINARY_DIR}")
