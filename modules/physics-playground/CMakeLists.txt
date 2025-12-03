cmake_minimum_required(VERSION 3.10)
project(physics-playground)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find GLFW if installed locally
# set(CMAKE_PREFIX_PATH="$HOME/.local")   # uncomment if you installed in home

# Add your executable
add_executable(physics-playground
    src/main.cpp
)

target_include_directories(physics-playground PRIVATE include)

target_link_libraries(physics-playground PRIVATE engine)

# # Copy the entire resources folder (recursively) into the build folder
# file(COPY "${PROJECT_SOURCE_DIR}/resources"
#      DESTINATION "${CMAKE_BINARY_DIR}/physics-playground")
