cmake_minimum_required(VERSION 3.10)
project(HelloTriangle)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find GLFW if installed locally
# set(CMAKE_PREFIX_PATH="$HOME/.local")   # uncomment if you installed in home

# Find packages
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)

# Add your executable
add_executable(render-car src/main.cpp)

# Link GLFW and OpenGL
target_link_libraries(render-car PRIVATE glfw OpenGL::GL)
