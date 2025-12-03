cmake_minimum_required(VERSION 3.10)

project(rocket-league-clone)

add_library(engine STATIC
    src/engine/glad.c
    src/engine/stb_image.cpp
    src/engine/render/Camera.cpp
    src/engine/render/Shader.cpp
    src/engine/render/Mesh.cpp
    src/engine/render/Model.cpp
)

target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(render-car)

# Find packages
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(assimp REQUIRED)

# Link GLFW and OpenGL
target_link_libraries(engine PRIVATE glfw OpenGL::GL)
target_link_libraries(engine PRIVATE assimp::assimp)
