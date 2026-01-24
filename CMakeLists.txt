cmake_minimum_required(VERSION 3.10)

project(rocket-league-clone)

add_library(engine STATIC
    src/engine/glad.c
    src/engine/stb_image.cpp
    src/engine/Match.cpp
    src/engine/physics/Collisions.cpp
    src/engine/render/Camera.cpp
    src/engine/render/Shader.cpp
    src/engine/render/Mesh.cpp
    src/engine/render/Model.cpp
    src/engine/render/Renderer.cpp
    src/engine/physics/Plane.cpp
)

target_include_directories(engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include /usr/include)

add_subdirectory(src/demos)

# Find packages
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(assimp REQUIRED)

# Link GLFW and OpenGL
target_link_libraries(engine PRIVATE glfw OpenGL::GL)
target_link_libraries(engine PRIVATE assimp::assimp)

# Enable testing
enable_testing()

# Create test executable
add_executable(tests 
    tests/test_main.cpp
    tests/test_Plane.cpp
    tests/test_Collisions.cpp
    tests/TestUtil.cpp)
target_link_libraries(tests PRIVATE engine gtest)
target_include_directories(tests PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/tests/include)

# Register tests
include(GoogleTest)
gtest_discover_tests(tests)
