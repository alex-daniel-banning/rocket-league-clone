# List to keep track of all demo executable names
set(ALL_DEMO_NAMES "" CACHE INTERNAL "List of all demo executable names")

set(DEMO_SOURCES
    #rendering/ortho_shadows.cpp
    collisions/collisions.cpp
)

foreach(DEMO_SRC ${DEMO_SOURCES})
    get_filename_component(DEMO_NAME ${DEMO_SRC} NAME_WE)

    # Check for duplicate executable names
    if (${DEMO_NAME} IN_LIST ALL_DEMO_NAMES)
        message(FATAL_ERROR "Executable target '${DEMO_NAME}' already exists! Choose a different name.")
    endif()

    list(APPEND ALL_DEMO_NAMES ${DEMO_NAME})

    add_executable(${DEMO_NAME} ${DEMO_SRC})
    set_target_properties(${DEMO_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
    target_link_libraries(${DEMO_NAME} PRIVATE engine)
endforeach()
