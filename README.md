Commands
To build whole project -> "cmake --build build" from project root
To build a specific module -> "cmake --build build --target <module name>"

To install glfw3 library -> "sudo apth install libglfw3 libglfw3-dev"

Build debug -> "cmake -B build -DCMAKE_BUILD_TYPE=Debug"
:lua require'dap'.toggle_breakpoint()
:lua require'dap'.continue()
