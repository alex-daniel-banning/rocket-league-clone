# Build Guide

## Standard Build Process

### Initial build or after CMake changes
When building for the first time or after modifying CMakeLists.txt (e.g., adding new files):
```bash
mkdir -p build
cd build
cmake ..
cmake --build .
```

### Incremental builds
For subsequent builds when only source files have changed:
```bash
cd build
cmake --build .
```

## LSP Setup

### Generate compile_commands.json
Required for clang LSP support:
```bash
cd build
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..
```

### Link to project root
Run from the project root directory:
```bash
ln -sf build/compile_commands.json .
```

---

## Branching Strategy

**Workflow for experimental features:**

1. **Branch from master** using descriptive names (e.g., `experiment/shadow-mapping`)
2. **Experiment freely**—break things, try ideas, iterate rapidly
3. **Extract reusable code**—when the problem is solved, identify engine-level components worth keeping
4. **Merge selectively**—only merge the clean, reusable engine code back to master
5. **Tag the branch**—when the experiment has a working demo worth preserving
6. **Clean up**—delete the experimental branch once it's been mined for value

This approach keeps master stable while allowing aggressive experimentation in branches.
