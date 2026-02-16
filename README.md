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

### Debug logging
To build with debug-level logging:
```bash
cmake -B build -DLOG_LEVEL=DEBUG
cmake --build build
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
### Execute a single test
Run from the build directory:
```bash
./tests --gtest_filter=TestSuiteName.TestName
```

---

## Testing Strategy

**Unit tests** for deterministic components:
- Math calculations
- Collision detection
- Integrators
- Other pure functions

**Scene-based tests** for complex interactions:
- Create deterministic test scenes to verify behavior
- Useful for physics interactions, rendering pipelines, and system integration
