# Code Coverage

## Setup

### CMake option

Add a coverage build option:

```cmake
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)

if(ENABLE_COVERAGE)
  add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
  add_link_options(--coverage)
endif()
```

Configure with:
```bash
cmake -B build -DENABLE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

### Dependencies (Ubuntu)

```bash
sudo apt install lcov gcovr
```

## Generating reports

### lcov + genhtml (HTML reports)

```bash
# Run tests
cd build && ctest

# Capture coverage data
lcov --capture --directory . --output-file coverage.info

# Filter out system/test files
lcov --remove coverage.info '/usr/*' '*/test/*' '*/googletest/*' --output-file coverage_filtered.info

# Generate HTML report
genhtml coverage_filtered.info --output-directory coverage_report
```

Open `coverage_report/index.html` in a browser.

### gcovr (simpler alternative)

```bash
gcovr -r .. --html --html-details -o coverage.html
```

## Alternatives

- **llvm-cov** — if using Clang, pair with `llvm-profdata` for source-based coverage (more accurate than gcov)
- **codecov.io** or **coveralls** — for CI integration, upload the `coverage.info` file
