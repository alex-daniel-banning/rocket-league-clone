# Logging System

## Overview

A lightweight, custom logging system using macros and a global log level.
Replace scattered `std::cout` debug statements with leveled log calls that can
be toggled at runtime.

## Implementation

```cpp
// log.hpp
#pragma once
#include <cstdio>

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

// Without `inline`, if two .cpp files both #include this header, each gets
// its own definition of g_log_level. The linker sees two symbols with the
// same name and fails with a "multiple definition" error.
//
// `inline` (C++17 for variables) tells the linker: "these are all the same
// variable — pick one and share it." Every translation unit sees and modifies
// the same g_log_level instance.
inline LogLevel g_log_level = LogLevel::INFO;

// Macros perform text substitution at compile time. Each LOG_X call expands
// to an fprintf guarded by a level check. ##__VA_ARGS__ handles the case
// where no extra arguments are passed (strips the trailing comma).

#define LOG_DEBUG(fmt, ...) \
    if (LogLevel::DEBUG >= g_log_level) fprintf(stderr, "[DEBUG] " fmt "\n", ##__VA_ARGS__)

#define LOG_INFO(fmt, ...) \
    if (LogLevel::INFO >= g_log_level) fprintf(stderr, "[INFO]  " fmt "\n", ##__VA_ARGS__)

#define LOG_WARN(fmt, ...) \
    if (LogLevel::WARN >= g_log_level) fprintf(stderr, "[WARN]  " fmt "\n", ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...) \
    if (LogLevel::ERROR >= g_log_level) fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__)
```

## Usage

```cpp
#include "log.hpp"

// Set level at startup
g_log_level = LogLevel::DEBUG;  // see everything
g_log_level = LogLevel::WARN;   // only warnings and errors

// Log calls (C-style format specifiers: %f, %d, %s, etc.)
LOG_DEBUG("penetration=%.4f normal=(%.2f, %.2f, %.2f)", penetration, n.x, n.y, n.z);
LOG_INFO("collision detected between box %d and box %d", id_a, id_b);
LOG_WARN("large impulse: %.2f", impulse_scalar);
LOG_ERROR("two immovable objects in collision resolution");
```

## What to Log (Physics)

Focus on **boundaries and decision points**, not every calculation.

- **Contact detection**: which objects, penetration depth, normal, contact points
- **Resolution**: impulse magnitude, velocity changes, penetration correction amounts
- **Anomalies**: near-zero denominators, unusually large impulses, objects still penetrating after correction

## Future Improvements

- If needs outgrow this, swap in `spdlog` (fast, fmt-based formatting, file sinks, async logging). Log call sites would look similar, so migration is straightforward.
