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
#ifndef DEFAULT_LOG_LEVEL
#define DEFAULT_LOG_LEVEL LogLevel::INFO
#endif

inline LogLevel g_log_level = DEFAULT_LOG_LEVEL;

// Macros perform text substitution at compile time. Each LOG_X call expands
// to an fprintf guarded by a level check. ##__VA_ARGS__ handles the case
// where no extra arguments are passed (strips the trailing comma).
//
// do { } while(0) wraps the macro into a single statement so it behaves
// correctly in if/else chains (avoids the dangling-else bug).
#define LOG_DEBUG(fmt, ...) \
  do { if (LogLevel::DEBUG >= g_log_level) fprintf(stderr, "[DEBUG] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_INFO(fmt, ...) \
  do { if (LogLevel::INFO >= g_log_level) fprintf(stderr, "[INFO] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_WARN(fmt, ...) \
  do { if (LogLevel::WARN >= g_log_level) fprintf(stderr, "[WARN] " fmt "\n", ##__VA_ARGS__); } while(0)

#define LOG_ERROR(fmt, ...) \
  do { if (LogLevel::ERROR >= g_log_level) fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__); } while(0)
