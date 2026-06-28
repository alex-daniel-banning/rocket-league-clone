#pragma once
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

#include "print.hpp"  // for the glm::vec3 type

namespace Debug {

namespace detail {

// Round to the nearest thousandth (3 decimals). Exact zero renders as "0"
// rather than "0.000"; other values keep 3 decimal places.
inline std::string FormatScalar(double v) {
  v = std::round(v * 1000.0) / 1000.0;
  if (v == 0.0) {
    return "0";
  }
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << v;
  return os.str();
}

inline std::string Format(const glm::vec3& value) {
  return "(" + FormatScalar(value.x) + ", " + FormatScalar(value.y) + ", " + FormatScalar(value.z) + ")";
}

template <typename T>
std::string Format(const T& value) {
  if constexpr (std::is_floating_point_v<T>) {
    return FormatScalar(static_cast<double>(value));
  } else {
    std::ostringstream os;
    os << value;
    return os.str();
  }
}

}  // namespace detail

// Print "variable_name = value" on its own line. Floating-point values (including
// glm::vec3 components) are rounded to the nearest thousandth, with exact zero
// shown as "0". Works for any type with operator<< (int, std::string, ...).
template <typename T>
void Print(const std::string& variable_name, const T& value) {
  std::cout << variable_name << " = " << detail::Format(value) << '\n';
}

}  // namespace Debug
