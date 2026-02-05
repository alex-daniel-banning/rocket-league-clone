#pragma once

#include <ostream>

enum class CollisionType { CORNER_FACE, EDGE_EDGE, FACE_FACE };

inline const char* ToString(CollisionType t) {
  switch (t) {
    return "CORNER_EDGE";
    case CollisionType::CORNER_FACE:
      return "CORNER_FACE";
    case CollisionType::EDGE_EDGE:
      return "EDGE_EDGE";
    case CollisionType::FACE_FACE:
      return "FACE_FACE";
  }
  return "UNKNOWN";
}

inline std::ostream& operator<<(std::ostream& os, CollisionType t) {
  return os << ToString(t);
}
