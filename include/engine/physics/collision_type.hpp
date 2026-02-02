#pragma once

#include <ostream>

enum class CollisionType {
  CORNER_CORNER,
  CORNER_EDGE,
  CORNER_FACE,
  EDGE_EDGE,
  EDGE_FACE,
  FACE_FACE
};

inline const char* ToString(CollisionType t) {
  switch (t) {
    case CollisionType::CORNER_CORNER:
      return "CORNER_CORNER";
    case CollisionType::CORNER_EDGE:
      return "CORNER_EDGE";
    case CollisionType::CORNER_FACE:
      return "CORNER_FACE";
    case CollisionType::EDGE_EDGE:
      return "EDGE_EDGE";
    case CollisionType::EDGE_FACE:
      return "EDGE_FACE";
    case CollisionType::FACE_FACE:
      return "FACE_FACE";
  }
  return "UNKNOWN";
}

inline std::ostream& operator<<(std::ostream& os, CollisionType t) {
  return os << ToString(t);
}
