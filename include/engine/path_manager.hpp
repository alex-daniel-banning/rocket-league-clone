#pragma once

#include <filesystem>

namespace engine {

class PathManager {
 public:
  static void SetRoot(const std::filesystem::path& root) {
    project_root = std::filesystem::absolute(root);
  }
  static std::filesystem::path GlobalAsset(const std::filesystem::path& p) {
    return project_root / "resources" / p;
  }

 private:
  // todo, put in config/properties file
  inline static std::filesystem::path project_root =
      "/home/alex/source/rocket-league-clone";
};

}  // namespace engine
