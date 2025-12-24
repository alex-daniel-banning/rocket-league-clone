#pragma once

#include <filesystem>

namespace engine
{

class PathManager
{
  public:
    static void setRoot(const std::filesystem::path &root)
    {
        projectRoot = std::filesystem::absolute(root);
    }
    static std::filesystem::path globalAsset(const std::filesystem::path &p)
    {
        return projectRoot / "resources" / p;
    }

  private:
    // todo, put in config/properties file
    inline static std::filesystem::path projectRoot = "/home/alex/source/rocket-league-clone";
};

} // namespace engine
