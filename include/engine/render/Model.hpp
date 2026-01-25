#pragma once

#include <assimp/material.h>
#include <assimp/scene.h>
#include <engine/render/Mesh.hpp>
#include <engine/render/Shader.hpp>
#include <vector>

namespace engine::render {

unsigned int TextureFromFile(const char *path, const std::string &directory,
                             bool gamma = false);

class Model {
 public:
  Model(const char *path) { loadModel(path); }
  void Draw(engine::render::Shader &shader);

 private:
  // model data
  std::vector<Mesh> meshes;
  std::string directory;
  std::vector<Texture> textures_loaded;

  void loadModel(std::string path);
  void processNode(aiNode *node, const aiScene *scene);
  Mesh processMesh(aiMesh *mesh, const aiScene *scene);
  std::vector<Texture> loadMaterialTextures(aiMaterial *mat, aiTextureType type,
                                            std::string typeName);
};
}  // namespace engine::render
