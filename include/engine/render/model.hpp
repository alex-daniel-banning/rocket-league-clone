#pragma once

#include <assimp/material.h>
#include <assimp/scene.h>
#include <engine/render/mesh.hpp>
#include <engine/render/shader.hpp>
#include <vector>

namespace engine::render {

unsigned int TextureFromFile(const char* path, const std::string& directory,
                             bool gamma = false);

class Model {
 public:
  explicit Model(const char* path) { LoadModel(path); }
  void Draw(engine::render::Shader& shader);

 private:
  // model data
  std::vector<Mesh> meshes_;
  std::string directory_;
  std::vector<Texture> loaded_textures_;

  void LoadModel(std::string path);
  void ProcessNode(aiNode* node, const aiScene* scene);
  Mesh ProcessMesh(aiMesh* mesh, const aiScene* scene);
  std::vector<Texture> LoadMaterialTextures(aiMaterial* mat, aiTextureType type,
                                            std::string type_name);
};
}  // namespace engine::render
