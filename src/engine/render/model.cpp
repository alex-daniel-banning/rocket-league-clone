#include <glad/glad.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <engine/render/model.hpp>
#include <stb_image.h>

#include "iostream"

namespace engine::render {

void Model::Draw(engine::render::Shader& shader) const {
  for (const Mesh& mesh : meshes_) {
    mesh.Draw(shader);
  }
}

void Model::LoadModel(const std::string& path) {
  Assimp::Importer import;
  const aiScene* scene = import.ReadFile(path, aiProcess_Triangulate);

  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    std::cout << "ERROR::ASSIMP::" << import.GetErrorString() << std::endl;
    return;
  }

  directory_ = path.substr(0, path.find_last_of('/'));

  // logging
  std::cout << "Loading model with directory -> " << directory_ << std::endl;

  ProcessNode(scene->mRootNode, scene);
}

void Model::ProcessNode(aiNode* node, const aiScene* scene) {
  // process all the node's meshes (if any)
  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
    meshes_.push_back(ProcessMesh(mesh, scene));
  }
  for (unsigned int i = 0; i < node->mNumChildren; i++) {
    ProcessNode(node->mChildren[i], scene);
  }
}

Mesh Model::ProcessMesh(aiMesh* mesh, const aiScene* scene) {
  std::vector<Vertex> vertices;
  std::vector<unsigned int> indices;
  std::vector<Texture> textures;

  for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
    Vertex vertex;
    // process vertex positions, normals, and texture coordinates
    glm::vec3 pos;
    pos.x = mesh->mVertices[i].x;
    pos.y = mesh->mVertices[i].y;
    pos.z = mesh->mVertices[i].z;
    vertex.position = pos;
    glm::vec3 norm;
    norm.x = mesh->mNormals[i].x;
    norm.y = mesh->mNormals[i].y;
    norm.z = mesh->mNormals[i].z;
    vertex.normal = norm;
    if (mesh->mTextureCoords[0]) {
      // logging
      // std::cout << "Processing mesh with textures.\n";

      glm::vec2 tex;
      tex.x = mesh->mTextureCoords[0][i].x;
      tex.y = mesh->mTextureCoords[0][i].y;
      vertex.tex_coords = tex;
    } else {
      // logging
      // std::cout << "Processing mesh with no textures.\n";

      vertex.tex_coords = glm::vec2(0.0f, 0.0f);
    }
    vertices.push_back(vertex);
  }
  // process indices
  for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
    aiFace face = mesh->mFaces[i];
    for (unsigned int j = 0; j < face.mNumIndices; j++) {
      indices.push_back(face.mIndices[j]);
    }
  }
  // process material

  aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
  if (mesh->mMaterialIndex >= 0) {
    // logging
    // std::cout << "Processing mesh with material.\n";

    std::vector<Texture> diffuse_maps = LoadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
    textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());
    std::vector<Texture> specular_maps = LoadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
    textures.insert(textures.end(), specular_maps.begin(), specular_maps.end());
  } else {
    // logging
    // std::cout << "Processing mesh with no material.\n";
  }

  if (textures.size() == 0) {
    aiColor4D diffuse;
    glm::vec3 color;
    if (material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse) == AI_SUCCESS) {
      color = glm::vec3(diffuse.r, diffuse.g, diffuse.b);
    } else {
      std::cout << "Unsuccessfully extracted color from material.";
    }
    return Mesh(vertices, indices, color);
  } else {
    return Mesh(vertices, indices, textures);
  }
}

std::vector<Texture> Model::LoadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string type_name) {
  std::vector<Texture> textures;
  for (unsigned int i = 0; i < mat->GetTextureCount(type); i++) {
    aiString str;
    mat->GetTexture(type, i, &str);
    bool skip = false;
    for (Texture& texture : loaded_textures_) {
      if (std::strcmp(texture.path.data(), str.C_Str()) == 0) {
        textures.push_back(texture);
        skip = true;
        break;
      }
    }
    if (!skip) {
      Texture texture;
      texture.id = TextureFromFile(str.C_Str(), directory_);
      texture.type = type_name;
      texture.path = str.C_Str();
      textures.push_back(texture);
      loaded_textures_.push_back(texture);

      // logging
      std::cout << "Processing texture with path -> " << texture.path << std::endl;
    }
  }
  return textures;
}

unsigned int TextureFromFile(const char* path, const std::string& directory, bool gamma) {
  std::string filename = std::string(path);
  filename = directory + '/' + filename;

  unsigned int texture_id;
  glGenTextures(1, &texture_id);

  int width, height, nr_components;
  unsigned char* data = stbi_load(filename.c_str(), &width, &height, &nr_components, 0);
  if (data) {
    // logging
    std::cout << "Texture successfully loaded from filename -> " << filename << std::endl;

    GLenum format;
    if (nr_components == 1) {
      format = GL_RED;
    } else if (nr_components == 3) {
      format = GL_RGB;
    } else if (nr_components == 4) {
      format = GL_RGBA;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);
  } else {
    std::cout << "Texture failed to load at path: " << filename << std::endl;
    stbi_image_free(data);
  }
  return texture_id;
}
}  // namespace engine::render
