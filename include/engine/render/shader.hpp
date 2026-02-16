#pragma once
#include <glm/glm.hpp>
#include <string>

namespace engine::render {

class Shader {
 public:
  unsigned int id;
  // constructor generates the shader on the fly
  // ------------------------------------------------------------------------
  Shader(const char* vertex_path, const char* fragment_path);

  // activate the shader
  // ------------------------------------------------------------------------
  void Use();

  // utility uniform functions
  // ------------------------------------------------------------------------
  void SetBool(const std::string& name, bool value) const;

  // ------------------------------------------------------------------------
  void SetInt(const std::string& name, int value) const;

  // ------------------------------------------------------------------------
  void SetFloat(const std::string& name, float value) const;

  void SetVec3(const std::string& name, float x, float y, float z) const;
  void SetVec3(const std::string& name, const glm::vec3& value) const;

  void SetVec4(const std::string& name, float x, float y, float z, float w) const;

  void SetMat4(const std::string& name, const glm::mat4& value) const;

 private:
  // utility function for checking shader compilation/linking errors.
  // ------------------------------------------------------------------------
  void CheckCompileErrors(unsigned int shader, std::string type);
};
}  // namespace engine::render
