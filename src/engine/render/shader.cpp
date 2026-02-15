#include <glad/glad.h>

#include <engine/render/shader.hpp>
#include <filesystem>
#include <fstream>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

namespace engine::render {
namespace fs = std::filesystem;

Shader::Shader(const char* vertex_path, const char* fragment_path) {
  fs::path exe_dir = fs::absolute(fs::current_path());
  fs::path v_path = exe_dir / vertex_path;
  fs::path f_path = exe_dir / fragment_path;

#ifdef PROJECT_SOURCE_DIR
  // fallback to source folder if not found in build
  if (!fs::exists(vPath)) vPath = fs::path(PROJECT_SOURCE_DIR) / "resources/shaders" / vertexPath;
  if (!fs::exists(fPath)) fPath = fs::path(PROJECT_SOURCE_DIR) / "resources/shaders" / fragmentPath;
#endif

  // 1. retrieve the vertex/fragment source code from filePath
  std::string vertex_code;
  std::string fragment_code;
  std::ifstream v_shader_file;
  std::ifstream f_shader_file;
  // ensure ifstream objects can throw exceptions:
  v_shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  f_shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  try {
    // open files
    v_shader_file.open(v_path);
    f_shader_file.open(f_path);
    std::stringstream v_shader_stream, f_shader_stream;
    // read file's buffer contents into streams
    v_shader_stream << v_shader_file.rdbuf();
    f_shader_stream << f_shader_file.rdbuf();
    // close file handlers
    v_shader_file.close();
    f_shader_file.close();
    // convert stream into string
    vertex_code = v_shader_stream.str();
    fragment_code = f_shader_stream.str();
  } catch (std::ifstream::failure& e) {
    std::cout << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << e.what() << std::endl;
  }
  const char* v_shader_code = vertex_code.c_str();
  const char* f_shader_code = fragment_code.c_str();
  // 2. compile shaders
  unsigned int vertex, fragment;
  // vertex shader
  vertex = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vertex, 1, &v_shader_code, nullptr);
  glCompileShader(vertex);
  CheckCompileErrors(vertex, "VERTEX");
  // fragment Shader
  fragment = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fragment, 1, &f_shader_code, nullptr);
  glCompileShader(fragment);
  CheckCompileErrors(fragment, "FRAGMENT");
  // shader Program
  id = glCreateProgram();
  glAttachShader(id, vertex);
  glAttachShader(id, fragment);
  glLinkProgram(id);
  CheckCompileErrors(id, "PROGRAM");
  // delete the shaders as they're linked into our program now and no longer
  // necessary
  glDeleteShader(vertex);
  glDeleteShader(fragment);
};

void Shader::Use() { glUseProgram(id); };

void Shader::SetBool(const std::string& name, bool value) const {
  glUniform1i(glGetUniformLocation(id, name.c_str()), static_cast<int>(value));
};

void Shader::SetInt(const std::string& name, int value) const {
  glUniform1i(glGetUniformLocation(id, name.c_str()), value);
};

void Shader::SetFloat(const std::string& name, float value) const {
  glUniform1f(glGetUniformLocation(id, name.c_str()), value);
};

void Shader::SetVec3(const std::string& name, float x, float y, float z) const {
  glUniform3f(glGetUniformLocation(id, name.c_str()), x, y, z);
};

void Shader::SetVec3(const std::string& name, const glm::vec3& value) const {
  glUniform3f(glGetUniformLocation(id, name.c_str()), value.x, value.y, value.z);
};

void Shader::SetVec4(const std::string& name, float x, float y, float z, float w) const {
  glUniform4f(glGetUniformLocation(id, name.c_str()), x, y, z, w);
};

void Shader::SetMat4(const std::string& name, const glm::mat4& value) const {
  glUniformMatrix4fv(glGetUniformLocation(id, name.c_str()), 1, GL_FALSE, glm::value_ptr(value));
};

void Shader::CheckCompileErrors(unsigned int shader, std::string type) {
  int success;
  char info_log[1024];
  if (type != "PROGRAM") {
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      glGetShaderInfoLog(shader, 1024, nullptr, info_log);
      std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
                << info_log << "\n -- --------------------------------------------------- -- " << std::endl;
    }
  } else {
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success) {
      glGetProgramInfoLog(shader, 1024, nullptr, info_log);
      std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
                << info_log << "\n -- --------------------------------------------------- -- " << std::endl;
    }
  }
};
}  // namespace engine::render
