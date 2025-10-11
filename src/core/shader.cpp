#include "shader.hpp"
#include "utility.hpp"

#include <glad/glad.h>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <filesystem>
#include <fstream>
#include <print>

namespace sudoku {
namespace {

auto compile_shader(std::uint32_t type, const char* source) -> std::uint32_t
{
    std::uint32_t id = glCreateShader(type);
    glShaderSource(id, 1, &source, nullptr);
    glCompileShader(id);

    int result = 0;
    glGetShaderiv(id, GL_COMPILE_STATUS, &result);
    if (!result) {
        char infoLog[512];
        glGetShaderInfoLog(id, 512, NULL, infoLog);
        std::print("[{} ERROR] {}\n", (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT", infoLog);
        glDeleteShader(id);
        std::terminate();
    }

    return id;
}

}

shader::shader(const char* vertex_shader_source, const char* fragment_shader_source)
    : d_program(glCreateProgram())
    , d_vertex_shader(compile_shader(GL_VERTEX_SHADER, vertex_shader_source))
    , d_fragment_shader(compile_shader(GL_FRAGMENT_SHADER, fragment_shader_source))
{
    glAttachShader(d_program, d_vertex_shader);
    glAttachShader(d_program, d_fragment_shader);
    glLinkProgram(d_program);
    glValidateProgram(d_program);
}

shader::~shader()
{
    glDetachShader(d_program, d_vertex_shader);
    glDetachShader(d_program, d_fragment_shader);
    glDeleteShader(d_vertex_shader);
    glDeleteShader(d_fragment_shader);
    glDeleteProgram(d_program);
}

auto shader::get_location(const char* name) const -> u32
{
    return glGetUniformLocation(d_program, name);
}

auto shader::bind() const -> void
{
    glUseProgram(d_program);
}

auto shader::unbind() const -> void
{
    glUseProgram(0);
}

auto shader::load_mat4(const char* name, const glm::mat4& matrix) const -> void
{
    glProgramUniformMatrix4fv(d_program, get_location(name), 1, GL_FALSE, glm::value_ptr(matrix));
}

auto shader::load_vec2(const char* name, const glm::vec2& vector) const -> void
{
    glProgramUniform2f(d_program, get_location(name), vector.x, vector.y);
}

auto shader::load_vec3(const char* name, const glm::vec3& vector) const -> void
{
    glProgramUniform3f(d_program, get_location(name), vector.x, vector.y, vector.z);
}

auto shader::load_vec4(const char* name, const glm::vec4& vector) const -> void
{
    glProgramUniform4f(d_program, get_location(name), vector.x, vector.y, vector.z, vector.w);
}

auto shader::load_sampler(const char* name, int value) const -> void
{
    glProgramUniform1i(d_program, get_location(name), value);
}

auto shader::load_int(const char* name, int value) const -> void
{
    glProgramUniform1i(d_program, get_location(name), value);
}

auto shader::load_float(const char* name, float value) const -> void
{
    glProgramUniform1f(d_program, get_location(name), value);
}

}