#pragma once
#include "common.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <filesystem>

namespace sudoku {

class shader
{
    u32 d_program;
    u32 d_vertex_shader;
    u32 d_fragment_shader;

    u32 get_location(const char* name) const;

    shader(const shader&) = delete;
    shader& operator=(const shader&) = delete;

public:
    shader(const char* vertex_shader_source, const char* fragment_shader_source);
    ~shader();

    auto bind() const -> void;
    auto unbind() const -> void;

    auto load_mat4(const char* name, const glm::mat4& matrix) const -> void;
    auto load_vec2(const char* name, const glm::vec2& vector) const -> void;
    auto load_vec3(const char* name, const glm::vec3& vector) const -> void;
    auto load_vec4(const char* name, const glm::vec4& vector) const -> void;
    auto load_int(const char* name, int value) const -> void;
    auto load_float(const char* name, float value) const -> void;
    auto load_sampler(const char* name, int value) const -> void;
};

}