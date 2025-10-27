#pragma once
#include "utility.hpp"

#include <cstdint>
#include <span>

#include <glm/glm.hpp>

namespace snooker {

class texture_dyn
{
    u32 d_texture = 0;
    i32 d_width   = 0;
    i32 d_height  = 0;

    texture_dyn(const texture_dyn&) = delete;
    texture_dyn& operator=(const texture_dyn&) = delete;

public:
    texture_dyn(i32 width, i32 height);
    ~texture_dyn();

    auto set_data(std::span<const glm::vec4> data) -> void;
    auto set_subdata(std::span<const glm::vec4> data, glm::ivec2 top_left, i32 width, i32 height) -> void;
    auto bind() const -> void;

    auto resize(i32 width, i32 height) -> void;

    auto width() const -> i32 { return d_width; }
    auto height() const -> i32 { return d_height; }
};

class texture_png
{
    u32 d_texture = 0;
    i32 d_width   = 0;
    i32 d_height  = 0;

    texture_png(const texture_png&) = delete;
    texture_png& operator=(const texture_png&) = delete;

public:
    texture_png(const char* filename);
    ~texture_png();

    auto bind()    const -> void;
    auto width()   const -> i32 { return d_width; }
    auto height()  const -> i32 { return d_height; }
};

}