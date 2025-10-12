#pragma once
#include "buffer.hpp"
#include "shader.hpp"
#include "camera.hpp"
#include "common.hpp"
#include "font.hpp"

#include <glm/glm.hpp>

#include <memory>

namespace sudoku {

struct line
{
    glm::vec2 begin;
    glm::vec2 end;
    glm::vec4 begin_colour;
    glm::vec4 end_colour;
    float     thickness;

    static void set_buffer_attributes(u32 vbo);
};

struct circle
{
    glm::vec2 centre;
    float     inner_radius;
    float     outer_radius;
    glm::vec4 begin_colour;
    glm::vec4 end_colour;
    float     angle;

    static void set_buffer_attributes(u32 vbo);
};

struct quad
{
    glm::ivec2 top_left;
    int        width;
    int        height;
    float      angle;
    glm::vec4  colour;
    int        use_texture;
    glm::ivec2 uv_pos;
    glm::ivec2 uv_size;

    static void set_buffer_attributes(std::uint32_t vbo);
};

class renderer
{
    u32 d_vao;
    u32 d_vbo;
    u32 d_ebo;

    std::vector<line>   d_lines;
    std::vector<circle> d_circles;
    std::vector<quad>   d_quads;

    shader d_line_shader;
    shader d_circle_shader;
    shader d_quad_shader;

    vertex_buffer d_instances;

    font_atlas d_atlas;

public:
    renderer();
    ~renderer();

    // Renders all queued up geometry
    void draw(i32 screen_width, i32 screen_height);

    auto font() const -> const font_atlas& { return d_atlas; }

    // Queues up geometry to render
    void push_rect(glm::vec2 top_left, float width, float height, glm::vec4 colour);
    void push_quad(glm::vec2 centre, float width, float height, float angle, glm::vec4 colour);
    void push_line(glm::vec2 begin, glm::vec2 end, glm::vec4 begin_colour, glm::vec4 end_colour, float thickness);
    void push_line(glm::vec2 begin, glm::vec2 end, glm::vec4 colour, float thickness);
    void push_circle(glm::vec2 centre, glm::vec4 colour, float radius);
    void push_annulus(glm::vec2 centre, glm::vec4 colour, float inner_radius, float outer_radius);
    void push_text(std::string_view message, glm::ivec2 pos, i32 size, glm::vec4 colour);
    void push_text_box(std::string_view message, glm::ivec2 pos, i32 width, i32 height, i32 scale, glm::vec4 colour);
};

}