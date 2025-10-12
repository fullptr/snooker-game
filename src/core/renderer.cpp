#include "renderer.hpp"
#include "font.hpp"

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstddef>

namespace snooker {
namespace {

constexpr auto line_vertex = R"SHADER(
#version 410 core
layout (location = 0) in vec2 position;

layout (location = 1) in vec2  line_begin;
layout (location = 2) in vec2  line_end;
layout (location = 3) in vec4  line_begin_colour;
layout (location = 4) in vec4  line_end_colour;
layout (location = 5) in float line_thickness;

out vec2  o_line_begin;
out vec2  o_line_end;
out vec4  o_line_begin_colour;
out vec4  o_line_end_colour;
out float o_line_thickness;

void main()
{
    gl_Position = vec4(position, 0.0, 1.0);

    o_line_begin = line_begin;
    o_line_end = line_end;
    o_line_begin_colour = line_begin_colour;
    o_line_end_colour = line_end_colour;
    o_line_thickness = line_thickness;
} 
)SHADER";

constexpr auto line_fragment = R"SHADER(
#version 410 core
layout (location = 0) out vec4 out_colour;

in vec2  o_line_begin;
in vec2  o_line_end;
in vec4  o_line_begin_colour;
in vec4  o_line_end_colour;
in float o_line_thickness;

uniform int u_camera_height;

float cross2d(vec2 a, vec2 b)
{
    return a.x * b.y - b.x * a.y;
}

void main()
{   
    vec2 pixel = vec2(gl_FragCoord.x, u_camera_height - gl_FragCoord.y);

    vec2 line_begin = o_line_begin;
    vec2 line_end = o_line_end;

    if (line_begin == line_end && distance(pixel, line_begin) < o_line_thickness) {
        out_colour = o_line_begin_colour;
        return;
    }

    vec2 A = line_end - line_begin;
    vec2 B = pixel - line_begin;
    float lengthA = length(A);

    float distance_from_line = abs(cross2d(A, B)) / lengthA;

    float ratio_along = dot(A, B) / (lengthA * lengthA);

    if (distance_from_line <= o_line_thickness) {
        if (ratio_along > 0 && ratio_along < 1) {
            out_colour = mix(o_line_begin_colour, o_line_end_colour, ratio_along);
        } else if (ratio_along <= 0 && distance(line_begin, pixel) < o_line_thickness) {
            out_colour = o_line_begin_colour;
        } else if (ratio_along >= 1 && distance(line_end, pixel) < o_line_thickness) {
            out_colour = o_line_end_colour;
        } else {
            out_colour = vec4(0.0);
        }
    }
}
)SHADER";

constexpr auto circle_vertex = R"SHADER(
#version 410 core
layout (location = 0) in vec2 position;

layout (location = 1) in vec2  circle_centre;
layout (location = 2) in float circle_inner_radius;
layout (location = 3) in float circle_outer_radius;
layout (location = 4) in vec4  circle_begin_colour;
layout (location = 5) in vec4  circle_end_colour;
layout (location = 6) in float circle_angle;

out vec2  o_circle_centre;
out float o_circle_inner_radius;
out float o_circle_outer_radius;
out vec4  o_circle_begin_colour;
out vec4  o_circle_end_colour;
out float o_circle_angle;

void main()
{
    gl_Position = vec4(position, 0.0, 1.0);
    
    o_circle_centre       = circle_centre;
    o_circle_inner_radius = circle_inner_radius;
    o_circle_outer_radius = circle_outer_radius;
    o_circle_begin_colour = circle_begin_colour;
    o_circle_end_colour   = circle_end_colour;
    o_circle_angle        = circle_angle;
} 
)SHADER";

constexpr auto circle_fragment = R"SHADER(
#version 410 core
layout (location = 0) out vec4 out_colour;

in vec2  o_circle_centre;
in float o_circle_inner_radius;
in float o_circle_outer_radius;
in vec4  o_circle_begin_colour;
in vec4  o_circle_end_colour;
in float o_circle_angle;

uniform int u_camera_height;

const float pi = 3.1415926535897932384626433832795;

// Rotates the given vec2 anti-clockwise by the given amount of radians.
vec2 rot(vec2 v, float rad)
{
    float s = sin(rad);
	float c = cos(rad);
	mat2 m = mat2(c, -s, s, c);
	return m * v;
}

// Returns 0 for vectors on positive x axis, 0.5, for negative x axis, increasing
// rotating clockwise.
float rotation_value(vec2 v)
{
    if (v.y >= 0) { // Top right
        float cos_theta = normalize(-v).x;
        float theta = acos(cos_theta) + pi;
        return theta / (2 * pi);
    }
    else {
        float cos_theta = normalize(v).x;
        float theta = acos(cos_theta);
        return theta / (2 * pi);
    }
}

void main()
{   
    vec2 frag_coord = vec2(gl_FragCoord.x, u_camera_height - gl_FragCoord.y);
    vec2 pixel = frag_coord;

    vec2 to_pixel = pixel - o_circle_centre;
    float from_centre = length(to_pixel);
    
    if (o_circle_inner_radius < from_centre && from_centre < o_circle_outer_radius) {
        float angle_offset = o_circle_angle / (2 * pi);
        out_colour = mix(
            o_circle_begin_colour,
            o_circle_end_colour,
            rotation_value(rot(to_pixel, -o_circle_angle))
        );
        return;
    }
}
)SHADER";

constexpr auto quad_vertex = R"SHADER(
    #version 410 core
    layout (location = 0) in vec2 p_position;
    
    layout (location = 1) in ivec2 quad_top_left;
    layout (location = 2) in int   quad_width;
    layout (location = 3) in int   quad_height;
    layout (location = 4) in float quad_angle;
    layout (location = 5) in vec4  quad_colour;
    layout (location = 6) in int   quad_use_texture;
    layout (location = 7) in ivec2 quad_uv_pos;
    layout (location = 8) in ivec2 quad_uv_size;
    
    uniform mat4      u_proj_matrix;
    uniform sampler2D u_texture;
    
    flat out int o_use_texture;
    out vec4     o_colour;
    out vec2     o_uv;
    
    mat2 rotate(float theta)
    {
        float c = cos(theta);
        float s = sin(theta);
        return mat2(c, s, -s, c);
    }
    
    void main()
    {
        vec2 dimensions = vec2(quad_width, quad_height) / 2;
        vec2 position = quad_top_left + dimensions;
    
        vec2 screen_position = rotate(quad_angle) * (p_position * dimensions) + position;
    
        gl_Position = u_proj_matrix * vec4(screen_position, 0, 1);
    
        o_use_texture = quad_use_texture;
        o_colour = quad_colour;

        o_uv = (p_position + vec2(1, 1)) / 2;
        o_uv = (o_uv * quad_uv_size + quad_uv_pos) / textureSize(u_texture, 0);
    }
)SHADER";
    
constexpr auto quad_fragment = R"SHADER(
    #version 410 core
    layout (location = 0) out vec4 out_colour;
    
    flat in int o_use_texture;
    in vec4     o_colour;
    in vec2     o_uv;

    uniform int       u_use_texture;
    uniform sampler2D u_texture;
    
    void main()
    {
        if (o_use_texture > 0) {
            float red = texture(u_texture, o_uv).r;
            out_colour = vec4(o_colour.xyz, red);
        } else {
            out_colour = o_colour;
        }
    }
)SHADER";

auto load_pixel_font_atlas() -> font_atlas
{
    font_atlas atlas;
    atlas.texture = std::make_unique<texture_png>("res\\pixel_font.png");
    atlas.missing_char = { .position{24, 52}, .size{5, 6}, .bearing{0, -6}, .advance=6 };
    atlas.height = 7;
    
    atlas.chars['A'] = { .position{0, 0}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['B'] = { .position{6, 0}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['C'] = { .position{12, 0}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['D'] = { .position{18, 0}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['E'] = { .position{24, 0}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['F'] = { .position{29, 0}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['G'] = { .position{34, 0}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['H'] = { .position{40, 0}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['I'] = { .position{46, 0}, .size={3, 7}, .bearing={0, -7}, .advance=4 };
    atlas.chars['J'] = { .position{50, 0}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['K'] = { .position{55, 0}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['L'] = { .position{60, 0}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['M'] = { .position{0, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['N'] = { .position{6, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['O'] = { .position{12, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['P'] = { .position{18, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['Q'] = { .position{24, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['R'] = { .position{30, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['S'] = { .position{36, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['T'] = { .position{42, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['U'] = { .position{48, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['V'] = { .position{54, 8}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['W'] = { .position{0, 16}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['X'] = { .position{6, 16}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['Y'] = { .position{12, 16}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['Z'] = { .position{18, 16}, .size={5, 7}, .bearing={0, -7}, .advance=6 };

    atlas.chars['a'] = { .position{24, 18}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['b'] = { .position{29, 16}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['c'] = { .position{34, 18}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['d'] = { .position{39, 16}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['e'] = { .position{44, 18}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['f'] = { .position{49, 16}, .size={3, 10}, .bearing={0, -7}, .advance=4 };
    atlas.chars['g'] = { .position{53, 18}, .size={4, 8}, .bearing={0, -5}, .advance=5 };
    atlas.chars['h'] = { .position{58, 16}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['i'] = { .position{63, 16}, .size={1, 7}, .bearing={0, -7}, .advance=2 };
    atlas.chars['j'] = { .position{1, 24}, .size={2, 10}, .bearing={0, -7}, .advance=3 };
    atlas.chars['k'] = { .position{4, 24}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['l'] = { .position{9, 24}, .size={1, 7}, .bearing={0, -7}, .advance=2 };
    atlas.chars['m'] = { .position{11, 26}, .size={5, 5}, .bearing={0, -5}, .advance=6 };
    atlas.chars['n'] = { .position{17, 26}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['o'] = { .position{22, 26}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['p'] = { .position{27, 26}, .size={4, 7}, .bearing={0, -5}, .advance=5 };
    atlas.chars['q'] = { .position{32, 26}, .size={5, 7}, .bearing={0, -5}, .advance=5 };
    atlas.chars['r'] = { .position{37, 26}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['s'] = { .position{42, 26}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['t'] = { .position{47, 24}, .size={2, 7}, .bearing={0, -7}, .advance=3 };
    atlas.chars['u'] = { .position{50, 26}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['v'] = { .position{55, 26}, .size={5, 5}, .bearing={0, -5}, .advance=6 };
    atlas.chars['w'] = { .position{0, 34}, .size={5, 5}, .bearing={0, -5}, .advance=6 };
    atlas.chars['x'] = { .position{6, 34}, .size={4, 5}, .bearing={0, -5}, .advance=5 };
    atlas.chars['y'] = { .position{11, 34}, .size={4, 8}, .bearing={0, -5}, .advance=5 };
    atlas.chars['z'] = { .position{16, 34}, .size={4, 5}, .bearing={0, -5}, .advance=5 };

    atlas.chars['1'] = { .position{0, 43}, .size={3, 7}, .bearing={0, -7}, .advance=4 };
    atlas.chars['2'] = { .position{4, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['3'] = { .position{10, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['4'] = { .position{16, 43}, .size={4, 7}, .bearing={0, -7}, .advance=5 };
    atlas.chars['5'] = { .position{21, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['6'] = { .position{27, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['7'] = { .position{33, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['8'] = { .position{39, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['9'] = { .position{45, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['0'] = { .position{51, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };

    atlas.chars['#'] = { .position{42, 51}, .size={5, 5}, .bearing={0, -6}, .advance=6 };
    atlas.chars['&'] = { .position{28, 35}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['~'] = { .position{6, 40}, .size={4, 2}, .bearing={0, -5}, .advance=5 };
    atlas.chars['!'] = { .position{60, 8}, .size={1, 7}, .bearing={0, -7}, .advance=2 };
    atlas.chars['?'] = { .position{57, 43}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['$'] = { .position{36, 51}, .size={5, 9}, .bearing={0, -8}, .advance=6 };
    atlas.chars[';'] = { .position{62, 10}, .size={2, 5}, .bearing={0, -5}, .advance=3 };
    atlas.chars[':'] = { .position{63, 10}, .size={1, 4}, .bearing={0, -5}, .advance=2 };
    atlas.chars[','] = { .position{61, 29}, .size={2, 2}, .bearing={0, -2}, .advance=3 };
    atlas.chars['.'] = { .position{62, 29}, .size={1, 1}, .bearing={0, -1}, .advance=2 };
    atlas.chars[' '] = { .position{0, 0}, .size={1, 1}, .bearing={0, 0}, .advance=3 };
    atlas.chars['('] = { .position{38, 32}, .size={2, 9}, .bearing={0, -8}, .advance=3 };
    atlas.chars[')'] = { .position{41, 32}, .size={2, 9}, .bearing={0, -8}, .advance=3 };
    atlas.chars['{'] = { .position{44, 32}, .size={3, 9}, .bearing={0, -8}, .advance=4 };
    atlas.chars['}'] = { .position{48, 32}, .size={3, 9}, .bearing={0, -8}, .advance=4 };
    atlas.chars['['] = { .position{30, 51}, .size={2, 9}, .bearing={0, -8}, .advance=3 };
    atlas.chars[']'] = { .position{33, 51}, .size={2, 9}, .bearing={0, -8}, .advance=3 };
    atlas.chars['^'] = { .position{52, 32}, .size={5, 3}, .bearing={0, -7}, .advance=6 };
    atlas.chars['<'] = { .position{58, 32}, .size={3, 5}, .bearing={0, -6}, .advance=4 };
    atlas.chars['>'] = { .position{61, 34}, .size={3, 5}, .bearing={0, -6}, .advance=4 };
    atlas.chars['_'] = { .position{1, 41}, .size={4, 1}, .bearing={0, 0}, .advance=5 };
    atlas.chars['-'] = { .position{1, 41}, .size={3, 1}, .bearing={0, -4}, .advance=4 };
    atlas.chars['+'] = { .position{0, 51}, .size={3, 3}, .bearing={0, -5}, .advance=4 };
    atlas.chars['='] = { .position{4, 51}, .size={3, 3}, .bearing={0, -5}, .advance=4 };
    atlas.chars['/'] = { .position{21, 32}, .size={3, 7}, .bearing={0, -7}, .advance=4 };
    atlas.chars['@'] = { .position{12, 51}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['%'] = { .position{18, 51}, .size={5, 7}, .bearing={0, -7}, .advance=6 };
    atlas.chars['\\'] = { .position{24, 32}, .size={3, 7}, .bearing={0, -7}, .advance=4 };
    atlas.chars['\"'] = { .position{8, 51}, .size={3, 2}, .bearing={0, -7}, .advance=4 };
    atlas.chars['\''] = { .position{8, 51}, .size={1, 2}, .bearing={0, -7}, .advance=2 };
    atlas.chars['|'] = { .position{48, 51}, .size={1, 9}, .bearing={0, -8}, .advance=2 };
    atlas.chars['`'] = { .position{50, 51}, .size={2, 2}, .bearing={0, -7}, .advance=3 };
    return atlas;
}

}

void line::set_buffer_attributes(std::uint32_t vbo)
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    for (int i = 1; i != 6; ++i) {
        glEnableVertexAttribArray(i);
        glVertexAttribDivisor(i, 1);
    }
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(line), (void*)offsetof(line, begin));
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(line), (void*)offsetof(line, end));
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(line), (void*)offsetof(line, begin_colour));
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(line), (void*)offsetof(line, end_colour));
    glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, sizeof(line), (void*)offsetof(line, thickness));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void circle::set_buffer_attributes(std::uint32_t vbo)
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    for (int i = 1; i != 7; ++i) {
        glEnableVertexAttribArray(i);
        glVertexAttribDivisor(i, 1);
    }
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(circle), (void*)offsetof(circle, centre));
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(circle), (void*)offsetof(circle, inner_radius));
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(circle), (void*)offsetof(circle, outer_radius));
    glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(circle), (void*)offsetof(circle, begin_colour));
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(circle), (void*)offsetof(circle, end_colour));
    glVertexAttribPointer(6, 1, GL_FLOAT, GL_FALSE, sizeof(circle), (void*)offsetof(circle, angle));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void quad::set_buffer_attributes(std::uint32_t vbo)
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    for (int i = 1; i != 9; ++i) {
        glEnableVertexAttribArray(i);
        glVertexAttribDivisor(i, 1);
    }
    glVertexAttribIPointer(1, 2, GL_INT, sizeof(quad), (void*)offsetof(quad, top_left));
    glVertexAttribIPointer(2, 1, GL_INT, sizeof(quad), (void*)offsetof(quad, width));
    glVertexAttribIPointer(3, 1, GL_INT, sizeof(quad), (void*)offsetof(quad, height));
    glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(quad), (void*)offsetof(quad, angle));
    glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(quad), (void*)offsetof(quad, colour));
    glVertexAttribIPointer(6, 1, GL_INT, sizeof(quad), (void*)offsetof(quad, use_texture));
    glVertexAttribIPointer(7, 2, GL_INT, sizeof(quad), (void*)offsetof(quad, uv_pos));
    glVertexAttribIPointer(8, 2, GL_INT, sizeof(quad), (void*)offsetof(quad, uv_size));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

renderer::renderer()
    : d_line_shader(line_vertex, line_fragment)
    , d_circle_shader(circle_vertex, circle_fragment)
    , d_quad_shader(quad_vertex, quad_fragment)
    , d_atlas{load_pixel_font_atlas()}
{
    const float vertices[] = {-1.0f, -1.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f};
    const std::uint32_t indices[] = {0, 1, 2, 0, 2, 3};

    glGenVertexArrays(1, &d_vao);
    glBindVertexArray(d_vao);

    glGenBuffers(1, &d_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, d_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glGenBuffers(1, &d_ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, d_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    d_quad_shader.bind();
    d_quad_shader.load_sampler("u_texture", 0);
}

renderer::~renderer()
{
    glDeleteBuffers(1, &d_ebo);
    glDeleteBuffers(1, &d_vbo);
    glDeleteVertexArrays(1, &d_vao);
}

void renderer::draw(i32 screen_width, i32 screen_height)
{
    // TODO: Merge with the rest
    {
        glBindVertexArray(d_vao);
        d_atlas.texture->bind();
        d_quad_shader.load_int("u_use_texture", 1);
    
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
        const auto dimensions = glm::vec2{screen_width, screen_height};
        const auto projection = glm::ortho(0.0f, dimensions.x, dimensions.y, 0.0f);
        
        d_quad_shader.bind();
        d_quad_shader.load_mat4("u_proj_matrix", projection);
        d_instances.bind<quad>(d_quads);
        glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr, (int)d_quads.size());
    
        glDisable(GL_BLEND);
    
        d_quads.clear();
    }

    glBindVertexArray(d_vao);

    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    d_line_shader.bind();
    d_line_shader.load_int("u_camera_height", screen_height);

    d_circle_shader.bind();
    d_circle_shader.load_int("u_camera_height", screen_height);

    const auto dimensions = glm::vec2{screen_width, screen_height};
    const auto projection = glm::ortho(0.0f, dimensions.x, dimensions.y, 0.0f);

    d_line_shader.bind();
    d_instances.bind<line>(d_lines);
    glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr, (int)d_lines.size());

    d_circle_shader.bind();
    d_instances.bind<circle>(d_circles);
    glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr, (int)d_circles.size());

    glDisable(GL_BLEND);
    d_lines.clear();
    d_circles.clear();
}

void renderer::push_rect(glm::vec2 top_left, float width, float height, glm::vec4 colour)
{
    d_quads.emplace_back(top_left, width, height, 0.0f, colour, false, glm::ivec2{0, 0}, glm::ivec2{0, 0});
}

void renderer::push_quad(glm::vec2 centre, float width, float height, float angle, glm::vec4 colour)
{
    d_quads.emplace_back(centre - glm::vec2{width/2, height/2}, width, height, angle, colour, false, glm::ivec2{0, 0}, glm::ivec2{0, 0});
}

void renderer::push_line(glm::vec2 begin, glm::vec2 end, glm::vec4 begin_colour, glm::vec4 end_colour, float thickness)
{
    d_lines.emplace_back(begin, end, begin_colour, end_colour, thickness);
}

void renderer::push_line(glm::vec2 begin, glm::vec2 end, glm::vec4 colour, float thickness)
{
    d_lines.emplace_back(begin, end, colour, colour, thickness);
}

void renderer::push_circle(glm::vec2 centre, glm::vec4 colour, float radius)
{
    d_circles.emplace_back(centre, 0.0f, radius, colour, colour, 0.0f);
}

void renderer::push_annulus(glm::vec2 centre, glm::vec4 colour, float inner_radius, float outer_radius)
{
    d_circles.emplace_back(centre, inner_radius, outer_radius, colour, colour, 0.0f);
}

void renderer::push_text(std::string_view message, glm::ivec2 pos, i32 size, glm::vec4 colour)
{
    for (char c : message) {
        const auto ch = d_atlas.get_character(c);

        d_quads.push_back(quad{
            pos + (size * ch.bearing),
            size * ch.size.x,
            size * ch.size.y,
            0.0f,
            colour,
            1,
            ch.position,
            ch.size
        });
        pos.x += size * ch.advance;
    }
}

void renderer::push_text_box(std::string_view message, glm::ivec2 pos, i32 width, i32 height, i32 scale, glm::vec4 colour)
{
    if (message.empty()) return;
    auto text_pos = pos;
    text_pos.x += (width - d_atlas.length_of(message) * scale) / 2;
    text_pos.y += (height + d_atlas.height * scale) / 2;
    push_text(message, text_pos, scale, colour);
}
    
}