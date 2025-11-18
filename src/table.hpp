#pragma once
#include "utility.hpp"
#include "simulation.hpp"

#include <glm/glm.hpp>
#include <variant>

namespace snooker {

constexpr auto clear_colour = from_hex(0x222f3e);
constexpr auto ball_radius = 2.54f; // english pool bool cm == 1 inch
constexpr auto ball_mass = 140.0f; // grams
constexpr auto board_colour = from_hex(0x3db81e);
constexpr auto break_speed = 983.49f; // cm

struct ball
{
    std::size_t id;
    glm::vec4   colour;
    bool        is_pocketed = false;
};

// dimensions are an english pool table in cm (6ft x 3ft)
struct table
{
    f32 length;
    f32 width;

    simulation sim;

    // Colliders
    ball                     cue_ball;
    std::vector<ball>        object_balls;
    std::vector<std::size_t> border_boxes;
    std::vector<std::size_t> pockets;

    auto dimensions() -> glm::vec2 { return {length, width}; }

    void set_cue_ball(glm::vec2 position)
    {
        auto id = sim.add_dynamic_circle(position, ball_radius, ball_mass);
        cue_ball = ball{ .id=id, .colour={1, 1, 1, 1}};
    }

    void add_ball(glm::vec2 position, glm::vec4 colour)
    {
        auto id = sim.add_dynamic_circle(position, ball_radius, ball_mass);
        object_balls.emplace_back(id, colour);
    }

    void add_pocket(glm::vec2 position, float radius)
    {
        auto id = sim.add_attractor_circle(position, radius);
        pockets.push_back(id);
    }
};

}