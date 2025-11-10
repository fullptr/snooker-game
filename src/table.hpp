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
    std::size_t collider;
    glm::vec4   colour;
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

    auto dimensions() -> glm::vec2 { return {length, width}; }

    void set_cue_ball(glm::vec2 position)
    {
        const auto id = sim.add_circle(position, ball_radius, ball_mass);
        cue_ball = ball{ .collider=id, .colour={1, 1, 1, 1}};
    }

    void add_ball(glm::vec2 position, glm::vec4 colour)
    {
        const auto id = sim.add_circle(position, ball_radius, ball_mass);
        const auto b = ball{ .collider=id, .colour=colour };
        object_balls.push_back(b);
    }
};

}