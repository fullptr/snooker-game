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

    std::vector<ball> balls;
    std::vector<collider> colliders; // for the physics sim

    auto dimensions() -> glm::vec2 { return {length, width}; }
    auto add_ball(glm::vec2 position, glm::vec4 colour)
    {
        const auto col = collider{ .pos=position, .vel=glm::vec2{0, 0}, .geometry=circle_shape{ball_radius}, .mass=ball_mass };
        colliders.push_back(col);
        const auto b = ball{ .collider=(colliders.size()-1), .colour=colour };
        balls.push_back(b);
    }
};

}