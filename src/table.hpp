#pragma once
#include "utility.hpp"

#include <variant>
#include <glm/glm.hpp>

namespace snooker {

constexpr auto clear_colour = from_hex(0x222f3e);
constexpr auto ball_radius = 2.54f; // english pool bool cm == 1 inch
constexpr auto ball_mass = 140.0f; // grams
constexpr auto board_colour = from_hex(0x3db81e);
constexpr auto break_speed = 983.49f; // cm

struct circle_shape
{
    float radius;
};

struct box_shape
{
    float width;
    float height;
};

using shape = std::variant<circle_shape, box_shape>;

struct collider
{
    glm::vec2 pos;
    glm::vec2 vel;
    glm::vec4 colour;
    shape     geometry;
    float     mass; // negative mass == static
};

// dimensions are an english pool table in cm (6ft x 3ft)
struct table
{
    f32 length;
    f32 width;

    auto dimensions() -> glm::vec2 { return {length, width}; }
};

struct ball
{
    glm::vec2 pos;
    glm::vec2 vel;
    glm::vec4 colour;

    float mass = ball_mass;
    float radius = ball_radius;

    auto inv_mass() const -> float {
        assert_that(mass != 0, "mass cannot be zero\n");
        return 1.0f / mass;
    }
};

}