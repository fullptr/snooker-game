#pragma once
#include <variant>
#include <vector>
#include <glm/glm.hpp>

namespace snooker {

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
    shape     geometry;
    float     mass; // non-positive mass == static

    auto inv_mass() const -> float {
        if (mass <= 0) return 0;
        return 1.0f / mass;
    }
};

void step_simulation(std::vector<collider>& colliders, float dt);

}