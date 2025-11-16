#pragma once
#include <variant>
#include <vector>
#include <ranges>
#include <cassert>
#include <unordered_map>
#include <glm/glm.hpp>

#include "utility.hpp"
#include "id_vector.hpp"

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

    // if true, this pulls colliders towards it
    bool attractor = false;
};

class simulation
{
    id_vector<collider> d_colliders;

public:
    auto add_circle(glm::vec2 pos, float radius, float mass, bool attractor = false) -> std::size_t
    {
        const auto col = collider{ .pos=pos, .vel=glm::vec2{0, 0}, .geometry=circle_shape{radius}, .mass=mass, .attractor=attractor };
        const auto id = d_colliders.insert(col);
        return id;
    }

    // Currently only allows for static boxes
    auto add_box(glm::vec2 centre, float width, float height) -> std::size_t
    {
        const auto col = collider{ .pos=centre, .vel=glm::vec2{0, 0}, .geometry=box_shape{.width=width, .height=height}, .mass=-1};
        const auto id = d_colliders.insert(col);
        return id;
    }

    auto get(std::size_t id) -> collider&
    {
        assert_that(d_colliders.is_valid(id), std::format("invalid id {}\n", id));
        return d_colliders.get(id);
    }

    auto get(std::size_t id) const -> const collider&
    {
        assert_that(d_colliders.is_valid(id), std::format("invalid id {}\n", id));
        return d_colliders.get(id);
    }

    auto step(float dt) -> void;
    
    auto is_valid(std::size_t id) const -> bool
    {
        return d_colliders.is_valid(id);
    }

    auto remove(std::size_t id) -> void
    {
        assert_that(d_colliders.is_valid(id), std::format("invalid id {}\n", id));
        d_colliders.erase(id);
    }
};

}