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

struct line_shape
{
    glm::vec2 start; // start and end are both offsets from the position
    glm::vec2 end;
};

using shape_type = std::variant<circle_shape, box_shape, line_shape>;

struct static_body
{
};

struct attractor_body
{
};

struct dynamic_body
{
    float     mass;
    glm::vec2 vel;
};

using body_type = std::variant<static_body, attractor_body, dynamic_body>;

struct collider
{
    glm::vec2  pos;
    body_type  body;
    shape_type shape;
};

class simulation
{
    id_vector<collider> d_colliders;

public:
    static constexpr auto time_step = 1.0f / 60.0f;
    static constexpr auto num_substeps = 20;

    auto add_dynamic_circle(glm::vec2 pos, float radius, float mass) -> std::size_t
    {
        const auto col = collider{ .pos=pos, .body=dynamic_body{ .mass=mass, .vel={0.0f, 0.0f} }, .shape=circle_shape{radius} };
        const auto id = d_colliders.insert(col);
        return id;
    }

    auto add_attractor_circle(glm::vec2 pos, float radius) -> std::size_t
    {
        const auto col = collider{ .pos=pos, .body=attractor_body{}, .shape=circle_shape{radius} };
        const auto id = d_colliders.insert(col);
        return id;
    }

    auto add_box(glm::vec2 centre, float width, float height) -> std::size_t
    {
        const auto col = collider{ .pos=centre, .body=static_body{}, .shape=box_shape{.width=width, .height=height} };
        const auto id = d_colliders.insert(col);
        return id;
    }

    auto add_static_line(glm::vec2 start, glm::vec2 end) -> std::size_t
    {
        // TODO: The position should probably be the centre of the line? Maybe?
        const auto col = collider{ .pos=glm::vec2{0, 0}, .body=static_body{}, .shape=line_shape{ .start=start, .end=end} };
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

    auto step() -> void;
    
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