#pragma once
#include <variant>
#include <vector>
#include <ranges>
#include <cassert>
#include <unordered_map>
#include <glm/glm.hpp>

#include "utility.hpp"

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
};

template <typename T>
class id_vector
{
    std::vector<T>           d_data;
    std::vector<std::size_t> d_data_id;
    std::unordered_map<std::size_t, std::size_t> d_id_to_index;
    std::size_t d_next = 0;

public:
    auto is_valid(std::size_t id) -> bool
    {
        return d_id_to_index.contains(id);
    }
    auto add_collider(const T& c) -> std::size_t
    {
        const auto id = d_next++;
        const auto index = d_data.size();
        d_id_to_index[id] = index;
        d_data.emplace_back(c);
        d_data_id.emplace_back(id);
        return id;
    }
    auto remove_collider(std::size_t id) -> void
    {
        assert_that(is_valid(id));
        const auto index = d_id_to_index[id];

        // swap the id to delete to the end
        std::swap(d_data[index], d_data.back());
        std::swap(d_data_id[index], d_data_id.back());

        // update the map for the element swapped in
        d_id_to_index[d_data_id[index]] = index;

        // remove the element we want to remove
        d_data.pop_back();
        d_data_id.pop_back();
    }
    auto data() -> std::vector<T>&
    {
        return d_data;
    }
    auto get(std::size_t id) -> T&
    {
        assert_that(is_valid(id));
        return d_data[d_id_to_index[id]];
    }
};

void step_simulation(std::vector<collider>& colliders, float dt);

class simulation
{
    id_vector<collider> d_colliders;

public:
    auto add_circle(glm::vec2 pos, float radius, float mass) -> std::size_t
    {
        const auto col = collider{ .pos=pos, .vel=glm::vec2{0, 0}, .geometry=circle_shape{radius}, .mass=mass };
        const auto id = d_colliders.add_collider(col);
        return id;
    }

    // Currently only allows for static boxes
    auto add_box(glm::vec2 centre, float width, float height) -> std::size_t
    {
        const auto col = collider{ .pos=centre, .vel=glm::vec2{0, 0}, .geometry=box_shape{.width=width, .height=height}, .mass=-1};
        const auto id = d_colliders.add_collider(col);
        return id;
    }

    auto get(std::size_t id) -> collider&
    {
        assert_that(d_colliders.is_valid(id));
        return d_colliders.get(id);
    }

    auto step(float dt) -> void
    {
        step_simulation(d_colliders.data(), dt);
    }

    // TODO: Remove
    auto get_all() -> std::vector<collider>&
    {
        return d_colliders.data();
    }
};

}