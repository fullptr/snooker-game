#pragma once
#include <glm/glm.hpp>
#include <optional>

namespace snooker {

struct ray
{
    glm::vec2 start;
    glm::vec2 dir;
};

struct circle
{
    glm::vec2 centre;
    float     radius;
};

struct line
{
    glm::vec2 start;
    glm::vec2 end;
};

struct capsule
{
    glm::vec2 start;
    glm::vec2 end;
    float radius;
};

struct box
{
    glm::vec2 centre;
    float     width;
    float     height;
};

// A mix between a box and a capsule. Think of it at a box with a layer
// of padding round the edge.
struct padded_box
{
    glm::vec2 centre;
    float     width;
    float     height;
    float     radius;
};

auto ray_to_circle(ray r, circle c) -> std::optional<float>;
auto ray_to_line(ray r, line l) -> std::optional<float>;
auto ray_to_capsule(ray r, capsule c) -> std::optional<float>;
auto ray_to_box(ray r, box b) -> std::optional<float>;
auto ray_to_padded_box(ray r, padded_box b) -> std::optional<float>;

};