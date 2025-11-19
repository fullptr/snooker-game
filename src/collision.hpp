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

    auto rel() const -> glm::vec2 { return end - start; }
    auto length() const -> float { return glm::length(end - start); }
    auto dir() const -> glm::vec2 {
        if (length() == 0) return glm::vec2{1, 0}; // default direction for zero vector
        return glm::normalize(end - start);
    }
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

auto ray_cast(ray r, circle c) -> std::optional<float>;
auto ray_cast(ray r, line l) -> std::optional<float>;
auto ray_cast(ray r, capsule c) -> std::optional<float>;
auto ray_cast(ray r, box b) -> std::optional<float>;
auto ray_cast(ray r, padded_box b) -> std::optional<float>;

auto inflate(circle c, float radius) -> circle;
auto inflate(line l, float radius) -> capsule;
auto inflate(capsule c, float radius) -> capsule;
auto inflate(box b, float radius) -> padded_box;
auto inflate(padded_box b, float radius) -> padded_box;

};