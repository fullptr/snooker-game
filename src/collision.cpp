#include "collision.hpp"

namespace snooker {
namespace {

auto cross_2d(glm::vec2 a, glm::vec2 b) -> float {
    return a.x * b.y - a.y * b.x;
}

}

auto ray_cast(ray r, circle c) -> std::optional<float>
{
    const auto m = r.start - c.centre;
    const auto B = glm::dot(m, r.dir);
    const auto C = glm::dot(m, m) - c.radius * c.radius;

    // ray pointing away & outside radius
    if (C > 0.0f && B > 0.0f) return {};

    const auto disc = B * B - glm::dot(r.dir, r.dir) * C;
    if (disc < 0.0f) return {};

    float t = (-B - std::sqrt(disc)) / glm::dot(r.dir, r.dir);
    if (t < 0.0f) return {};

    return t;
}

auto ray_cast(ray r, line l) -> std::optional<float>
{
    glm::vec2 line_dir = l.end - l.start;

    const auto rxs = cross_2d(r.dir, line_dir);
    if (std::abs(rxs) < 0.0f) {
        return std::nullopt;
    }

    const auto line_to_ray = l.start - r.start;
    const auto t = cross_2d(line_to_ray, line_dir) / rxs; // distance along ray
    const auto u = cross_2d(line_to_ray, r.dir) / rxs;    // % along line

    if (t < 0.f) return {};            // intersection behind the ray
    if (u < 0.f || u > 1.f) return {}; // intersection outside segment

    return t;
}

// to handle ray to capsule, turn the capsule into two circles and two lines, and check
// each of those, taking the minimum
auto ray_cast(ray r, capsule c) -> std::optional<float>
{
    auto best = std::numeric_limits<float>::infinity();

    const auto capsule_dir = glm::normalize(c.end - c.start);
    const auto side = glm::vec2{-capsule_dir.y, capsule_dir.x}; // perpendicular to the line

    const auto top_line = line{.start=c.start+side*c.radius, .end=c.end+side*c.radius};
    if (auto t = ray_cast(r, top_line)) {
        best = std::min(best, *t);
    }
    
    const auto bot_line = line{.start=c.start-side*c.radius, .end=c.end-side*c.radius};
    if (auto t = ray_cast(r, bot_line)) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, circle{.centre=c.start, .radius=c.radius})) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, circle{.centre=c.end, .radius=c.radius})) {
        best = std::min(best, *t);
    }

    if (!std::isfinite(best))
        return {};

    return best;
}

// to handle ray to box, turn the box into the four bounding lines
auto ray_cast(ray r, box b) -> std::optional<float>
{
    auto best = std::numeric_limits<float>::infinity();

    const auto tl = b.centre + glm::vec2{-b.width, -b.height} / 2.0f;
    const auto tr = b.centre + glm::vec2{ b.width, -b.height} / 2.0f;
    const auto bl = b.centre + glm::vec2{-b.width,  b.height} / 2.0f;
    const auto br = b.centre + glm::vec2{ b.width,  b.height} / 2.0f;

    if (auto t = ray_cast(r, line{tl, tr})) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, line{tr, bl})) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, line{bl, br})) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, line{br, tl})) {
        best = std::min(best, *t);
    }

    if (!std::isfinite(best))
        return {};

    return best;
}

auto ray_cast(ray r, padded_box b) -> std::optional<float>
{
    auto best = std::numeric_limits<float>::infinity();

    const auto tl = b.centre + glm::vec2{-b.width, -b.height} / 2.0f;
    const auto tr = b.centre + glm::vec2{ b.width, -b.height} / 2.0f;
    const auto bl = b.centre + glm::vec2{-b.width,  b.height} / 2.0f;
    const auto br = b.centre + glm::vec2{ b.width,  b.height} / 2.0f;

    if (auto t = ray_cast(r, line{ .start = tl + glm::vec2{0, -b.radius}, .end=tr + glm::vec2{0, -b.radius} })) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, line{ .start = bl + glm::vec2{0, b.radius}, .end=br + glm::vec2{0, b.radius} })) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, line{ .start = tl + glm::vec2{-b.radius, 0}, .end=bl + glm::vec2{-b.radius, 0} })) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, line{ .start = tr + glm::vec2{b.radius, 0}, .end=br + glm::vec2{b.radius, 0} })) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, circle{.centre=tl, .radius=b.radius})) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, circle{.centre=tr, .radius=b.radius})) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, circle{.centre=bl, .radius=b.radius})) {
        best = std::min(best, *t);
    }

    if (auto t = ray_cast(r, circle{.centre=br, .radius=b.radius})) {
        best = std::min(best, *t);
    }

    if (!std::isfinite(best))
        return {};

    return best;
}

auto inflate(circle c, float radius) -> circle
{
    return circle{ .centre=c.centre, .radius=c.radius+radius };
}

auto inflate(line l, float radius) -> capsule
{
    return capsule{ .start=l.start, .end=l.end, .radius=radius };
}

auto inflate(capsule c, float radius) -> capsule
{
    return capsule{ .start=c.start, .end=c.end, .radius=c.radius+radius};
}

auto inflate(box b, float radius) -> padded_box
{
    return padded_box{ .centre=b.centre, .width=b.width, .height=b.height, .radius=radius};
}

auto inflate(padded_box b, float radius) -> padded_box
{
    return padded_box{ .centre=b.centre, .width=b.width, .height=b.height, .radius=b.radius+radius};
}

}