#pragma once
#include <chrono>
#include <concepts>
#include <cstddef>
#include <string>
#include <span>
#include <format>
#include <filesystem>
#include <source_location>
#include <print>

#include <glm/glm.hpp>

namespace snooker {

static constexpr auto up = glm::ivec2{0, -1};
static constexpr auto right = glm::ivec2{1, 0};
static constexpr auto down = glm::ivec2{0, 1};
static constexpr auto left = glm::ivec2{-1, 0};
static constexpr auto offsets = {up, right, down, left};

static constexpr auto step = 1.0 / 60.0;

using i8  = std::int8_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;

using u8  = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using f32 = float;
using f64 = double;

static constexpr u64 u64_max = std::numeric_limits<u64>::max();

static_assert(std::is_same_v<u64, std::size_t>);

inline auto assert_that(
    bool condition,
    std::string_view message = {},
    std::source_location loc = std::source_location::current()
)
    -> void
{
    if (!condition) {
        std::print("FAILED ASSERTION: ({}) {}\n", loc.line(), message);
        std::terminate();
    }
}

template <typename... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};

class timer
{
public:
    using clock = std::chrono::steady_clock;

private:
    clock d_clock;
    clock::time_point d_prev_time;
    clock::time_point d_curr_time;
    clock::time_point d_last_time_printed;
    
    std::uint32_t d_frame_count;
    std::uint32_t d_frame_rate = 0;

public:
    timer();
    auto on_update() -> double;
    auto frame_rate() const -> std::uint32_t { return d_frame_rate; }

    auto now() const -> clock::time_point;
};

using time_point = typename timer::clock::time_point;
using namespace std::chrono_literals;

auto random_from_range(float min, float max) -> float;
auto random_from_range(int min, int max) -> int;
auto random_from_circle(float radius) -> glm::ivec2;

auto random_normal(float centre, float sd) -> float;

template <typename Elements>
auto random_element(const Elements& elements)
{
    return elements[random_from_range(0, std::ssize(elements) - 1)];
}

auto coin_flip() -> bool;
auto sign_flip() -> int;
auto random_unit() -> float; // Same as random_from_range(0.0f, 1.0f)

constexpr auto from_hex(int hex, float alpha = 1.0f) -> glm::vec4
{
    const auto blue = static_cast<float>(hex & 0xff) / 256.0f;
    hex /= 0x100;
    const auto green = static_cast<float>(hex & 0xff) / 256.0f;
    hex /= 0x100;
    const auto red = static_cast<float>(hex & 0xff) / 256.0f;
    return glm::vec4{red, green, blue, alpha};
}

auto get_executable_filepath() -> std::filesystem::path;

template <typename T>
auto lerp(const T& a, const T& b, float t) -> T
{
    return t * b + (1 - t) * a;
};

inline auto to_string(glm::ivec2 v) -> std::string
{
    return std::format("glm::ivec2{{{}, {}}}", v.x, v.y);
}

inline auto to_string(glm::vec2 v) -> std::string
{
    return std::format("glm::vec2{{{}, {}}}", v.x, v.y);
}

template <typename... Args>
auto format_to(std::span<char> buffer, std::format_string<Args...> fmt, Args&&... args) -> std::string_view
{
    const auto result = std::format_to_n(buffer.data(), buffer.size(), fmt, std::forward<Args>(args)...);
    return {buffer.data(), result.out};
}

template <typename T>
concept has_to_string_member = requires(T obj)
{
    { obj.to_string() } -> std::convertible_to<std::string>;
};

template <has_to_string_member T>
struct std::formatter<T> : std::formatter<std::string>
{
    auto format(const T& obj, auto& ctx) const {
        return std::formatter<std::string>::format(obj.to_string(), ctx);
    }
};

template <typename T>
concept has_to_string_free_function = requires(T obj)
{
    { snooker::to_string(obj) } -> std::convertible_to<std::string>;
};

template <has_to_string_free_function T>
struct std::formatter<T> : std::formatter<std::string>
{
    auto format(const T& obj, auto& ctx) const {
        return std::formatter<std::string>::format(snooker::to_string(obj), ctx);
    }
};

template <typename T>
const T& signed_index(const std::vector<T>& v, int index)
{
    while (index < 0) index += v.size();
    while (index >= v.size()) index -= v.size();
    return v[index];
}

inline auto is_in_region(glm::vec2 pos, glm::vec2 top_left, f32 width, f32 height) -> bool
{
    return top_left.x <= pos.x && pos.x < top_left.x + width
        && top_left.y <= pos.y && pos.y < top_left.y + height;
}

inline auto clamp(double val, double lo, double hi) -> double
{
    return std::min(std::max(lo, val), hi);
}

}