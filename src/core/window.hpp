#pragma once
#include "event.hpp"
#include "utility.hpp"

#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <cstdint>
#include <span>
#include <functional>

struct GLFWwindow;

namespace snooker {

using window_callback = std::function<void(const event&)>;

struct window_data
{
    int width;
    int height;

    bool fullscreen;
    bool running;
    bool focused;

    GLFWwindow* native_window = nullptr;

    std::vector<event> events;
};

class window
{
   window_data d_data;

    window(const window&) = delete;
    window& operator=(const window&) = delete;

    window(window&&) = delete;
    window& operator=(window&&) = delete;

public:
    window(const char* name, int width, int height);
    ~window();

    auto begin_frame(glm::vec4 colour = {0, 0, 0, 1}) -> void;
    auto end_frame() -> void;
    auto events() -> std::span<const event>;

    auto mouse_pos() const -> glm::ivec2;

    auto is_running() const -> bool;

    auto width() const -> int;
    auto height() const -> int;
    auto dimensions() const -> glm::vec2;

    auto native_handle() -> GLFWwindow*;
};

}