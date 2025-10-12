#include "input.hpp"
#include "event.hpp"

#include <type_traits>
#include <utility>

namespace snooker {

auto input::on_event(const event& event) -> void
{
    if (const auto e = event.get_if<snooker::mouse_pressed_event>()) {
        d_mouse_down[std::to_underlying(e->button)] = true;
        d_mouse_down_this_frame[std::to_underlying(e->button)] = true;
    }
    else if (const auto e = event.get_if<snooker::mouse_released_event>()) {
        d_mouse_down[std::to_underlying(e->button)] = false;
    }
    else if (const auto e = event.get_if<snooker::mouse_moved_event>()) {
        d_positiion_this_frame = e->pos;
    }
    else if (const auto e = event.get_if<snooker::keyboard_pressed_event>()) {
        d_keyboard_down[std::to_underlying(e->key)] = true;
        d_keyboard_down_this_frame[std::to_underlying(e->key)] = true;
    }
    else if (const auto e = event.get_if<snooker::keyboard_released_event>()) {
        d_keyboard_down[std::to_underlying(e->key)] = false;
    }
}

auto input::on_new_frame() -> void
{
    d_keyboard_down_this_frame.reset();
    d_mouse_down_this_frame.reset();
    d_position_last_frame = d_positiion_this_frame;
}

auto input::is_down(mouse button) const -> bool
{
    return d_mouse_down.test(std::to_underlying(button));
}

auto input::is_down_this_frame(mouse button) const -> bool
{
    return d_mouse_down_this_frame.test(std::to_underlying(button));
}

auto input::offset() const -> glm::vec2
{
    return d_positiion_this_frame - d_position_last_frame;
}

auto input::position() const -> glm::vec2
{
    return d_positiion_this_frame;
}

auto input::is_down(keyboard key) const -> bool
{
    return d_keyboard_down.test(std::to_underlying(key));
}

auto input::is_down_this_frame(keyboard key) const -> bool
{
    return d_keyboard_down_this_frame.test(std::to_underlying(key));
}

}