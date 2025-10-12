#pragma once
#include "event.hpp"

#include <bitset>

namespace snooker {

class input
{
    std::bitset<348> d_keyboard_down;
    std::bitset<348> d_keyboard_down_this_frame;
    
    std::bitset<8> d_mouse_down;
    std::bitset<8> d_mouse_down_this_frame;
    glm::vec2      d_position_last_frame;
    glm::vec2      d_positiion_this_frame;

public:
    auto on_event(const event& e) -> void;
    auto on_new_frame() -> void;

    auto is_down(keyboard key) const -> bool;
    auto is_down_this_frame(keyboard key) const -> bool;
    
    auto is_down(mouse button) const -> bool;
    auto is_down_this_frame(mouse button) const-> bool;

    auto offset() const -> glm::vec2;
    auto position() const -> glm::vec2;
};
    
}