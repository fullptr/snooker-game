#include "ui.hpp"
#include "utility.hpp"
#include "renderer.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <ranges>
#include <print>

namespace snooker {

ui_engine::ui_engine(renderer* renderer)
    : d_renderer{renderer}
{
}

void ui_engine::end_frame(f64 dt)
{
    // Clean out any elements no longer around
    std::erase_if(d_data, [&](auto& elem) {
        return !elem.second.active;
    });

    d_time += dt;
    d_capture_mouse = false;
    
    for (auto& [hash, data] : d_data) {
        data.active = false; // if made next frame, it will activate again
        data.hovered_this_frame = false;
        data.clicked_this_frame = false;
        data.unhovered_this_frame = true;
        data.unclicked_this_frame = true;
        
        if (d_unclicked_this_frame && data.is_clicked()) {
            data.unclicked_this_frame = true;
            data.unclicked_time = d_time;
        }
        
        if (is_in_region(d_mouse_pos, data.top_left, data.width, data.height)) {
            d_capture_mouse = true;

            if (!data.is_hovered()) {
                data.hovered_this_frame = true;
                data.hovered_time = d_time;
            }
            if (d_clicked_this_frame) {
                data.clicked_this_frame = true;
                data.clicked_time = d_time;
            }
        } else if (data.is_hovered()) {
            data.unhovered_this_frame = true;
            data.unhovered_time = d_time;
        }
    }
    
    d_clicked_this_frame = false;
    d_unclicked_this_frame = false;
}

bool ui_engine::on_event(const event& event)
{
    if (const auto e = event.get_if<mouse_moved_event>()) {
        d_mouse_pos = e->pos;
    }
    else if (const auto e = event.get_if<mouse_pressed_event>()) {
        if (e->button == mouse::left && d_capture_mouse) {
            d_clicked_this_frame = true;
            return true;
        }
    }
    else if (const auto e = event.get_if<mouse_released_event>()) {
        d_unclicked_this_frame = true;
    }
    return false;
}

bool ui_engine::button(
    std::string_view msg, glm::ivec2 pos, i32 width, i32 height, i32 scale, const widget_key& key)
{
    const auto& data = get_data(key, pos, width, height);
    
    constexpr auto unhovered_colour = from_hex(0x2c3e50);
    constexpr auto hovered_colour = from_hex(0x34495e);
    constexpr auto clicked_colour = from_hex(0xbdc3c7);

    const auto lerp_time = 0.1;
    
    auto colour = unhovered_colour;
    if (data.is_clicked()) {
        colour = clicked_colour;
    }
    else if (data.is_hovered()) {
        const auto t = std::clamp(data.time_hovered(d_time) / lerp_time, 0.0, 1.0);
        colour = snooker::lerp(unhovered_colour, hovered_colour, t);
    }
    else if (d_time > lerp_time) { // Don't start the game looking hovered
        const auto t = std::clamp(data.time_unhovered(d_time) / lerp_time, 0.0, 1.0);
        colour = snooker::lerp(hovered_colour, unhovered_colour, t);
    }
    
    d_renderer->push_rect(pos, width, height, colour);
    d_renderer->push_text_box(msg, pos, width, height, scale, from_hex(0xecf0f1));
    return data.clicked_this_frame;
}

}