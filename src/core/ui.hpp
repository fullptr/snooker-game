#pragma once
#include "buffer.hpp"
#include "shader.hpp"
#include "texture.hpp"
#include "event.hpp"
#include "utility.hpp"
#include "font.hpp"
#include "common.hpp"

#include <array>
#include <unordered_map>
#include <source_location>

#include <glm/glm.hpp>

namespace sudoku {

class widget_key
{
    u64              d_id;
    std::string_view d_file;
    u64              d_line;
    u64              d_column;

public:
    widget_key(u64 id, std::source_location loc = std::source_location::current())
        : d_id{id}
        , d_file{loc.file_name()}
        , d_line{loc.line()}
        , d_column{loc.column()}
    {}
    widget_key(std::source_location loc = std::source_location::current())
        : d_id{0}
        , d_file{loc.file_name()}
        , d_line{loc.line()}
        , d_column{loc.column()}
    {}
    auto operator<=>(const widget_key&) const = default;
    auto operator==(const widget_key&) const -> bool = default;
    auto hash() const -> std::size_t {
        // TODO: Make this better
        static constexpr auto h = std::hash<u64>{};
        return h(d_id) ^ h(d_line) ^ h(d_column);
    }
};

}

template<>
struct std::hash<sudoku::widget_key>
{
    auto operator()(const sudoku::widget_key& wk) const -> std::size_t
    {
        return wk.hash();
    }
};

namespace sudoku {

struct ui_logic_quad
{
    glm::vec2 top_left = {0, 0};
    f32       width = 0;
    f32       height = 0;
    bool      active = false;
    
    f64 hovered_time   = 0.0;
    f64 clicked_time   = 0.0;
    f64 unhovered_time = 0.0;
    f64 unclicked_time = 0.0;

    bool hovered_this_frame = false;
    bool clicked_this_frame = false;
    bool unhovered_this_frame = false;
    bool unclicked_this_frame = false;

    auto is_hovered() const -> bool { return hovered_time > unhovered_time; }
    auto is_clicked() const -> bool { return clicked_time > unclicked_time; }

    auto time_hovered(f64 now) const -> f64 { return glm::max(0.0, now - hovered_time); }
    auto time_clicked(f64 now) const -> f64 { return glm::max(0.0, now - clicked_time); }
    auto time_unhovered(f64 now) const -> f64 { return glm::max(0.0, now - unhovered_time); }
    auto time_unclicked(f64 now) const -> f64 { return glm::max(0.0, now - unclicked_time); }
};

class renderer;
class ui_engine
{
    renderer* d_renderer;
    
    // Data from events
    glm::vec2 d_mouse_pos            = {-1, -1};
    bool      d_clicked_this_frame   = false;
    bool      d_unclicked_this_frame = false;
    
    // Data from update
    f64       d_time                 = 0.0;
    bool      d_capture_mouse        = false;
    
    std::unordered_map<widget_key, ui_logic_quad> d_data;

    const ui_logic_quad& get_data(const widget_key& key, glm::vec2 top_left, f32 width, f32 height) { 
        auto& data = d_data[key];
        data.active = true; // keep this alive
        data.top_left = top_left;
        data.width = width;
        data.height = height;
        return data;
    }

    ui_engine(const ui_engine&) = delete;
    ui_engine& operator=(const ui_engine&) = delete;

public:
    ui_engine(renderer* renderer);

    // Step 1: process events
    bool on_event(const event& e);

    // Step 2: setup ui elements    
    bool button(std::string_view msg, glm::ivec2 pos, i32 width, i32 height, i32 scale, const widget_key& key = {});

    // Step 3: draw
    void end_frame(f64 dt);
};

}