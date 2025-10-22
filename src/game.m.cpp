#include "common.hpp"
#include "input.hpp"
#include "window.hpp"
#include "utility.hpp"
#include "renderer.hpp"
#include "ui.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/hash.hpp>

#include <format>
#include <print>
#include <initializer_list>
#include <string>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <unordered_set>

using namespace snooker;

enum class next_state
{
    main_menu,
    game,
    exit,
};

constexpr auto clear_colour = snooker::from_hex(0x222f3e);
constexpr auto ball_radius = 2.54f; // english pool bool cm == 1 inch
constexpr auto ball_mass = 140.0f; // grams
constexpr auto board_colour = from_hex(0x3db81e);
constexpr auto break_speed = 983.49f; // cm

auto scene_main_menu(snooker::window& window, snooker::renderer& renderer) -> next_state
{
    auto timer = snooker::timer{};
    auto ui    = snooker::ui_engine{&renderer};

    while (window.is_running()) {
        const double dt = timer.on_update();
        window.begin_frame(clear_colour);

        for (const auto event : window.events()) {
            ui.on_event(event);
        }
        
        const auto scale = 3.0f;
        const auto button_width = 200;
        const auto button_height = 50;
        const auto button_left = (window.width() - button_width) / 2;

        if (ui.button("Start Game", {button_left, 100}, button_width, button_height, scale)) {
            std::print("starting game!\n");
            return next_state::game;
        }

        if (ui.button("Exit", {button_left, 160}, button_width, button_height, scale)) {
            std::print("exiting!\n");
            return next_state::exit;
        }

        const auto para_left = 100;
        const auto para_top = 300;
        constexpr auto colour = snooker::from_hex(0xecf0f1);

        const auto lines = {
            "Lorem ipsum dolor sit amet, consectetur adipiscing elit,",
            "sed do eiusmod tempor incididunt ut labore et dolore magna",
            "aliqua. Ut enim ad minim veniam, quis nostrud exercitation",
            "ullamco laboris nisi ut aliquip ex ea commodo consequat.",
            "Duis aute irure dolor in reprehenderit in voluptate velit",
            "esse cillum dolore eu fugiat nulla pariatur. Excepteur",
            "sint occaecat cupidatat non proident, sunt in culpa",
            "qui officia deserunt mollit anim id est laborum.",
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ abcdefghijklmnopqrstuvwxyz",
            "0123456789 () {} [] ^ < > - _ = + ! ? : ; . , @ % $ / \\ \" ' # ~ & | `"
        };
        for (const auto [index, line] : std::views::enumerate(lines)) {
            renderer.push_text(line, {para_left, para_top + index * 11 * scale}, scale, colour);
        }

        std::array<char, 8> buf = {};
        renderer.push_text_box(snooker::format_to(buf, "{}", timer.frame_rate()), {0, 0}, 120, 50, 3, colour);
        ui.end_frame(dt);

        renderer.draw(window.width(), window.height());
        window.end_frame();
    }

    return next_state::exit;
}

// dimensions are an english pool table in cm (6ft x 3ft)
struct table
{
    f32 length;
    f32 width;

    auto dimensions() -> glm::vec2 { return {length, width}; }
};

struct ball
{
    glm::vec2 pos;
    glm::vec2 vel;
    glm::vec4 colour;

    float mass = ball_mass;
};

// TODO: handle balls intersecting
auto update_ball(ball& b, const table& t, float dt) -> void
{
    b.pos += b.vel * dt;
    b.vel *= 0.98f;

    if (b.pos.x - ball_radius < 0) {
        b.pos.x = ball_radius;
        b.vel.x = -b.vel.x;
    }
    if (b.pos.x + ball_radius > t.length) {
        b.pos.x = t.length - ball_radius;
        b.vel.x = -b.vel.x;
    }

    if (b.pos.y - ball_radius < 0) {
        b.pos.y = ball_radius;
        b.vel.y = -b.vel.y;
    }
    if (b.pos.y + ball_radius > t.width) {
        b.pos.y = t.width - ball_radius;
        b.vel.y = -b.vel.y;
    }
}

// TODO: allow for balls of different masses
auto update_ball_collision(ball& a, ball& b, const table& t, float dt) -> void
{
    if (glm::length(a.pos - b.pos) > 2 * ball_radius) {
        return; // no contact
    }

    
    constexpr auto restitution = 0.8f;
    
    const auto dp = a.pos - b.pos;
    const auto dv = a.vel - b.vel;
    
    if (glm::length2(dp) == 0) {
        return;
    }

    const auto overlap = (ball_radius * 2) - glm::length(dp);
    const auto correction = glm::normalize(dp) * (overlap / 2.0f);
    a.pos += correction;
    b.pos -= correction;

    const auto length2 = glm::dot(dp, dp);
    if (length2 == 0.0f) {
        return; // avoid division by zero
    }

    const auto normal = glm::normalize(dp);
    const auto vel_along_normal = glm::dot(dv, normal);

    if (vel_along_normal >= 0.0f) {
        return; // moving away so no need to resolve
    }

    const auto inverse_mass = (1.0f / a.mass) + (1.0f / b.mass);
    const auto impulse_size = -(1.0f + restitution) * vel_along_normal / inverse_mass;
    const auto impulse = impulse_size * normal;

    a.vel += impulse / a.mass;
    b.vel -= impulse / b.mass;
}

auto add_triangle(std::vector<ball>& balls, glm::vec2 front_pos)
{
    const auto left = glm::vec2{std::sqrt(3) * ball_radius, -ball_radius};
    const auto down = glm::vec2{0, 2 * ball_radius};

    const auto red = glm::vec4{1, 0, 0, 1};
    const auto yel = glm::vec4{1, 1, 0, 1};
    const auto blk = glm::vec4{0, 0, 0, 1};

    balls.push_back(ball{ front_pos + 0.0f * left + 0.0f * down, {0.0f, 0.0f}, red });

    //balls.push_back(ball{ front_pos + 1.0f * left + 0.0f * down, {0.0f, 0.0f}, red });
    //balls.push_back(ball{ front_pos + 1.0f * left + 1.0f * down, {0.0f, 0.0f}, yel });
//
    //balls.push_back(ball{ front_pos + 2.0f * left + 0.0f * down, {0.0f, 0.0f}, yel });
    //balls.push_back(ball{ front_pos + 2.0f * left + 1.0f * down, {0.0f, 0.0f}, blk });
    //balls.push_back(ball{ front_pos + 2.0f * left + 2.0f * down, {0.0f, 0.0f}, red });
//
    //balls.push_back(ball{ front_pos + 3.0f * left + 0.0f * down, {0.0f, 0.0f}, red });
    //balls.push_back(ball{ front_pos + 3.0f * left + 1.0f * down, {0.0f, 0.0f}, yel });
    //balls.push_back(ball{ front_pos + 3.0f * left + 2.0f * down, {0.0f, 0.0f}, red });
    //balls.push_back(ball{ front_pos + 3.0f * left + 3.0f * down, {0.0f, 0.0f}, yel });
//
    //balls.push_back(ball{ front_pos + 4.0f * left + 0.0f * down, {0.0f, 0.0f}, yel });
    //balls.push_back(ball{ front_pos + 4.0f * left + 1.0f * down, {0.0f, 0.0f}, yel });
    //balls.push_back(ball{ front_pos + 4.0f * left + 2.0f * down, {0.0f, 0.0f}, red });
    //balls.push_back(ball{ front_pos + 4.0f * left + 3.0f * down, {0.0f, 0.0f}, yel });
    //balls.push_back(ball{ front_pos + 4.0f * left + 4.0f * down, {0.0f, 0.0f}, red });
}

// Assumes that the direction vector is length 1
auto line_intersect(glm::vec2 start, glm::vec2 dir, glm::vec2 ball_pos, float ball_radius) -> bool
{
    if (start == ball_pos) return false; // exclude cue ball
    
    const auto v = ball_pos - start;
    const auto cross = v.x * dir.y - v.y * dir.x;
    const auto distance = glm::abs(cross);
    return distance < ball_radius;
}

auto scene_game(snooker::window& window, snooker::renderer& renderer) -> next_state
{
    using namespace snooker;
    auto timer    = snooker::timer{};
    auto ui       = snooker::ui_engine{&renderer};

    auto pool_table = table{182.88f, 91.44f}; // english pool table dimensions in cm (6ft x 3ft)
    auto pool_balls = std::vector{
        ball{{50.0f, pool_table.width / 2.0f}, {0.0f, 0.0f}, {1, 1, 1, 1}},
    };
    add_triangle(pool_balls, {0.8f * pool_table.length, pool_table.width / 2.0f});
    
    while (window.is_running()) {
        const double dt = timer.on_update();
        window.begin_frame(clear_colour);
        
        auto& cue_ball = pool_balls[0];
        const auto board_to_screen = (0.9f * window.width()) / pool_table.length;
        const auto top_left = window.dimensions() / 2.0f - pool_table.dimensions() * board_to_screen / 2.0f;
        const auto aim_direction = -glm::normalize(top_left + cue_ball.pos * board_to_screen - glm::vec2{window.mouse_pos()});
        
        for (const auto event : window.events()) {
            ui.on_event(event);
            if (const auto e = event.get_if<mouse_pressed_event>()) {
                cue_ball.vel = 200.0f * aim_direction;
            }
        }

        // TODO: Fix the time step of the simulation with an accumulator

        // Update ball positions
        for (std::size_t i = 0; i != pool_balls.size(); ++i) {
            for (std::size_t j = i + 1; j != pool_balls.size(); ++j) {
                update_ball_collision(pool_balls[i], pool_balls[j], pool_table, (float)dt);
            }
        }

        for (auto& ball : pool_balls) {
            update_ball(ball, pool_table, (float)dt);
        }

        // Draw table
        renderer.push_quad({window.width() / 2, window.height() / 2}, pool_table.length * board_to_screen, pool_table.width * board_to_screen, 0, board_colour);

        // Draw balls
        int index = 0;
        for (const auto& ball : pool_balls) {
            if (line_intersect(
                    top_left + cue_ball.pos * board_to_screen,
                    glm::normalize(glm::vec2{window.mouse_pos()} - (top_left + cue_ball.pos * board_to_screen)),
                    top_left + ball.pos * board_to_screen,
                    ball_radius * board_to_screen))
            {
                renderer.push_circle(top_left + ball.pos * board_to_screen, glm::vec4{0, 1, 1, 1}, ball_radius * board_to_screen);
            } else {
                renderer.push_circle(top_left + ball.pos * board_to_screen, ball.colour, ball_radius * board_to_screen);
            }
        }

        // Draw cue
        renderer.push_line(top_left + cue_ball.pos * board_to_screen, window.mouse_pos(), {0, 0, 1, 0.5f}, 2.0f);
        renderer.push_line(top_left + cue_ball.pos * board_to_screen, top_left + cue_ball.pos * board_to_screen + aim_direction * 5.0f * board_to_screen, {0, 0, 1, 1}, 2.0f);

        if (ui.button("Back", {0, 0}, 200, 50, 3)) {
            return next_state::main_menu;
        }

        ui.end_frame(dt);
        renderer.draw(window.width(), window.height());
        window.end_frame();
    }

    return next_state::exit;
}

auto main() -> int
{
    using namespace snooker;

    auto window = snooker::window{"Snooker Game", 1280, 720};
    auto renderer = snooker::renderer{};
    auto next   = next_state::main_menu;

    while (true) {
        switch (next) {
            case next_state::main_menu: {
                next = scene_main_menu(window, renderer);
            } break;
            case next_state::game: {
                next = scene_game(window, renderer);
            } break;
            case next_state::exit: {
                std::print("closing game\n");
                return 0;
            } break;
        }
    }
    
    return 0;
}