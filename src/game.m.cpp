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
#include <source_location>

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

auto assert_that(bool condition, std::string_view message = {}, std::source_location loc = std::source_location::current()) -> void
{
    if (!condition) {
        std::print("FAILED ASSERTION: ({}) {}\n", loc.line(), message);
        std::terminate();
    }
}

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
    float radius = ball_radius;
};

auto update_ball(ball& b, const table& t, float dt) -> void
{
    b.pos += b.vel * dt;
    b.vel *= 0.98f;
    
    if (b.pos.x - b.radius < 0) {
        b.pos.x = b.radius;
        b.vel.x = -b.vel.x;
    }
    if (b.pos.x + b.radius > t.length) {
        b.pos.x = t.length - b.radius;
        b.vel.x = -b.vel.x;
    }
    
    if (b.pos.y - b.radius < 0) {
        b.pos.y = b.radius;
        b.vel.y = -b.vel.y;
    }
    if (b.pos.y + b.radius > t.width) {
        b.pos.y = t.width - b.radius;
        b.vel.y = -b.vel.y;
    }
}

// TODO: handle balls intersecting (tunnelling)
auto update_ball_collision(ball& a, ball& b, const table& t, float dt) -> void
{
    if (glm::length(a.pos - b.pos) > (a.radius + b.radius)) {
        return; // no contact
    }

    constexpr auto restitution = 0.8f;
    
    const auto dp = a.pos - b.pos;
    const auto dv = a.vel - b.vel;
    
    if (glm::length2(dp) == 0) {
        return;
    }

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

    balls.push_back(ball{ front_pos + 1.0f * left + 0.0f * down, {0.0f, 0.0f}, red });
    balls.push_back(ball{ front_pos + 1.0f * left + 1.0f * down, {0.0f, 0.0f}, yel });

    balls.push_back(ball{ front_pos + 2.0f * left + 0.0f * down, {0.0f, 0.0f}, yel });
    balls.push_back(ball{ front_pos + 2.0f * left + 1.0f * down, {0.0f, 0.0f}, blk });
    balls.push_back(ball{ front_pos + 2.0f * left + 2.0f * down, {0.0f, 0.0f}, red });

    balls.push_back(ball{ front_pos + 3.0f * left + 0.0f * down, {0.0f, 0.0f}, red });
    balls.push_back(ball{ front_pos + 3.0f * left + 1.0f * down, {0.0f, 0.0f}, yel });
    balls.push_back(ball{ front_pos + 3.0f * left + 2.0f * down, {0.0f, 0.0f}, red });
    balls.push_back(ball{ front_pos + 3.0f * left + 3.0f * down, {0.0f, 0.0f}, yel });

    balls.push_back(ball{ front_pos + 4.0f * left + 0.0f * down, {0.0f, 0.0f}, yel });
    balls.push_back(ball{ front_pos + 4.0f * left + 1.0f * down, {0.0f, 0.0f}, yel });
    balls.push_back(ball{ front_pos + 4.0f * left + 2.0f * down, {0.0f, 0.0f}, red });
    balls.push_back(ball{ front_pos + 4.0f * left + 3.0f * down, {0.0f, 0.0f}, yel });
    balls.push_back(ball{ front_pos + 4.0f * left + 4.0f * down, {0.0f, 0.0f}, red });
}

struct raycast_info
{
    float     distance_from_line;
    float     distance_along_line;
    glm::vec2 dir;
};

auto raycast(glm::vec2 start, glm::vec2 end, const ball& cue_ball, const ball& b) -> std::optional<raycast_info>
{
    const auto dir = glm::normalize(end - start);
    const auto v = b.pos - start;
    const auto cross = v.x * dir.y - v.y * dir.x;
    const auto distance_from = glm::abs(cross);
    if (distance_from > (cue_ball.radius + b.radius)) {
        return {};
    }

    const auto distance_along = glm::sqrt(glm::length2(v) - distance_from * distance_from);
    if (glm::dot(dir, b.pos - start) < 0) { // only raycast forward
        return {};
    }

    return raycast_info{ distance_from, distance_along, dir };
}

struct hit_contact
{
    std::size_t ball_index;
    glm::vec2   cue_ball_pos;
};

auto find_contact_ball(const std::vector<ball>& balls, glm::vec2 start, glm::vec2 end) -> std::optional<hit_contact>
{
    assert_that(!balls.empty(), "balls should never be empty");

    auto ret = std::optional<hit_contact>{};
    auto distance = std::numeric_limits<std::size_t>::max();

    for (std::size_t i = 1; i != balls.size(); ++i) {
        const auto ray = raycast(start, end, balls[0], balls[i]);
        if (ray) {
            const auto rad_sum = (balls[0].radius + balls[i].radius);
            const auto new_cue_pos = start + ray->dir * (ray->distance_along_line - glm::sqrt(std::powf(rad_sum, 2) - std::powf(ray->distance_from_line, 2)));
            const auto ball_dist = glm::length(new_cue_pos - start);
            if (ball_dist < distance) {
                distance = ball_dist;
                ret = hit_contact{ .ball_index=i, .cue_ball_pos=new_cue_pos };
            }
        }
    }
    return ret;
}

class converter
{
    float     d_board_to_screen;
    glm::vec2 d_top_left;

public:
    converter(glm::vec2 window_dim, glm::vec2 table_dim, float screen_fill_factor)
        : d_board_to_screen{(screen_fill_factor * window_dim.x) / table_dim.x}
        , d_top_left{(window_dim / d_board_to_screen - table_dim) / 2.0f}
    {}

    auto to_board(glm::vec2 value) const -> glm::vec2 { return value / d_board_to_screen - d_top_left; }
    auto to_screen(glm::vec2 value) const -> glm::vec2 { return (d_top_left + value) * d_board_to_screen; }
    auto to_screen(float value) const -> float { return value * d_board_to_screen; }
};

auto adjust_alpha(glm::vec4 colour, float alpha) -> glm::vec4
{
    colour.a = alpha;
    return colour;
}

auto scene_game(snooker::window& window, snooker::renderer& renderer) -> next_state
{
    using namespace snooker;
    auto timer    = snooker::timer{};
    auto ui       = snooker::ui_engine{&renderer};

    auto pool_table = table{182.88f, 91.44f}; // english pool table dimensions in cm (6ft x 3ft)
    auto pool_balls = std::vector{
        ball{{50.0f, pool_table.width / 2.0f}, {0.0f, 0.0f}, {1, 1, 1, 1}}, // ball 0 is always the cue ball
    };
    add_triangle(pool_balls, {0.8f * pool_table.length, pool_table.width / 2.0f});
    
    double accumulator = 0.0;
    constexpr double time_step = 1.0 / 10.0;
    while (window.is_running()) {
        const double dt = timer.on_update();
        window.begin_frame(clear_colour);
        
        const auto c = converter{window.dimensions(), pool_table.dimensions(), 0.9f};
        
        auto& cue_ball = pool_balls[0];
        const auto aim_direction = glm::normalize(c.to_board(window.mouse_pos()) - cue_ball.pos);
        
        for (const auto event : window.events()) {
            ui.on_event(event);
            if (const auto e = event.get_if<mouse_pressed_event>()) {
                cue_ball.vel = 200.0f * aim_direction;
            }
        }

        accumulator += dt;
        while (accumulator > time_step) {
            // Update ball positions
            for (std::size_t i = 0; i != pool_balls.size(); ++i) {
                for (std::size_t j = i + 1; j != pool_balls.size(); ++j) {
                    update_ball_collision(pool_balls[i], pool_balls[j], pool_table, time_step);
                }
            }
    
            for (auto& ball : pool_balls) {
                update_ball(ball, pool_table, time_step);
            }
            accumulator -= time_step;
        }


        // Draw table
        renderer.push_rect(c.to_screen({0, 0}), c.to_screen(pool_table.length), c.to_screen(pool_table.width), board_colour);

        // Draw balls
        const auto contact_ball = find_contact_ball(pool_balls, cue_ball.pos, c.to_board(window.mouse_pos()));
        for (std::size_t i = 0; i != pool_balls.size(); ++i) { // This assumes that balls[0] is the cue ball
            const auto& ball = pool_balls[i];
            if (contact_ball && contact_ball->ball_index == i) {
                renderer.push_line(c.to_screen(cue_ball.pos), c.to_screen(contact_ball->cue_ball_pos), {1, 1, 1, 0.5f}, 2.0f);
                renderer.push_circle(c.to_screen(contact_ball->cue_ball_pos), adjust_alpha(cue_ball.colour, 0.5f), c.to_screen(cue_ball.radius));
            }
            renderer.push_circle(c.to_screen(ball.pos), ball.colour, c.to_screen(ball.radius));
        }

        // Draw cue
        renderer.push_line(c.to_screen(cue_ball.pos), c.to_screen(cue_ball.pos) + aim_direction * c.to_screen(5.0f), {0, 0, 1, 1}, 2.0f);

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