#include "input.hpp"
#include "window.hpp"
#include "utility.hpp"
#include "renderer.hpp"
#include "ui.hpp"
#include "collision.hpp"

#include "table.hpp"
#include "simulation.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/hash.hpp>

#include <format>
#include <print>
#include <initializer_list>
#include <string>
#include <optional>
#include <numbers>
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

struct table_dimensions
{
    float border_width;
    float centre_pocket_radius;
    float corner_pocket_radius;
    
    float centre_pocket_offset;
    float centre_pocket_back_pinch;
};

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

auto add_triangle(table& t, glm::vec2 front_pos) -> void
{
    const auto left = glm::vec2{std::sqrt(3) * ball_radius, -ball_radius};
    const auto down = glm::vec2{0, 2 * ball_radius};

    const auto red = glm::vec4{1, 0, 0, 1};
    const auto yel = glm::vec4{1, 1, 0, 1};
    const auto blk = glm::vec4{0, 0, 0, 1};

    t.add_ball(front_pos + 0.0f * left + 0.0f * down, red);

    t.add_ball(front_pos + 1.0f * left + 0.0f * down, red);
    t.add_ball(front_pos + 1.0f * left + 1.0f * down, yel);

    t.add_ball(front_pos + 2.0f * left + 0.0f * down, yel);
    t.add_ball(front_pos + 2.0f * left + 1.0f * down, blk);
    t.add_ball(front_pos + 2.0f * left + 2.0f * down, red);

    t.add_ball(front_pos + 3.0f * left + 0.0f * down, red);
    t.add_ball(front_pos + 3.0f * left + 1.0f * down, yel);
    t.add_ball(front_pos + 3.0f * left + 2.0f * down, red);
    t.add_ball(front_pos + 3.0f * left + 3.0f * down, yel);

    t.add_ball(front_pos + 4.0f * left + 0.0f * down, yel);
    t.add_ball(front_pos + 4.0f * left + 1.0f * down, yel);
    t.add_ball(front_pos + 4.0f * left + 2.0f * down, red);
    t.add_ball(front_pos + 4.0f * left + 3.0f * down, yel);
    t.add_ball(front_pos + 4.0f * left + 4.0f * down, red);
}

auto add_chain(table& t, const std::vector<glm::vec2>& points)
{
    assert_that(points.size() >= 2, "chain requires 2 points");
    for (std::size_t i = 0; i != points.size() - 1; ++i) {
        t.border_boxes.push_back(t.sim.add_static_line(points[i], points[i+1]));
    }
    t.border_boxes.push_back(t.sim.add_static_line(points.back(), points.front()));
}

auto add_border(table& t) -> void
{
    static constexpr auto cfg = table_dimensions{
        .border_width = 4.0f,
        .centre_pocket_radius = 6.0f,
        .corner_pocket_radius = 7.0f,
        .centre_pocket_offset = 3.0f,
        .centre_pocket_back_pinch = 2.0f
    };

    t.add_pocket({0.0f,            0.0f}, cfg.corner_pocket_radius);
    t.add_pocket({t.length / 2.0f, -cfg.centre_pocket_offset}, cfg.centre_pocket_radius);
    t.add_pocket({t.length,        0.0f}, cfg.corner_pocket_radius);

    t.add_pocket({0.0f,            t.width}, cfg.corner_pocket_radius);
    t.add_pocket({t.length / 2.0f, t.width + cfg.centre_pocket_offset}, cfg.centre_pocket_radius);
    t.add_pocket({t.length,        t.width}, cfg.corner_pocket_radius);

    add_chain(t, std::vector<glm::vec2>{
        // top left pocket
        glm::vec2{cfg.border_width, cfg.corner_pocket_radius + cfg.border_width},
        glm::vec2{-cfg.corner_pocket_radius, 0},
        glm::vec2{0, -cfg.corner_pocket_radius},
        glm::vec2{cfg.corner_pocket_radius + cfg.border_width, cfg.border_width},

        // top centre pocket
        glm::vec2{t.length / 2.0f - cfg.centre_pocket_radius, cfg.border_width},
        glm::vec2{t.length / 2.0f - cfg.centre_pocket_radius + cfg.centre_pocket_back_pinch, -cfg.border_width},
        glm::vec2{t.length / 2.0f + cfg.centre_pocket_radius - cfg.centre_pocket_back_pinch, -cfg.border_width},
        glm::vec2{t.length / 2.0f + cfg.centre_pocket_radius, cfg.border_width},

        // top right pocket
        glm::vec2{t.length - cfg.corner_pocket_radius - cfg.border_width, cfg.border_width},
        glm::vec2{t.length, -cfg.corner_pocket_radius},
        glm::vec2{t.length + cfg.corner_pocket_radius, 0},
        glm::vec2{t.length - cfg.border_width, cfg.corner_pocket_radius + cfg.border_width},

        // bottom right pocket
        glm::vec2{t.length - cfg.border_width, t.width - cfg.corner_pocket_radius - cfg.border_width},
        glm::vec2{t.length + cfg.corner_pocket_radius, t.width},
        glm::vec2{t.length, t.width + cfg.corner_pocket_radius},
        glm::vec2{t.length - cfg.corner_pocket_radius - cfg.border_width, t.width - cfg.border_width},

        // bottom centre pocket
        glm::vec2{t.length / 2.0f + cfg.centre_pocket_radius, t.width - cfg.border_width},
        glm::vec2{t.length / 2.0f + cfg.centre_pocket_radius - cfg.centre_pocket_back_pinch, t.width + cfg.border_width},
        glm::vec2{t.length / 2.0f - cfg.centre_pocket_radius + cfg.centre_pocket_back_pinch, t.width + cfg.border_width},
        glm::vec2{t.length / 2.0f - cfg.centre_pocket_radius, t.width - cfg.border_width},

        // bottom left pocket
        glm::vec2{cfg.corner_pocket_radius + cfg.border_width, t.width - cfg.border_width},
        glm::vec2{0, t.width + cfg.corner_pocket_radius},
        glm::vec2{-cfg.corner_pocket_radius, t.width},
        glm::vec2{cfg.border_width, t.width - cfg.corner_pocket_radius - cfg.border_width},
    });
}

auto cue_trajectory_single_check(ray r, float radius, const collider& other) -> std::optional<float>
{
    return std::visit(overloaded{
        // To cast a circle at another circle is the same as casting a point at a circle with the radius sum
        [&](const circle_shape& shape) -> std::optional<float> {
            const auto c = circle{.centre=other.pos, .radius=shape.radius};
            return ray_cast(r, inflate(c, radius));
        },
        [&](const box_shape& shape) -> std::optional<float> {
            const auto b = box{.centre=other.pos, .width=shape.width, .height=shape.height};
            return ray_cast(r, inflate(b, radius));
        },
        [&](const line_shape& shape) -> std::optional<float> {
            const auto l = line{.start=other.pos + shape.start, .end=other.pos+shape.end};
            return ray_cast(r, inflate(l, radius));
        },
        [](auto&&) -> std::optional<float> {
            return {};
        }
    }, other.shape);
}

auto cue_trajectory(const table& t, glm::vec2 start, glm::vec2 dir) -> std::optional<glm::vec2>
{
    assert_that(std::holds_alternative<circle_shape>(t.sim.get(t.cue_ball.id).shape), "cue ball must be a circle");
    const auto cue_ball_radius = std::get<circle_shape>(t.sim.get(t.cue_ball.id).shape).radius;

    auto best = std::numeric_limits<float>::infinity();

    const auto r = ray{.start=start, .dir=dir};

    for (const auto& ball : t.object_balls) {
        if (const auto R = cue_trajectory_single_check(r, cue_ball_radius, t.sim.get(ball.id))) {
            best = std::min(best, *R);
        }
    }
    for (const auto& id : t.border_boxes) {
        if (const auto R = cue_trajectory_single_check(r, cue_ball_radius, t.sim.get(id))) {
            best = std::min(best, *R);
        }
    }

    if (!std::isfinite(best)) {
        return {};
    }
    return r.start + best * r.dir;
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

struct shot
{
    float     power;
    glm::vec2 direction;
    glm::vec2 start_mouse_pos;
};

auto scene_game(snooker::window& window, snooker::renderer& renderer) -> next_state
{
    using namespace snooker;
    auto timer    = snooker::timer{};
    auto ui       = snooker::ui_engine{&renderer};

    auto t = table{182.88f, 91.44f}; // english pool table dimensions in cm (6ft x 3ft)
    t.set_cue_ball({50.0f, t.width / 2.0f});
    add_triangle(t, {0.8f * t.length, t.width / 2.0f});
    add_border(t); // TODO: replace with a better construction
    
    auto cue = std::optional<shot>{};

    double accumulator = 0.0;
    while (window.is_running()) {
        const double dt = timer.on_update();
        window.begin_frame(clear_colour);
        
        const auto c = converter{window.dimensions(), t.dimensions(), 0.8f};
        
        auto& cue_ball_coll = t.sim.get(t.cue_ball.id);
        const auto aim_direction = cue ? cue->direction : glm::normalize(cue_ball_coll.pos - c.to_board(window.mouse_pos()));
        
        for (const auto event : window.events()) {
            ui.on_event(event);
            if (const auto e = event.get_if<mouse_pressed_event>(); e && e->button == mouse::left) {
                cue = shot{400.0f, aim_direction, c.to_board(window.mouse_pos())};
            }
            if (const auto e = event.get_if<mouse_released_event>(); e && e->button == mouse::left) {
                if (cue) {
                    std::get<dynamic_body>(cue_ball_coll.body).vel = cue->power * aim_direction;
                    cue = {};
                }
            }
        }

        accumulator += dt;
        while (accumulator > simulation::time_step) {
            t.sim.step();
            accumulator -= step;
        }
        
        // Handle pocketed balls
        for (auto& ball : t.object_balls) {
            for (const auto& pocket : t.pockets) {
                assert_that(std::holds_alternative<circle_shape>(t.sim.get(pocket).shape), "pockets must be circles for now");
                assert_that(std::holds_alternative<circle_shape>(t.sim.get(ball.id).shape), "balls must be circles for now");
                const auto ball_r = std::get<circle_shape>(t.sim.get(ball.id).shape).radius;
                const auto pock_r = std::get<circle_shape>(t.sim.get(pocket).shape).radius;
                const auto dist = glm::distance(t.sim.get(ball.id).pos, t.sim.get(pocket).pos);
                if (dist + ball_r < pock_r) {
                    ball.is_pocketed = true;
                    break;
                }
            }
        }
        for (auto& ball : t.object_balls) {
            if (ball.is_pocketed) {
                t.sim.remove(ball.id);
            }
        }
        std::erase_if(t.object_balls, [&](const ball& b) { return b.is_pocketed; }); 

        // Draw table
        const auto delta = 0.0f;
        renderer.push_rect(c.to_screen({-delta, -delta}), c.to_screen(t.length+2*delta), c.to_screen(t.width+2*delta), board_colour);

        // Draw pockets
        for (const auto& id : t.pockets) {
            const auto& coll = t.sim.get(id);
            renderer.push_circle(c.to_screen(coll.pos), from_hex(0x422007), c.to_screen(std::get<circle_shape>(coll.shape).radius));
        }

        renderer.draw(window.width(), window.height());

        // Draw cue ball
        {
            const auto& ball = t.cue_ball;
            const auto& coll = t.sim.get(ball.id);
            assert_that(std::holds_alternative<circle_shape>(coll.shape), "only supporting balls for now");
            const auto radius = std::get<circle_shape>(coll.shape).radius;
            renderer.push_circle(c.to_screen(coll.pos), ball.colour, c.to_screen(radius));
        }

        // Draw object balls
        const auto contact_ball = cue_trajectory(t, cue_ball_coll.pos, aim_direction);
        if (contact_ball) {
            const auto& ball = t.cue_ball;
            const auto& coll = t.sim.get(ball.id);
            assert_that(std::holds_alternative<circle_shape>(coll.shape), "only supporting balls for now");
            const auto radius = std::get<circle_shape>(coll.shape).radius;
            
            renderer.push_line(c.to_screen(cue_ball_coll.pos), c.to_screen(*contact_ball), adjust_alpha(t.cue_ball.colour, 0.5f), 2.0f);
            renderer.push_circle(c.to_screen(*contact_ball), adjust_alpha(t.cue_ball.colour, 0.5f), c.to_screen(radius));
        }

        for (const auto& ball : t.object_balls) {
            const auto& coll = t.sim.get(ball.id);
            assert_that(std::holds_alternative<circle_shape>(coll.shape), "only supporting balls for now");
            const auto radius = std::get<circle_shape>(coll.shape).radius;

            renderer.push_circle(c.to_screen(coll.pos), ball.colour, c.to_screen(radius));
        }

        // Draw the boundary boxes
        for (const auto id : t.border_boxes) {
            const auto& coll = t.sim.get(id);
            std::visit(overloaded{
                [&](const box_shape& b) {
                    renderer.push_quad(c.to_screen(coll.pos), c.to_screen(b.width), c.to_screen(b.height), 0, from_hex(0x73380b));
                },
                [&](const line_shape& l) {
                    renderer.push_line(c.to_screen(coll.pos + l.start), c.to_screen(coll.pos + l.end), from_hex(0x73380b), 2.0f);
                },
                [&](auto&&) {}
            }, coll.shape);
        }

        // Draw cue
        renderer.push_line(c.to_screen(cue_ball_coll.pos), c.to_screen(cue_ball_coll.pos) + aim_direction * c.to_screen(5.0f), {0, 0, 1, 1}, 2.0f);

        if (cue) {
            const auto curr_mouse_pos = c.to_board(window.mouse_pos());

            const auto A = line{.start=cue_ball_coll.pos, .end=cue->start_mouse_pos};
            const auto B = line{.start=cue_ball_coll.pos, .end=curr_mouse_pos};

            const auto magnitude = glm::max(glm::dot(A.rel(), B.rel()) / glm::length(A.rel()), 0.0f);
            const auto dir = A.rel() / glm::length(A.rel());
            const auto relvel = magnitude * A.dir();

            const auto C = line{.start=cue_ball_coll.pos, .end=cue_ball_coll.pos + relvel};
            renderer.push_line(c.to_screen(C.start), c.to_screen(C.end), {1, 0, 0, 1}, 2.0f);
        }

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