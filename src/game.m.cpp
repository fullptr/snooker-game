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

static constexpr auto pocket_radius = 5.0f;

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

auto add_border(table& t) -> void
{
    static constexpr auto border_width = 5.0f;

    t.border_boxes.push_back(t.sim.add_box({-border_width/2.0f, t.width/2.0f},         border_width, 2*border_width + t.width - 4*pocket_radius)); // left cushion
    t.border_boxes.push_back(t.sim.add_box({t.length+border_width/2.0f, t.width/2.0f}, border_width, 2*border_width + t.width - 4*pocket_radius)); // right cushion

    t.border_boxes.push_back(t.sim.add_box({t.length/4.0f, -border_width/2.0f},        2*border_width + t.length/2.0f - 4*pocket_radius, border_width)); // top left cushion
    t.border_boxes.push_back(t.sim.add_box({3.0f*t.length/4.0f, -border_width/2.0f},   2*border_width + t.length/2.0f - 4*pocket_radius, border_width)); // top right cushion

    t.border_boxes.push_back(t.sim.add_box({t.length/4.0f, t.width+border_width/2.0f},        2*border_width + t.length/2.0f - 4*pocket_radius, border_width)); // bottom left cushion
    t.border_boxes.push_back(t.sim.add_box({3.0f*t.length/4.0f, t.width+border_width/2.0f},   2*border_width + t.length/2.0f - 4*pocket_radius, border_width)); // bottom right cushion
}

auto raycast(glm::vec2 start, glm::vec2 end, float radius, const collider& other) -> std::optional<glm::vec2>
{
    const auto r = ray{.start=start, .dir=end-start};

    const auto contact = std::visit(overloaded{
        // To cast a circle at another circle is the same as casting a point at a circle with the radius sum
        [&](const circle_shape& shape) -> std::optional<float> {
            const auto circ = circle{.centre=other.pos, .radius=radius + shape.radius};
            return ray_to_circle(r, circ);
        },
        [&](const box_shape& shape) -> std::optional<float> {
            const auto b = box{.centre=other.pos, .width=shape.width, .height=shape.height};
            return ray_to_box(r, b);
        },
        [&](const line_shape& shape) -> std::optional<float> {
            const auto c = capsule{.start=other.pos + shape.start, .end=other.pos+shape.end, .radius=radius};
            return ray_to_capsule(r, c);
        },
        [](auto&&) -> std::optional<float> {
            return {};
        }
    }, other.shape);

    if (contact) {
        return r.start + *contact * r.dir;
    }
    return {};
}

struct hit_contact
{
    std::size_t id;
    glm::vec2   cue_ball_pos;
};

auto find_contact_ball(const table& t, glm::vec2 start, glm::vec2 end) -> std::optional<hit_contact>
{
    if (t.object_balls.empty()) { return {}; }
    
    assert_that(std::holds_alternative<circle_shape>(t.sim.get(t.cue_ball.id).shape), "cue ball must be a circle");
    const auto cue_ball_radius = std::get<circle_shape>(t.sim.get(t.cue_ball.id).shape).radius;

    auto ret = std::optional<hit_contact>{};
    auto distance = std::numeric_limits<std::size_t>::max();

    for (const auto& ball : t.object_balls) {
        const auto ray = raycast(start, end, cue_ball_radius, t.sim.get(ball.id));
        if (ray) {
            const auto new_cue_pos = *ray;
            const auto ball_dist = glm::length(new_cue_pos - start);
            if (ball_dist < distance) {
                distance = ball_dist;
                ret = hit_contact{ .id=ball.id, .cue_ball_pos=new_cue_pos };
            }
        }
    }
    {
        const auto ray = raycast(start, end, cue_ball_radius, t.sim.get(t.test));
        if (ray) {
            const auto new_cue_pos = *ray;
            const auto ball_dist = glm::length(new_cue_pos - start);
            if (ball_dist < distance) {
                distance = ball_dist;
                ret = hit_contact{ .id=t.test, .cue_ball_pos=new_cue_pos };
            }
        }
    }
    for (const auto& id : t.border_boxes) {
        const auto ray = raycast(start, end, cue_ball_radius, t.sim.get(id));
        if (ray) {
            const auto new_cue_pos = *ray;
            const auto ball_dist = glm::length(new_cue_pos - start);
            if (ball_dist < distance) {
                distance = ball_dist;
                ret = hit_contact{ .id=id, .cue_ball_pos=new_cue_pos };
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

    auto t = table{182.88f, 91.44f}; // english pool table dimensions in cm (6ft x 3ft)
    t.set_cue_ball({50.0f, t.width / 2.0f});
    add_triangle(t, {0.8f * t.length, t.width / 2.0f});
    add_border(t); // TODO: replace with a better construction
    t.add_pocket({0.0f,            0.0f}, pocket_radius);
    t.add_pocket({t.length / 2.0f, 0.0f}, pocket_radius);
    t.add_pocket({t.length,        0.0f}, pocket_radius);

    t.add_pocket({0.0f,            t.width}, pocket_radius);
    t.add_pocket({t.length / 2.0f, t.width}, pocket_radius);
    t.add_pocket({t.length,        t.width}, pocket_radius);

    // Line test
    const auto start = glm::vec2{20, 20};
    const auto end = glm::vec2{150, 30};
    t.test = t.sim.add_static_line(start, end);
    
    double accumulator = 0.0;
    while (window.is_running()) {
        const double dt = timer.on_update();
        window.begin_frame(clear_colour);
        
        const auto c = converter{window.dimensions(), t.dimensions(), 0.9f};
        
        auto& cue_ball_coll = t.sim.get(t.cue_ball.id);
        const auto aim_direction = glm::normalize(c.to_board(window.mouse_pos()) - cue_ball_coll.pos);
        
        for (const auto event : window.events()) {
            ui.on_event(event);
            if (const auto e = event.get_if<mouse_pressed_event>()) {
                std::get<dynamic_body>(cue_ball_coll.body).vel = 400.0f * aim_direction;
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

        // Draw cue ball
        {
            const auto& ball = t.cue_ball;
            const auto& coll = t.sim.get(ball.id);
            assert_that(std::holds_alternative<circle_shape>(coll.shape), "only supporting balls for now");
            const auto radius = std::get<circle_shape>(coll.shape).radius;
            renderer.push_circle(c.to_screen(coll.pos), ball.colour, c.to_screen(radius));
        }

        // Draw object balls
        const auto contact_ball = find_contact_ball(t, cue_ball_coll.pos, c.to_board(window.mouse_pos()));
        if (contact_ball) {
            const auto& ball = t.cue_ball;
            const auto& coll = t.sim.get(ball.id);
            assert_that(std::holds_alternative<circle_shape>(coll.shape), "only supporting balls for now");
            const auto radius = std::get<circle_shape>(coll.shape).radius;
            
            renderer.push_line(c.to_screen(cue_ball_coll.pos), c.to_screen(contact_ball->cue_ball_pos), adjust_alpha(t.cue_ball.colour, 0.5f), 2.0f);
            renderer.push_circle(c.to_screen(contact_ball->cue_ball_pos), adjust_alpha(t.cue_ball.colour, 0.5f), c.to_screen(radius));
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
            const auto& box = std::get<box_shape>(coll.shape);
            renderer.push_quad(c.to_screen(coll.pos), c.to_screen(box.width), c.to_screen(box.height), 0, from_hex(0x73380b));
        }

        // Draw TEMP line code
        renderer.push_line(c.to_screen(start), c.to_screen(end), from_hex(0x73380b), 2.0f);

        // Draw cue
        renderer.push_line(c.to_screen(cue_ball_coll.pos), c.to_screen(cue_ball_coll.pos) + aim_direction * c.to_screen(5.0f), {0, 0, 1, 1}, 2.0f);

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