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

auto scene_main_menu(snooker::window& window) -> next_state
{
    auto timer = snooker::timer{};
    auto renderer = snooker::renderer{};
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
struct board
{
    f32 length = 182.88f;
    f32 width = 91.44;
};

auto scene_game(snooker::window& window) -> next_state
{
    using namespace snooker;
    auto timer    = snooker::timer{};
    auto renderer = snooker::renderer{};
    auto ui       = snooker::ui_engine{&renderer};

    const auto ball_radius = 2.54f; // english pool bool cm == 1 inch
    const auto board_colour = from_hex(0x3db81e);
    auto b = board{};
    
    while (window.is_running()) {
        const double dt = timer.on_update();
        window.begin_frame(clear_colour);
        
        for (const auto event : window.events()) {
            ui.on_event(event);
        }
        
        const auto board_to_screen = (0.9f * window.width()) / b.length;

        renderer.push_quad({window.width() / 2, window.height() / 2}, b.length * board_to_screen, b.width * board_to_screen, 0, board_colour);
        renderer.push_circle({window.width() / 2, window.height() / 2}, {1, 1, 1, 1}, ball_radius * board_to_screen);

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
    auto next   = next_state::main_menu;

    while (true) {
        switch (next) {
            case next_state::main_menu: {
                next = scene_main_menu(window);
            } break;
            case next_state::game: {
                next = scene_game(window);
            } break;
            case next_state::exit: {
                std::print("closing game\n");
                return 0;
            } break;
        }
    }
    
    return 0;
}