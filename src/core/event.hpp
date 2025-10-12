#pragma once
#include <glm/glm.hpp>

#include <variant>
#include <string>
#include <cstdint>

namespace snooker {

enum class mouse
{
	left, right, middle,
};

enum class keyboard
{
	A = 65, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
	space     = 32,
	escape    = 256,
	enter     = 257,
	tab       = 258,
	backspace = 259,
	right     = 262,
	left      = 263,
	down      = 264,
	up        = 265,

	num_0 = 48, num_1, num_2, num_3, num_4, num_5, num_6, num_7, num_8, num_9,
};

enum modifier // not an enum class because they can be | together
{
	shift = 1 << 0,
	ctrl  = 1 << 1,
	alt   = 1 << 2
};

// KEYBOARD EVENTS 
struct keyboard_pressed_event {
	keyboard key;
	int scancode;
	int mods;
};

struct keyboard_released_event {
	keyboard key;
	int scancode;
	int mods;
};

struct keyboard_held_event {
	keyboard key;
	int scancode;
	int mods;
};

struct keyboard_typed_event {
	std::uint32_t key;
};

// MOUSE EVENTS
struct mouse_pressed_event {
	mouse button;
	int mods;
};

struct mouse_released_event {
	mouse button;
	int mods;
};

struct mouse_moved_event {
	glm::ivec2 pos;
};

struct mouse_scrolled_event {
	glm::ivec2 offset;
};

// WINDOW EVENTS
struct window_resize_event {
	int width;
	int height;
};

struct window_closed_event {};
struct window_got_focus_event {};
struct window_lost_focus_event {};
struct window_maximise_event {};
struct window_minimise_event {};

class event
{
	using event_variant = std::variant<
		keyboard_pressed_event,
		keyboard_released_event,
		keyboard_held_event,
		keyboard_typed_event,
		mouse_pressed_event,
		mouse_released_event,
		mouse_moved_event,
		mouse_scrolled_event,
		window_resize_event,
		window_closed_event,
		window_got_focus_event,
		window_lost_focus_event,
		window_maximise_event,
		window_minimise_event
	>;

    event_variant d_event;

public:
	template <typename T>
	explicit event(T&& t) : d_event{t} {}

	template <typename T>
	auto is() const noexcept -> bool { return std::holds_alternative<T>(d_event); }

	template <typename T>
	auto as() const -> const T& { return std::get<T>(d_event); }

	template <typename T>
	auto get_if() const -> const T* { return std::get_if<T>(&d_event); }

	auto is_keyboard_event() const -> bool;
	auto is_mount_event() const -> bool;
	auto is_window_event() const -> bool;
};

inline auto event::is_keyboard_event() const -> bool
{
	return is<keyboard_held_event>()
		|| is<keyboard_typed_event>()
		|| is<keyboard_released_event>()
		|| is<keyboard_typed_event>();
}

inline auto event::is_mount_event() const -> bool
{
	return is<mouse_moved_event>()
		|| is<mouse_pressed_event>()
		|| is<mouse_released_event>()
		|| is<mouse_scrolled_event>();
}

inline auto event::is_window_event() const -> bool
{
	return is<window_resize_event>()
		|| is<window_closed_event>()
		|| is<window_got_focus_event>()
		|| is<window_lost_focus_event>()
		|| is<window_maximise_event>()
		|| is<window_minimise_event>();
}

}