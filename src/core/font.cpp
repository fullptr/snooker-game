#include "font.hpp"

#include <ranges>

namespace snooker {

auto font_atlas::get_character(char c) const -> const character&
{
    if (auto it = chars.find(c); it != chars.end()) {
        return it->second;
    }
    return missing_char;
}

auto font_atlas::length_of(std::string_view message) const -> i32
{
    if (message.empty()) {
        return 0;
    }
    i32 length = 0;
    for (char c : message | std::views::drop(1)) {
        length += get_character(c).advance;
    }
    length += get_character(message.back()).size.x;
    return length;
}

}