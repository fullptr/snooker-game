#pragma once
#include "utility.hpp"
#include "texture.hpp"

#include <glm/glm.hpp>

#include <memory>
#include <unordered_map>
#include <string_view>

namespace sudoku {
    
struct character
{
    glm::ivec2 position;
    glm::ivec2 size;
    glm::ivec2 bearing;
    i32        advance;
};

struct font_atlas
{
    std::unique_ptr<texture_png>        texture;
    std::unordered_map<char, character> chars;
    character                           missing_char;
    i32                                 height; // height of an "a", used for centring

    auto get_character(char c) const -> const character&;
    auto length_of(std::string_view message) const -> i32;
};

}