#pragma once
#include <glm/glm.hpp>

namespace sudoku {

struct camera
{
    int screen_width;      // screen space
    int screen_height;     // screen space
};

// Returns the width and height as a glm::vec2
inline auto dimensions(const camera& c) -> glm::vec2
{
    return {c.screen_width, c.screen_height};
}

}