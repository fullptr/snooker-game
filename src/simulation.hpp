#pragma once
#include <vector>

#include "table.hpp"

namespace snooker {

void step_simulation(std::vector<collider>& colliders, float dt);

}