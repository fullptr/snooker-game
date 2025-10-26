cmake_minimum_required(VERSION 3.30)

add_subdirectory(core)

find_package(glm CONFIG REQUIRED)

add_executable(game
               game.m.cpp
               simulation.cpp)

target_include_directories(game PUBLIC .)

target_link_libraries(game PRIVATE
    core
    glm::glm
)