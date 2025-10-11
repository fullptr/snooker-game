cmake_minimum_required(VERSION 3.30)

find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)

add_library(core STATIC
    buffer.cpp
    renderer.cpp
    window.cpp
    shader.cpp
    texture.cpp
    font.cpp
    utility.cpp
    input.cpp
    ui.cpp
)

target_include_directories(core PUBLIC .)

target_link_libraries(core PRIVATE
    glfw
    glad::glad
    glm::glm
)