#include "window.hpp"
#include "utility.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <cstdint>
#include <print>

namespace sudoku {
namespace {

auto get_window_data(GLFWwindow* window) -> window_data&
{
    return *static_cast<window_data*>(glfwGetWindowUserPointer(window));
}

}

window::window(const char* name, int width, int height)
    : d_data({width, height, true, true, true})
{
    if (GLFW_TRUE != glfwInit()) {
        std::print("FATAL: Failed to initialise GLFW\n");
        std::exit(-1);
    }

    auto native_window = glfwCreateWindow(width, height, name, nullptr, nullptr);
    if (!native_window) {
        std::print("FATAL: Failed to create window\n");
        std::exit(-2);
    }

    d_data.native_window = native_window;

    glfwMakeContextCurrent(native_window);
    glfwSetWindowUserPointer(native_window, &d_data);
    glfwSwapInterval(1);

    // Initialise GLAD
    if (0 == gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::print("FATAL: Failed to initialise GLAD\n");
        std::exit(-3);
    }

    // Set OpenGL error callback
    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback([](GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message, const void*) {
        if (severity != GL_DEBUG_SEVERITY_NOTIFICATION) {
            std::print("{}, {}, {}, {}, {}\n", source, type, id, length, message);
        }
    }, nullptr);

    // Set GLFW callbacks
    glfwSetWindowSizeCallback(native_window, [](GLFWwindow* window, int width, int height)
    {
        glViewport(0, 0, width, height);
        auto& data = get_window_data(window);
        if (!data.focused) return;
        data.width = width;
        data.height = height;
        data.events.emplace_back(window_resize_event{width, height});
    });

    glfwSetWindowCloseCallback(native_window, [](GLFWwindow* window)
    {
        auto& data = get_window_data(window);
        if (!data.focused) return;
        data.running = false;
        data.events.emplace_back(window_closed_event{});
    });

    glfwSetKeyCallback(native_window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        auto& data = get_window_data(window);
        if (!data.focused) return;
        switch (action)
        {
            case GLFW_PRESS: {
                data.events.emplace_back(keyboard_pressed_event{keyboard{key}, scancode, mods});
            } break;
            case GLFW_RELEASE: {
                data.events.emplace_back(keyboard_released_event{keyboard{key}, scancode, mods});
            } break;
            case GLFW_REPEAT: {
                data.events.emplace_back(keyboard_held_event{keyboard{key}, scancode, mods});
            } break;
        }
    });

    glfwSetMouseButtonCallback(native_window, [](GLFWwindow* window, int button, int action, int mods) {
        auto& data = get_window_data(window);
        if (!data.focused) return;
        switch (action)
        {
            case GLFW_PRESS: {
                data.events.emplace_back(mouse_pressed_event{mouse{button}, mods});
            } break;
            case GLFW_RELEASE: {
                data.events.emplace_back(mouse_released_event{mouse{button}, mods});
            } break;
        }
    });

    glfwSetCursorPosCallback(native_window, [](GLFWwindow* window, double x_pos, double y_pos) {
        auto& data = get_window_data(window);
        if (!data.focused) return;
        data.events.emplace_back(mouse_moved_event{{x_pos, y_pos}});
    });

    glfwSetScrollCallback(native_window, [](GLFWwindow* window, double x_offset, double y_offset) {
        auto& data = get_window_data(window);
        if (!data.focused) return;
        data.events.emplace_back(mouse_scrolled_event{{x_offset, y_offset}});
    });

    glfwSetWindowFocusCallback(native_window, [](GLFWwindow* window, int focused) {
        auto& data = get_window_data(window);
        data.focused = focused;
        if (focused) {
            data.events.emplace_back(window_got_focus_event{});
        } else {
            data.events.emplace_back(window_lost_focus_event{});
        }
    });

    glfwSetWindowMaximizeCallback(native_window, [](GLFWwindow* window, int maximized) {
        auto& data = get_window_data(window);
        if (maximized) {
            data.events.emplace_back(window_maximise_event{});
        } else {
            data.events.emplace_back(window_minimise_event{});
        }
    });

    glfwSetCharCallback(native_window, [](GLFWwindow* window, std::uint32_t key) {
        auto& data = get_window_data(window);
        if (!data.focused) return;
        data.events.emplace_back(keyboard_typed_event{key});
    });
}

window::~window()
{
    glfwDestroyWindow(d_data.native_window);
    glfwTerminate();
}

auto window::begin_frame(glm::vec4 colour) -> void
{
    d_data.events.clear();
    glfwPollEvents();
    glClearColor(colour.r, colour.g, colour.b, colour.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

auto window::end_frame() -> void
{
    glfwSwapBuffers(d_data.native_window);
}

auto window::events() -> std::span<const event>
{
    return d_data.events;
}

auto window::mouse_pos() const -> glm::ivec2
{
    double x, y;
    glfwGetCursorPos(d_data.native_window, &x, &y);
    return {static_cast<i32>(x), static_cast<i32>(y)};
}

bool window::is_running() const
{
    return d_data.running;
}

auto window::width() const -> int
{
    return d_data.width; 
}

auto window::height() const -> int
{
    return d_data.height; 
}

auto window::native_handle() -> GLFWwindow*
{
    return d_data.native_window;
}

}