#include "texture.hpp"

#include <glad/glad.h>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <cassert>
#include <print>

namespace snooker {

texture_dyn::texture_dyn(i32 width, i32 height)
{
    resize(width, height);
}

texture_dyn::~texture_dyn()
{
    glDeleteTextures(1, &d_texture);
}

auto texture_dyn::set_data(std::span<const glm::vec4> data) -> void
{
    assert(data.size() == d_width * d_height);
    bind();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, d_width, d_height, 0, GL_RGBA, GL_FLOAT, data.data());
}

auto texture_dyn::set_subdata(std::span<const glm::vec4> data, glm::ivec2 top_left, i32 width, i32 height) -> void
{
    assert(data.size() == width * height);
    glTextureSubImage2D(d_texture, 0, top_left.x, top_left.y, width, height, GL_RGBA, GL_FLOAT, data.data());
}

auto texture_dyn::bind() const -> void
{
    glBindTexture(GL_TEXTURE_2D, d_texture);
}

auto texture_dyn::resize(i32 width, i32 height) -> void
{
    if (d_texture) {
        glDeleteTextures(1, &d_texture);
    }

    d_width = width;
    d_height = height;

    glGenTextures(1, &d_texture);
    glBindTexture(GL_TEXTURE_2D, d_texture);
    glTextureParameteri(d_texture, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTextureParameteri(d_texture, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTextureParameteri(d_texture, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTextureParameteri(d_texture, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, d_width, d_height, 0, GL_RGBA, GL_FLOAT, nullptr);
}

texture_png::texture_png(const char* filename)
{
    i32 channels = 0;
    unsigned char *data = stbi_load(filename, &d_width, &d_height, &channels, 0);
    if (!data) {
        std::print("Failed to load image\n");
        std::exit(1);
    }
    
    glGenTextures(1, &d_texture);
    glBindTexture(GL_TEXTURE_2D, d_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, d_width, d_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    stbi_image_free(data);
}

texture_png::~texture_png()
{
    glDeleteTextures(1, &d_texture);
}

auto texture_png::bind() const -> void
{
    glBindTexture(GL_TEXTURE_2D, d_texture);
}

}