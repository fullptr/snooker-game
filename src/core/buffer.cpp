#include "buffer.hpp"

#include <glad/glad.h>

namespace snooker {

void vertex_buffer::set_data(std::size_t size, const void* data) const
{
    glNamedBufferData(d_vbo, size, data, GL_STATIC_DRAW);
}

vertex_buffer::vertex_buffer()
    : d_vbo{0}
{
    glCreateBuffers(1, &d_vbo);
}

vertex_buffer::~vertex_buffer()
{
    glDeleteBuffers(1, &d_vbo);
}

}