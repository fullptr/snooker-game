#pragma once
#include <cstdint>
#include <cstddef>
#include <span>

namespace sudoku {

class vertex_buffer
{
    std::uint32_t d_vbo;

    void set_data(std::size_t size, const void* data) const;
    
    vertex_buffer(const vertex_buffer&) = delete;
    vertex_buffer& operator=(const vertex_buffer&) = delete;

public:
    vertex_buffer();
    ~vertex_buffer();

    template <typename T> 
    void bind(std::span<const T> data) const { 
        T::set_buffer_attributes(d_vbo);
        set_data(data.size_bytes(), data.data());
    }
};

}