#pragma once
#include <vector>
#include <unordered_map>
#include <cstdint>

#include "utility.hpp"

namespace snooker {

// Stores elements in a packed vector with each element assigned a unique ID.
// Elements can be accessed by ID and they can be looped over.
// The constraints that are maintained is that the IDs are stable and the 
// underlying vector is packed contiguously. The ordering of elements can change
// when elements are removed.
template <typename T>
class id_vector
{
    std::vector<T>           d_data;
    std::vector<std::size_t> d_data_id;
    std::unordered_map<std::size_t, std::size_t> d_id_to_index;
    std::size_t d_next = 0;

public:
    auto is_valid(std::size_t id) const -> bool
    {
        return d_id_to_index.contains(id);
    }
    auto insert(const T& c) -> std::size_t
    {
        const auto id = d_next++;
        const auto index = d_data.size();
        d_id_to_index[id] = index;
        d_data.emplace_back(c);
        d_data_id.emplace_back(id);
        return id;
    }
    auto erase(std::size_t id) -> void
    {
        assert_that(is_valid(id), std::format("invalid id {}\n", id));
        const auto index = d_id_to_index[id];

        // swap the id to delete to the end
        std::swap(d_data[index], d_data.back());
        std::swap(d_data_id[index], d_data_id.back());

        // update the map for the element swapped in
        d_id_to_index[d_data_id[index]] = index;
        d_id_to_index.erase(id);

        // remove the element we want to remove
        d_data.pop_back();
        d_data_id.pop_back();
    }
    auto data() -> std::vector<T>&
    {
        return d_data;
    }
    auto get(std::size_t id) -> T&
    {
        assert_that(is_valid(id), std::format("invalid id {}\n", id));
        return d_data[d_id_to_index[id]];
    }
    auto get(std::size_t id) const -> const T&
    {
        assert_that(is_valid(id), std::format("invalid id {}\n", id));
        return d_data[d_id_to_index.at(id)];
    }
};

}