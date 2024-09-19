#ifndef MESS2_ALGORITHM_PLUGINS_ADJACENCY_HPP
#define MESS2_ALGORITHM_PLUGINS_ADJACENCY_HPP

#include "mess2_algorithm_plugins/common.hpp"

#include "mess2_algorithm_plugins/graph/_graph.hpp"

namespace mess2_algorithms
{
    using adjacency = std::vector<std::vector<int64_t>>;

    class Adjacency
    {
    public:
        Adjacency();

        adjacency get_adjacency() const;
        std::vector<int64_t> get_adjacencies(const int64_t& index_parent);

        void print_adjacency() const;

        void fill_adjacency(const Graph& graph, const std::string& type);

    private:
        adjacency adjacency_;
    };

    Adjacency generate_adjacency(const Graph& graph, const std::string& type);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ADJACENCY_HPP
