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

        void print_adjacencies() const;

        void fill_adjacency(const Graph& graph);

    private:
        adjacency adjacency_;
    };

    Adjacency generate_adjacency(const Graph& graph);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ADJACENCY_HPP
