
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/low_level/adjacency.hpp"

namespace mess2_algorithms
{
    Adjacency::Adjacency() {}

    adjacency Adjacency::get_adjacency() const {
        return adjacency_;
    }

    void Adjacency::print_adjacencies() const {
        for (int64_t iter = 0; iter < static_cast<int64_t>(adjacency_.size()); ++iter) {
            std::cout << "node " << iter << " -> ";
            if (adjacency_[iter].empty()) {
                std::cout << "no adjacent nodes";
            } else {
                for (const auto& child : adjacency_[iter]) {
                    std::cout << child << " ";
                }
            }
            std::cout << std::endl;
        }
    }

    void Adjacency::fill_adjacency(const Graph& graph)
    {
        const auto edges = graph.get_edges_();
        for (const auto& edge : edges) {
            const auto index_parent = edge.get_index_parent_();
            const auto index_child = edge.get_index_child_();

            adjacency_[index_parent].emplace_back(index_child);
        }
    }

    Adjacency generate_adjacency(const Graph& graph)
    {
        auto _adjacency = Adjacency();
        _adjacency.fill_adjacency(graph);
        return _adjacency;
    }

} // namespace mess2_algorithms
