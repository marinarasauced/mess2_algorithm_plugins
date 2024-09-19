
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/low_level/adjacency.hpp"

namespace mess2_algorithms
{
    Adjacency::Adjacency() {}

    adjacency Adjacency::get_adjacency() const {
        return adjacency_;
    }

    std::vector<int64_t> Adjacency::get_adjacencies(const int64_t& index_parent) {
        return adjacency_[index_parent];
    }

    void Adjacency::print_adjacency() const {
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

    void Adjacency::fill_adjacency(const Graph& graph, const std::string& type_fill)
    {
        const bool is_wait = (type_fill=="wait");
        const bool is_rotate = (type_fill=="rotate");
        const bool is_translate = (type_fill=="translate");

        const auto edges = graph.get_edges();
        for (int64_t iter = 0; iter < static_cast<int64_t>(edges.size()); ++iter) {
            const auto edge = edges[iter];
            const auto index_parent = edge.get_index_parent();
            const auto type = edge.get_type();

            if (is_wait) {
                // after wait : wait, rotate, translate
                adjacency_[index_parent].emplace_back(iter);
            } else if (is_rotate && type=="translate") {
                // after rotate : translate
                adjacency_[index_parent].emplace_back(iter);
            } else if (is_translate) {
                // after translate : wait, rotate, translate
                adjacency_[index_parent].emplace_back(iter);
            }
        }
        // for (const auto& edge : edges) {
        //     const auto index_parent = edge.get_index_parent_();
        //     const auto index_child = edge.get_index_child_();

        //     adjacency_[index_parent].emplace_back(index_child);
        // }
    }

    Adjacency generate_adjacency(const Graph& graph, const std::string& type)
    {
        auto _adjacency = Adjacency();
        _adjacency.fill_adjacency(graph, type);
        return _adjacency;
    }

} // namespace mess2_algorithms
