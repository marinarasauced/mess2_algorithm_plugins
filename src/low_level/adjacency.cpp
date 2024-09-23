
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

    void Adjacency::print_adjacency(const Graph& graph)
    {
        const auto edges = graph.get_edges();
        const auto vertices = graph.get_vertices();

        if (vertices.size() != adjacency_.size()) {
            throw std::runtime_error("adjacency and vertices must match size");
        }

        const auto n_vertices = static_cast<int64_t>(vertices.size());
        for (int64_t iter = 0; iter < n_vertices; ++iter) {
            const auto vertex_parent = vertices[iter];
            const auto x_parent = vertex_parent.get_x();
            const auto y_parent = vertex_parent.get_y();
            const auto theta_parent = vertex_parent.get_theta();

            const auto adjacencies = adjacency_[iter];

            std::cout << adjacencies.size() << " (" << x_parent << ", " << y_parent << ", " << theta_parent << ") -> "; 

            for (int64_t jter = 0; jter < static_cast<int64_t>(adjacencies.size()); ++jter) {
                const auto index_edge = adjacencies[jter];
                const auto edge = edges[index_edge];
                const auto index_parent = edge.get_index_parent();
                
                if (iter != index_parent) {
                    throw std::runtime_error("vertex index and edge parent do not match");
                }

                const auto index_child = edge.get_index_child();
                const auto vertex_child = vertices[index_child];
                const auto x_child = vertex_child.get_x();
                const auto y_child = vertex_child.get_y();
                const auto theta_child = vertex_child.get_theta();

                std::cout << "(" << x_child << ", " << y_child << ", " << theta_child << "), ";
            }
            std::cout << std::endl;
        }
    }

    void Adjacency::fill_adjacency(const Graph& graph, const std::string& type_fill)
    {
        const bool is_wait = (type_fill=="wait");
        const bool is_rotate = (type_fill=="rotate");
        const bool is_translate = (type_fill=="translate");

        const auto vertices = graph.get_vertices();
        adjacency_.resize(vertices.size());

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
