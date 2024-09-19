
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/_graph.hpp"

namespace mess2_algorithms
{
    Graph::Graph() {};

    std::vector<Vertex> Graph::get_vertices_() const {
        return vertices_;
    }

    std::vector<Edge> Graph::get_edges_() const {
        return edges_;
    }

    void Graph::print_vertices() const {
        std::cout << "new row:" << std::endl;
        for (int64_t iter = 0; iter < static_cast<int64_t>(vertices_.size()); ++iter) {
            const auto& vertex = vertices_[iter];
            std::cout << iter << ": " << vertex.get_x_() << ", " << vertex.get_y_() << ", " << vertex.get_theta_() << std::endl;
        }
    }

    void Graph::print_edges() const {
        for (int64_t iter = 0; iter < static_cast<int64_t>(edges_.size()); ++iter) {
            const auto& edge = edges_[iter];
            std::cout << iter << ": " << edge.get_index_parent_() << " to " << edge.get_index_child_() << std::endl;
        }
    }

    void Graph::fill_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        vertices_ = generate_vertices(x_mesh, y_mesh);
        edges_ = generate_edges(x_mesh, y_mesh, vertices_);
    }

    Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        auto _graph = Graph();
        _graph.fill_graph(x_mesh, y_mesh);
        return _graph;
    }

} // namespace mess2_algorithms
