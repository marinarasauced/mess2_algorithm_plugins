
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/graph.hpp"

namespace mess2_algorithms
{
    Graph::Graph() {};

    std::vector<Vertex> Graph::get_vertices() const {
        return vertices_;
    }

    Vertex Graph::get_vertex(const int64_t& index_vertex) {
        return vertices_[index_vertex];
    }

    std::vector<Edge> Graph::get_edges() const {
        return edges_;
    }

    Edge Graph::get_edge(const int64_t& index_edge) {
        return edges_[index_edge];
    }

    std::unordered_map<std::pair<double, double>, std::vector<int64_t>, hash_vertices> Graph::get_vertices_map() const {
        return vertices_map_;
    }

    void Graph::print_vertices() const {
        for (int64_t iter = 0; iter < static_cast<int64_t>(vertices_.size()); ++iter) {
            const auto& vertex = vertices_[iter];
            std::cout << iter << ": " << vertex.get_x() << ", " << vertex.get_y() << ", " << vertex.get_theta() << std::endl;
        }
    }

    void Graph::print_edges() const {
        for (int64_t iter = 0; iter < static_cast<int64_t>(edges_.size()); ++iter) {
            const auto& edge = edges_[iter];
            const auto& vertex_parent = vertices_[edge.get_index_parent()];
            const auto& vertex_child = vertices_[edge.get_index_child()];

            std::cout << iter << ": " 
                  << edge.get_index_parent() << " (" 
                  << vertex_parent.get_x() << ", " 
                  << vertex_parent.get_y() << ", " 
                  << vertex_parent.get_theta() << ")"
                  << " to " 
                  << edge.get_index_child() << " (" 
                  << vertex_child.get_x() << ", " 
                  << vertex_child.get_y() << ", " 
                  << vertex_child.get_theta() << ")" 
                  << std::endl;
        }
    }

    void Graph::fill_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        vertices_ = generate_vertices(x_mesh, y_mesh);
        
        vertices_map_.reserve(x_mesh.n_elem);
        for (int64_t iter = 0; iter < static_cast<int64_t>(vertices_.size()); ++iter) {
            const auto vertex = vertices_[iter];
            vertices_map_[{vertex.get_x(), vertex.get_y()}].push_back(iter);
        }
        
        edges_ = generate_edges(x_mesh, y_mesh, vertices_, vertices_map_);
    }

    Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        auto _graph = Graph();
        _graph.fill_graph(x_mesh, y_mesh);
        return _graph;
    }

} // namespace mess2_algorithms
