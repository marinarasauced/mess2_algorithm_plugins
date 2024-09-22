#ifndef MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
#define MESS2_ALGORITHM_PLUGINS_GRAPH_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/edge.hpp"
#include "mess2_algorithm_plugins/graph/vertex.hpp"

namespace mess2_algorithms
{
    class Graph
    {
    public:
        Graph();

        std::vector<Vertex> get_vertices() const;
        Vertex get_vertex(const int64_t& index_vertex);
        std::vector<Edge> get_edges() const;
        Edge get_edge(const int64_t& index_edge);

        void print_vertices() const;
        void print_edges() const;

        void fill_graph(const arma::mat& x_mesh, const arma::mat& y_mesh);

    private:
    
        std::vector<Vertex> vertices_;
        std::vector<Edge> edges_;
    };

    Graph generate_graph(const arma::mat& x_mesh, const arma::mat& y_mesh);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_GRAPH_HPP
