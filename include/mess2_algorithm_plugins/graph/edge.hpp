#ifndef MESS2_ALGORITHM_PLUGINS_EDGE_HPP
#define MESS2_ALGORITHM_PLUGINS_EDGE_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/vertex.hpp"

namespace mess2_algorithms
{
    class Edge
    {
    public:
        Edge(const int64_t& index_parent, const int64_t& index_child, const std::string& type);

        int64_t get_index_parent() const;
        int64_t get_index_child() const;
        std::string get_type() const;

    private:
        int64_t index_parent_;
        int64_t index_child_;
        std::string type_;
    };

    std::vector<Edge> generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh, const std::vector<Vertex>& vertices, std::unordered_map<std::pair<double, double>, std::vector<int64_t>, hash_vertices> vertices_map);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_EDGE_HPP
