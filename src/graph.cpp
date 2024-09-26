
#include "mess2_algorithm_plugins/graph.hpp"

namespace mess2_algorithms
{
    Graph::Graph(const arma::mat& x_mesh, const arma::mat& y_mesh, const int64_t& resolution, const bool& use_diagonals)
    {
        (void) generate_vertices(x_mesh, y_mesh, use_diagonals);
        (void) generate_map(resolution);
        (void) generate_edges(x_mesh, y_mesh, use_diagonals);
        (void) generate_adjacencies();
    };

    void Graph::generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh, const bool& use_diagonals)
    {
        if (x_mesh.n_rows != y_mesh.n_rows || x_mesh.n_cols != y_mesh.n_cols) {
            throw std::runtime_error("x_mesh and y_mesh must have the same dimensions");
        }

        const int64_t n_rows = x_mesh.n_rows;
        const int64_t n_cols = x_mesh.n_cols;

        int64_t c1;
        double c2;
        if (use_diagonals) {
            c1 = 8;
            c2 = 45;
        } else if (!use_diagonals) {
            c1 = 4;
            c2 = 90;
        }

        for (int64_t iter = 0; iter < c1; ++iter) {
            for (int64_t jter = 0; jter < n_rows; ++jter) {
                for (int64_t kter = 0; kter < n_cols; ++ kter) {
                    vertices_.emplace_back(graph_vertex{x_mesh(jter, kter), y_mesh(jter, kter), c2 * iter});
                }
            }
        }

        n_vertices = static_cast<int64_t>(vertices_.size());
    }

    void Graph::generate_map(const int64_t& resolution)
    {
        const int64_t n_elem = std::pow(resolution, 2);
        map_.reserve(n_elem);

        for (int64_t iter = 0; iter < n_vertices; ++iter) {
            const auto vertex = vertices_[iter];
            map_[{vertex.x, vertex.y}].push_back(iter);
        }
    }

    void Graph::generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh, const bool& use_diagonals)
    {
        if (x_mesh.n_rows != y_mesh.n_rows || x_mesh.n_cols != y_mesh.n_cols) {
            throw std::runtime_error("x_mesh and y_mesh must have the same dimensions");
        } else if (map_.size() == 0) {
            throw std::runtime_error("graph map cannot be empty");
        }

        const int64_t n_rows = x_mesh.n_rows;
        const int64_t n_cols = x_mesh.n_cols;

        std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> pairs;
        for (int64_t iter = 0; iter < n_rows; ++iter) {
            for (int64_t jter = 0; jter < n_cols; ++jter) {

                // wait
                std::pair<double, double> pair_nw = {x_mesh(iter, jter), y_mesh(iter, jter)};
                pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_nw));

                // translate horizontally in graph
                if (jter + 1 < n_cols) {
                    std::pair<double, double> pair_ne = {x_mesh(iter, jter + 1), y_mesh(iter, jter + 1)};
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_ne));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_ne, pair_nw));
                }

                // translate vertically in graph
                if (iter + 1 < n_rows) {
                    std::pair<double, double> pair_sw = {x_mesh(iter + 1, jter), y_mesh(iter + 1, jter)};
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_sw));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_sw, pair_nw));
                }

                // translate diagonally in graph
                if (iter + 1 < n_rows && jter + 1 < n_cols && use_diagonals) {
                    std::pair<double, double> pair_ne = {x_mesh(iter, jter + 1), y_mesh(iter, jter + 1)};
                    std::pair<double, double> pair_sw = {x_mesh(iter + 1, jter), y_mesh(iter + 1, jter)};
                    std::pair<double, double> pair_se = {x_mesh(iter + 1, jter + 1), y_mesh(iter + 1, jter + 1)};
                    
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_se));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_se, pair_nw));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_ne, pair_sw));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_sw, pair_ne));
                }
            }
        }

        for (const auto& pair : pairs) {
            auto iterator_1 = map_.find(pair.first);
            auto iterator_2 = map_.find(pair.second);

            if (iterator_1 != map_.end() && iterator_2 != map_.end()) {
                const std::vector<int64_t> indices_1 = iterator_1->second;
                const std::vector<int64_t> indices_2 = iterator_2->second;

                for (const auto& index_1 : indices_1) {
                    for (const auto& index_2 : indices_2) {
                        const auto vertex_1 = vertices_[index_1];
                        const auto vertex_2 = vertices_[index_2];

                        bool is_same_x = (vertex_1.x == vertex_2.x);
                        bool is_same_y = (vertex_1.y == vertex_2.y);
                        bool is_same_theta = (vertex_1.theta == vertex_2.theta);

                        if (is_same_x && is_same_y && is_same_theta) {
                            edges_.emplace_back(graph_edge{index_1, index_2, "wait"});
                        } else if (is_same_x && is_same_y && !is_same_theta) {
                            edges_.emplace_back(graph_edge{index_1, index_2, "rotate"});
                        } else if (is_same_theta) {
                            auto theta_true = (180.0 / M_PI) * std::atan2(vertex_2.y - vertex_1.y, vertex_2.x - vertex_1.x);
                            if (theta_true < 0) {
                                theta_true += 360;
                            }
                            if (theta_true == vertex_1.theta) {
                                edges_.emplace_back(graph_edge{index_1, index_2, "translate"});
                            }
                        }
                    }
                }
            }
        }

        n_edges = static_cast<int64_t>(edges_.size());
    }

    void Graph::generate_adjacencies()
    {
        if (n_vertices == 0 || n_edges == 0) {
            throw std::runtime_error("vertices and edges cannot have zero elements");
        }

        adjacencies_wait_.resize(n_vertices);
        adjacencies_rotate_.resize(n_vertices);
        adjacencies_translate_.resize(n_vertices);

        for (int64_t iter = 0; iter < n_edges; ++iter) {
            const auto edge = edges_[iter];

            adjacencies_wait_[edge.index_parent].push_back(iter);
            if (edge.type == "translate") {
                adjacencies_rotate_[edge.index_parent].push_back(iter);
            }
            adjacencies_translate_[edge.index_parent].push_back(iter);
        }
    }

    graph_vertex Graph::lookup_vertex(const int64_t& index_vertex) const
    {
        if (index_vertex > n_vertices) {
            throw std::out_of_range("vertex index cannot exceed number of vertices");
        }
        return vertices_[index_vertex];
    }

    std::vector<int64_t> Graph::lookup_map(const double&x, const double& y) const
    {
        auto iterator = map_.find({x, y});
        if (iterator == map_.end()) {
            throw std::runtime_error("iterator cannot equal end of map");
        }

        const std::vector<int64_t> map = iterator->second;
        return map;
    }

    graph_edge Graph::lookup_edge(const int64_t& index_edge) const
    {
        if (index_edge > n_edges) {
            throw std::out_of_range("edge index cannot exceed number of edges");
        }
        return edges_[index_edge];
    }

    std::vector<int64_t> Graph::lookup_adjacencies(const int64_t& index_vertex, const std::string& type) const
    {
        if (index_vertex > n_vertices) {
            throw std::out_of_range("vertex index cannot exceed number of vertices");
        }
        
        if (type == "wait") {
            return adjacencies_wait_[index_vertex];
        } else if (type == "rotate") {
            return adjacencies_rotate_[index_vertex];
        } else if (type == "translate") {
            return adjacencies_translate_[index_vertex];
        } else {
            throw std::runtime_error("transition type must be wait, rotate, or translate");
        }

        return std::vector<int64_t>();
    }

    int64_t Graph::lookup_index_edge(const int64_t& index_parent, const int64_t& index_child) const
    {
        for (int64_t iter = 0; iter < n_edges; ++iter) {
            const auto edge = lookup_edge(iter);
            if (edge.index_parent == index_parent && edge.index_child == index_child) {
                return iter;
            }
        }

        return INT64_MAX; // if no edge exists
    }

    int64_t Graph::lookup_index_vertex(const double& x, const double& y, const double& theta) const
    {
        const auto indices_vertices = lookup_map(x, y);
        for (const auto& index_vertex : indices_vertices) {
            const auto vertex = lookup_vertex(index_vertex);
            if (vertex.theta == theta) {
                return index_vertex;
            }
        }

        return INT64_MAX; // if no vertex exists
    }

    void Graph::info()
    {
        throw std::runtime_error("graph info not implemented");
    }

    std::tuple<arma::mat, arma::mat> generate_mesh(const double& x_min, const double& x_max, const double& y_min, const double& y_max, const int64_t& resolution)
    {
        auto x_vector = arma::linspace(x_min, x_max, resolution);
        auto y_vector = arma::linspace(y_min, y_max, resolution);

        int n_rows = y_vector.n_elem;
        int n_cols = x_vector.n_elem;
        arma::mat x_mesh(n_rows, n_cols);
        arma::mat y_mesh(n_rows, n_cols);
        for (int iter = 0; iter < n_rows; ++iter)
        {
            x_mesh.row(iter) = x_vector.t();
            y_mesh.row(iter).fill(y_vector(iter));
        }
        return std::make_tuple(x_mesh, y_mesh);
    }

} // namespace mess2_algorithms
