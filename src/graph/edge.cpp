
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/edge.hpp"

namespace mess2_algorithms
{
    Edge::Edge(const int64_t& index_parent, const int64_t& index_child, const std::string& type)
    {
        index_parent_ = index_parent;
        index_child_ = index_child;
        type_ = type;
    }

    int64_t Edge::get_index_parent() const {
        return index_parent_;
    }

    int64_t Edge::get_index_child() const {
        return index_child_;
    }

    std::string Edge::get_type() const {
        return type_;
    }

    std::vector<Edge> generate_edges(const arma::mat& x_mesh, const arma::mat& y_mesh, const std::vector<Vertex>& vertices, std::unordered_map<std::pair<double, double>, std::vector<int64_t>, hash_vertices> map_vertices)
    {
        std::vector<Edge> edges;

        int64_t n_rows = x_mesh.n_rows;
        int64_t n_cols = y_mesh.n_cols;

        std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> pairs;
        for (int64_t iter = 0; iter < n_rows; ++iter) {
            for (int64_t jter = 0; jter < n_cols; ++jter) {
                std::pair<double, double> pair_nw = {x_mesh(iter, jter), y_mesh(iter, jter)};
                pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_nw));

                if (jter + 1 < n_cols) {
                    std::pair<double, double> pair_ne = {x_mesh(iter, jter + 1), y_mesh(iter, jter + 1)};
                    
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_ne));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_ne, pair_nw));
                }

                if (iter + 1 < n_rows) {
                    std::pair<double, double> pair_sw = {x_mesh(iter + 1, jter), y_mesh(iter + 1, jter)};
                    
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_nw, pair_sw));
                    pairs.push_back(std::pair<std::pair<double, double>, std::pair<double, double>>(pair_sw, pair_nw));
                }

                if (iter + 1 < n_rows && jter + 1 < n_cols) {
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
            const auto vertex_1 = pair.first;
            const auto vertex_2 = pair.second;
            const auto iter_1 = map_vertices.find(vertex_1);
            const auto iter_2 = map_vertices.find(vertex_2);

            if (iter_1 != map_vertices.end() && iter_2 != map_vertices.end()) {
                const std::vector<int64_t> indices_1 = iter_1->second;
                const std::vector<int64_t> indices_2 = iter_2->second;

                for (const auto index_1 : indices_1) {
                    for (const auto index_2 : indices_2) {
                        const auto vertex_1 = vertices[index_1];
                        const auto vertex_2 = vertices[index_2];

                        const auto x_1 = vertex_1.get_x();
                        const auto x_2 = vertex_2.get_x();
                        const auto y_1 = vertex_1.get_y();
                        const auto y_2 = vertex_2.get_y();
                        const auto theta_1 = vertex_1.get_theta();
                        const auto theta_2 = vertex_2.get_theta();

                        bool is_same_x = (x_1 == x_2);
                        bool is_same_y = (y_1 == y_2);
                        bool is_same_theta = (theta_1 == theta_2);

                        if (is_same_x && is_same_y && is_same_theta) {
                            edges.emplace_back(Edge(index_1, index_2, "wait"));
                        } else if (is_same_x && is_same_y && !is_same_theta) {
                            edges.emplace_back(Edge(index_1, index_2, "rotate"));
                        } else if (is_same_theta) {
                            double theta_true = (180.0 / M_PI) * std::atan2(
                                y_2 - y_1,
                                x_2 - x_1
                            );
                            // std::cout << theta_true << std::endl;
                            if (theta_true < 0) {
                                theta_true += 360;
                            }

                            if (theta_true == theta_1) {
                                edges.emplace_back(Edge(index_1, index_2, "translate"));
                            }
                        }
                    }
                }
            }
        }

        // std::unordered_map<std::pair<double, double>, size_t, pair_hash> vertex_map;
        // for (size_t i = 0; i < vertices.size(); ++i) {
        //     const auto& vertex = vertices[i];
        //     vertex_map[{vertex.get_x(), vertex.get_y()}] = i;
        // }

        // for (const auto& pair : pairs) {
        //     const auto& vertex_1 = pair.first;
        //     const auto& vertex_2 = pair.second;

        //     const auto iter_1 = map_vertices.find(vertex_1);
        //     const auto iter_2 = map_vertices.find(vertex_2);

        //      if (iter_1 != vertex_map.end() && iter_2 != vertex_map.end())


        //     const parent_iter = vertex_map.find(vertex_1)->second;
        //     auto child_iter = vertex_map.find(vertex_2);

        //     if (parent_iter != vertex_map.end() && child_iter != vertex_map.end()) {
        //         size_t iter = parent_iter->second;
        //         size_t jter = child_iter->second;

        //         const auto& vertex_parent = vertices[iter];
        //         const auto& vertex_child = vertices[jter];

        //         auto vertex_parent_x_ = vertex_parent.get_x();
        //         auto vertex_parent_y_ = vertex_parent.get_y();
        //         auto vertex_parent_theta_ = vertex_parent.get_theta();
        //         auto vertex_child_x_ = vertex_child.get_x();
        //         auto vertex_child_y_ = vertex_child.get_y();
        //         auto vertex_child_theta_ = vertex_child.get_theta();

        //         bool is_same_x = (vertex_child_x_ == vertex_parent_x_);
        //         bool is_same_y = (vertex_child_y_ == vertex_parent_y_);
        //         bool is_same_theta = (vertex_child_theta_ == vertex_parent_theta_);

        //         if (is_same_x && is_same_y && is_same_theta) {
        //             edges.emplace_back(Edge(iter, jter, "wait"));
        //         } else if (is_same_x && is_same_y && !is_same_theta) {
        //             edges.emplace_back(Edge(iter, jter, "rotate"));
        //         } else if (is_same_theta) {
        //             double theta_true = (180.0 / M_PI) * std::atan2(
        //                 vertex_child_y_ - vertex_parent_y_,
        //                 vertex_child_x_ - vertex_parent_x_
        //             );
        //             if (theta_true < 0) {
        //                 theta_true += 360;
        //             }

        //             if (theta_true == vertex_parent_theta_) {
        //                 edges.emplace_back(Edge(iter, jter, "translate"));
        //             }
        //         }
        //     }
        // }

        return edges;
    }

    std::vector<int64_t> generate_edges_key(const int64_t& n_edges) {
        std::vector<int64_t> key_edges;
        key_edges.resize(n_edges);
        return key_edges;
    }

    std::vector<int64_t> reset_edges_key(std::vector<int64_t>& key_edges) {
        for (int64_t iter = 0; iter < static_cast<int64_t>(key_edges.size()); ++iter) {
            key_edges[iter] = 0;
        }
        return key_edges;
    }


} // namespace mess2_algorithms
