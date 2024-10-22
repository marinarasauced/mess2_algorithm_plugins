
#include "mess2_algorithm_plugins/graph.hpp"

namespace mess2_algorithms
{
    void Graph::compute_keys_points_vertices()
    {
        keys.clear();
        points.clear();
        vertices.clear();

        int n_heading = 4;
        double dheading = 90.0;
        if (use_diagonals_in_plane) {
            n_heading = 8;
            dheading = 45.0;
        }

        for (auto i = 0; i < n_i; ++i) {
            for (auto j = 0; j < n_j; ++j) {
                for (auto k = 0; k < n_k; ++k) {
                    auto key = std::make_shared<Key3D>();
                    key->index_key = static_cast<int>(keys.size());
                    key->i = i;
                    key->j = j;
                    key->k = k;
                    keys.push_back(key);

                    auto point = std::make_shared<Point3D>();
                    point->index_point = static_cast<int>(points.size());
                    point->key = keys.back();
                    point->x = values_x[i];
                    point->y = values_y[j];
                    point->z = values_z[k];
                    point->value_threat = values_threat[i][j][k];
                    point->value_obstacle = values_obstacles[i][j][k];
                    points.push_back(point);

                    for (auto h = 0; h < n_heading; ++h) {
                        auto vertex = std::make_shared<Vertex>();
                        vertex->index_vertex = static_cast<int>(vertices.size());
                        vertex->point = points.back();
                        vertex->heading = h * dheading;
                        vertices.push_back(vertex);

                        vertices_by_index_point[point->index_point].push_back(vertices.back());
                    }
                }
            }
        }

        n_keys = static_cast<int>(keys.size());
        n_points = static_cast<int>(points.size());
        n_vertices = static_cast<int>(vertices.size());
    }


    void Graph::compute_edges()
    {
        std::vector<std::pair<std::shared_ptr<Point3D>, std::shared_ptr<Point3D>>> pairs;
        for (auto i = 0; i < n_i; ++i) {
            for (auto j = 0; j < n_j; ++j) {
                for (auto k = 0; k < n_k; ++k) {
                    if (!are_indices_valid(i, j, k)) {
                        continue;
                    }

                    int index_nw;
                    int index_sw;
                    int index_se;
                    int index_ne;
                    int index_up;

                    std::shared_ptr<Point3D> point_nw;
                    std::shared_ptr<Point3D> point_sw;
                    std::shared_ptr<Point3D> point_se;
                    std::shared_ptr<Point3D> point_ne;
                    std::shared_ptr<Point3D> point_up;

                    index_nw = find_index_point(i, j, k);
                    point_nw = points[index_nw];
                    pairs.push_back({point_nw, point_nw}); // allows waiting and rotating in plane

                    if (are_indices_valid(i + 1, j, k)) {
                        index_sw = find_index_point(i + 1, j, k);
                        point_sw = points[index_sw];
                        pairs.push_back({point_nw, point_sw}); // allows translating non-diagonally in plane
                        pairs.push_back({point_sw, point_nw}); // allows translating non-diagonally in plane
                    }

                    if (are_indices_valid(i, j + 1, k)) {
                        index_ne = find_index_point(i, j + 1, k);
                        point_ne = points[index_ne];
                        pairs.push_back({point_nw, point_ne}); // allows translating non-diagonally in plane
                        pairs.push_back({point_ne, point_nw}); // allows translating non-diagonally in plane
                    }

                    if (are_indices_valid(i + 1, j + 1, k) && use_diagonals_in_plane) {
                        index_se = find_index_point(i + 1, j + 1, k);
                        point_se = points[index_se];
                        pairs.push_back({point_nw, point_se}); // allows translating diagonally in plane
                        pairs.push_back({point_se, point_nw}); // allows translating diagonally in plane
                        pairs.push_back({point_sw, point_ne}); // allows translating diagonally in plane
                        pairs.push_back({point_ne, point_sw}); // allows translating diagonally in plane
                    }

                    if (are_indices_valid(i, j, k + 1)) {
                        index_up = find_index_point(i, j, k + 1);
                        point_up = points[index_up];
                        pairs.push_back({point_nw, point_up}); // allows translating out of plane
                        pairs.push_back({point_up, point_nw}); // allows translating out of plane
                    }
                }
            }
        }

        for (const auto &pair : pairs) {
            auto vertices1 = lookup_vertices(pair.first->index_point);
            auto vertices2 = lookup_vertices(pair.second->index_point);

            for (const auto &vertex1 : vertices1) {
                for (const auto &vertex2 : vertices2) {
                    bool is_same_x = (vertex1->point->x == vertex2->point->x);
                    bool is_same_y = (vertex1->point->y == vertex2->point->y);
                    bool is_same_z = (vertex1->point->z == vertex2->point->z);
                    bool is_same_point = (vertex1->point->index_point == vertex2->point->index_point);
                    bool is_same_heading = (vertex1->heading == vertex2->heading);

                    bool is_wait = (is_same_point && is_same_heading);
                    bool is_rotate = (is_same_point && !is_same_heading);
                    bool is_translate_in_plane = (!is_same_point && is_same_z && is_same_heading);
                    bool is_translate_out_of_plane = (!is_same_point && is_same_x && is_same_y && is_same_heading);

                    bool is_valid = true;
                    if (is_wait || is_rotate) {
                        auto i = vertex2->point->key->i;
                        auto j = vertex2->point->key->j;
                        auto k = vertex2->point->key->k;
                        if (vertex2->heading == 0.0) {
                            if (!are_indices_valid(i + 1, j, k)) {
                                is_valid = false;
                            }
                        } else if (vertex2->heading == 45.0) {
                            if (!are_indices_valid(i + 1, j + 1, k)) {
                                is_valid = false;
                            }
                        } else if (vertex2->heading == 90.0) {
                            if (!are_indices_valid(i, j + 1, k)) {
                                is_valid = false;
                            }
                        } else if (vertex2->heading == 135.0) {
                            if (!are_indices_valid(i - 1, j + 1, k)) {
                                is_valid = false;
                            }
                        } else if (vertex2->heading == 180.0) {
                            if (!are_indices_valid(i - 1, j, k)) {
                                is_valid = false;
                            }
                        } else if (vertex2->heading == 225.0) {
                            if (!are_indices_valid(i - 1, j - 1, k)) {
                                is_valid = false;
                            }
                        } else if (vertex2->heading == 270.0) {
                            if (!are_indices_valid(i, j - 1, k)) {
                                is_valid = false;
                            }
                        } else if (vertex2->heading == 315.0) {
                            if (!are_indices_valid(i + 1, j - 1, k)) {
                                is_valid = false;
                            }
                        }
                    }

                    if (!is_valid) {
                        continue;
                    }

                    auto edge = std::make_shared<Edge>();
                    edge->index_edge = static_cast<int>(edges.size());
                    edge->vertex_parent = vertex1;
                    edge->vertex_child = vertex2;
                    if (is_wait) {
                        edge->type = edge_type::WAIT;
                        edges.push_back(edge);
                        edges_by_index_vertex[edge->vertex_parent->index_vertex].push_back(edge);
                    } else if (is_rotate) {
                        edge->type = edge_type::ROTATE;
                        edges.push_back(edge);
                        edges_by_index_vertex[edge->vertex_parent->index_vertex].push_back(edge);
                    } else if (is_translate_in_plane) {
                        auto dy = *(vertex2->point->y) - *(vertex1->point->y);
                        auto dx = *(vertex2->point->x) - *(vertex1->point->x);
                        auto theta = (180 / M_PI) * std::atan2(dy, dx);
                        if (theta < 0.0) {
                            theta += 360.0;
                        }
                        if (theta == vertex1->heading) {
                            edge->type = edge_type::TRANSLATE_IN_PLANE;
                            edges.push_back(edge);
                            edges_by_index_vertex[edge->vertex_parent->index_vertex].push_back(edge);
                        }
                    } else if (is_translate_out_of_plane) {
                        edge->type = edge_type::TRANSLATE_OUT_OF_PLANE;
                        edges.push_back(edge);
                        edges_by_index_vertex[edge->vertex_parent->index_vertex].push_back(edge);
                    }
                }
            }
        }

        n_edges = static_cast<int>(edges.size());
    }


    Linspace1D generate_linspace1d(const double &_n_min, const double &_n_max, const int &_n_n)
    {
        assert(_n_n >= 1);
        if (_n_min == _n_max && _n_n > 1) {
            throw std::logic_error("generate_linspace : lower and upper bounds cannot be equal with more than one point");
        }

        Linspace1D linspace(_n_n);
        double step;
        if (_n_n == 1) {
            step = 0.0;
        } else {
            step = (_n_max - _n_min) / (_n_n - 1);
        }
        for (auto i = 0; i < _n_n; ++i) {
            linspace[i] = std::make_shared<double>(_n_min + step * i);
        }
        return linspace;
    }


    Mesh3D generate_mesh3d_uniform(const int &_n_i, const int &_n_j, const int &_n_k, const double &_value)
    {
        assert(_n_i >= 1 && _n_j >= 1 && _n_k >= 1);

        Mesh3D mesh;
        mesh.resize(_n_i, std::vector<Linspace1D>(_n_j, std::vector<std::shared_ptr<double>>(_n_k, std::make_shared<double>(_value))));
        return mesh;
    }

} // namespace mess2_algorithms
