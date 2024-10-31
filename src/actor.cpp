
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
    std::list<std::shared_ptr<Key3D>> compute_occupancies_symbolically(const std::shared_ptr<Graph> &_graph, const double &_radius)
    {
        std::list<std::shared_ptr<Key3D>> occupancies;

        double res_x, res_y, res_z;
        int steps_x, steps_y, steps_z;

        if (_graph->n_i < 2) {
            res_x = 0.0;
            steps_x = 0;
        } else {
            res_x = (_graph->lookup_x(_graph->n_i - 1) - _graph->lookup_x(0));
            steps_x = static_cast<int>(std::ceil(_radius / res_x));
        }
        if (_graph->n_j < 2) {
            res_y = 0.0;
            steps_y = 0;
        } else {
            res_y = (_graph->lookup_y(_graph->n_j - 1) - _graph->lookup_y(0));
            steps_y = static_cast<int>(std::ceil(_radius / res_y));
        }
        if (_graph->n_k < 2) {
            res_z = 0.0;
            steps_z = 0;
        } else {
            res_z = (_graph->lookup_z(_graph->n_k - 1) - _graph->lookup_z(0));
            steps_z = static_cast<int>(std::ceil(_radius / res_z));
        }

        for (auto i = -steps_x; i <= steps_x; ++i) {
            for (auto j = -steps_y; j <= steps_y; ++j) {
                for (auto k = -steps_z; k <= steps_z; ++ k) {
                    const double child_x = i * res_x;
                    const double child_y = j * res_y;
                    const double child_z = k * res_z;
                    
                    const double dxyz = std::sqrt(std::pow(0.0 - child_x, 2) + std::pow(0.0 - child_y, 2) + std::pow(0.0 - child_z, 2));
                    if (dxyz <= _radius) {
                        auto key = std::make_shared<Key3D>();
                        key->index_key = occupancies.size();
                        key->i = i;
                        key->j = j;
                        key->k = k;
                        occupancies.push_back(key);
                    }
                }
            }
        }
        return occupancies;
    }


    double Actor::compute_time_rotate(const std::shared_ptr<Edge> &_edge)
    {
        if (_edge->type != edge_type::ROTATE) {
            return 0.0;
        }

        auto dtheta = std::abs((M_PI / 180.0) * (_edge->vertex_parent->heading - _edge->vertex_child->heading));

        if (dtheta > M_PI) {
            dtheta -= 2 * M_PI;
        }
        dtheta = std::abs(dtheta);

        if (dtheta == 0.0) {
            return 0.0;
        }

        auto theta_p = dtheta;
        auto theta_c = 0.0;
        if (dtheta * k_ang > u_ang_max) {
            theta_p = u_ang_max / k_ang;
            theta_c = dtheta - theta_p;
        }

        const auto time_p = -std::log(x_ang_tol / theta_p) / k_ang;
        const auto time_c = theta_c / u_ang_max;
        return time_p + time_c;
    }


    double Actor::compute_time_translate(const std::shared_ptr<Edge> &_edge)
    {
        if (_edge->type == edge_type::ROTATE || _edge->type == edge_type::WAIT) {
            return 0.0;
        }

        auto dxyz = std::sqrt(
            std::pow(*(_edge->vertex_parent->point->x) - *(_edge->vertex_child->point->x), 2) +
            std::pow(*(_edge->vertex_parent->point->y) - *(_edge->vertex_child->point->y), 2) + 
            std::pow(*(_edge->vertex_parent->point->z) - *(_edge->vertex_child->point->z), 2)
        );

        const auto time_c = dxyz / u_lin_max;
        return time_c;
    }


    std::list<std::shared_ptr<Key3D>> Actor::lookup_occupancies_symbolically(const std::shared_ptr<Graph> &_graph, int _i, int _j, int _k, const std::list<std::shared_ptr<Key3D>> &_map)
    {
        std::list<std::shared_ptr<Key3D>> occupancies;
        for (const auto & _key : _map) {
            auto i = _key->i + _i;
            auto j = _key->j + _j;
            auto k = _key->k + _k;
            if (!_graph->are_indices_valid(i, j, k)) {
                continue;
            }

            auto index_point = _graph->find_index_point(i, j, k);
            auto key = _graph->lookup_key(index_point);
            occupancies.push_back(key);
        }
        return occupancies;
    }


    void Actor::compute_occupancies_weights_heuristics(const std::shared_ptr<Graph> &_graph)
    {
        occupancies_by_index_point.clear();
        g_by_index_point.clear();
        h_by_index_point.clear();

        std::unordered_map<int, double> distances_by_index_point;
        auto target_key = _graph->lookup_vertex(index_target)->point->key;
        auto target_x = _graph->lookup_x(target_key->i);
        auto target_y = _graph->lookup_y(target_key->j);
        auto target_z = _graph->lookup_z(target_key->k);
        double distance_max = 0.0;

        for (auto i = 0; i < _graph->n_i; ++i) {
            for (auto j = 0; j < _graph->n_j; ++j) {
                for (auto k = 0; k < _graph->n_k; ++k) {
                    auto occupancies = lookup_occupancies_symbolically(_graph, i, j, k, occupancies_symbolic);

                    auto index_point = _graph->find_index_point(i, j, k);
                    occupancies_by_index_point[index_point] = occupancies;

                    double weight = 0.0;
                    for (const auto &occupancy : occupancies) {
                        auto value_threat = _graph->lookup_threat(occupancy->i, occupancy->j, occupancy->k);
                        weight += value_threat;
                    }
                    g_by_index_point[index_point] = weight;

                    auto parent_x = _graph->lookup_x(i);
                    auto parent_y = _graph->lookup_y(j);
                    auto parent_z = _graph->lookup_z(k);

                    auto dx = std::abs(parent_x - target_x);
                    auto dy = std::abs(parent_y - target_y);
                    auto dz = std::abs(parent_z - target_z);

                    double distance; // = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (!_graph->use_diagonals_in_plane) {
                        distance = (dx + dy) + dz;
                    } else if (_graph->use_diagonals_in_plane) {
                        distance = (dx + dy + (std::sqrt(2) - 2) * std::min(dx, dy)) + dz;
                    }
                    distances_by_index_point[index_point] = distance;
                    if (distance > distance_max) {
                        distance_max = distance;
                    }
                    // if (index_point == vertex_target->point->index_point) {
                    //     distance_max = distance;
                    // }
                }
            }
        }
        for (auto i = 0; i < _graph->n_i; ++i) {
            for (auto j = 0; j < _graph->n_j; ++j) {
                for (auto k = 0; k < _graph->n_k; ++k) {
                    auto index_point = _graph->find_index_point(i, j, k);
                    auto distance = distances_by_index_point.find(index_point)->second / distance_max;
                    auto weight = g_by_index_point.find(index_point)->second;
                    auto heuristic = distance * weight;
                    h_by_index_point[index_point] = heuristic;
                }
            }
        }
    }


    void Actor::compute_times(const std::shared_ptr<Graph> &_graph)
    {
        t_by_index_edge.clear();

        for (auto i = 0; i < _graph->n_edges; ++i)
        {
            auto edge = _graph->lookup_edge(i);

            auto time_wait = compute_time_wait(edge);
            auto time_rotate = compute_time_rotate(edge);
            auto time_translate = compute_time_translate(edge);
            auto dt = time_wait + time_rotate + time_translate;

            t_by_index_edge[i] = dt;
        }
    }


    void Actor::compute_obstacle_avoidance_table(const std::shared_ptr<Graph> &_graph)
    {
        for (int i = 0; i < _graph->n_i; ++i) {
            for (int j = 0; j < _graph->n_j; ++j) {
                for (int k = 0; k < _graph->n_k; ++k) {
                    auto index_point = _graph->find_index_point(i, j, k);

                    bool is_in_ot = false;
                    bool bool_in_ot = false;
                    auto iterator = ot.find(index_point);
                    if (iterator != ot.end()) {
                        is_in_ot = true;
                        bool_in_ot = iterator->second;
                    }

                    auto occupancies = lookup_occupancies_symbolically(_graph, i, j, k, occupancies_symbolic);

                    for (const auto &occupancy : occupancies) {
                        auto is_obstacle = (_graph->lookup_obstacle(occupancy->i, occupancy->j, occupancy->k) >= _graph->threshold_obstacles);
                        if (!is_in_ot || (is_in_ot && !bool_in_ot)) {
                            ot[index_point] = is_obstacle;
                        }
                    }
                }
            }
        }
    }


    void Actor::save_actor(const std::string &_path_actor)
    {
        std::string path_new;
        if (!_path_actor.empty() && _path_actor[0] == '~') {
            const char* home = getenv("HOME");
            if (home) {
                path_new = std::string(home) + _path_actor.substr(1);
            } else {
                throw std::runtime_error("could not determine the home directory");
            }
        }

        std::ofstream file(path_new);
        if (!file.is_open()) {
            throw std::runtime_error("could not open file for writing");
        }

        file << "k_ang" << "," << "k_lin" << "," << "x_tol_ang" << "," << "x_tol_lin" << "," << "u_max_ang" << "," << "u_max_lin" << "," << "radius" << "," << "t_wait" << "\n";

        file << k_ang << "," << k_lin << "," << x_ang_tol << "," << x_lin_tol << "," << u_ang_max << "," << u_lin_max << "," << radius << "," << time_wait << "\n";

        file.close();
    }

} // namespace mess2_algorithms
