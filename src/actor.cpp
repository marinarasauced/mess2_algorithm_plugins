
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
    Actor::Actor(const double& k_ang, const double& k_lin, const double& x_tol_ang, const double& x_tol_lin, const double& u_max_ang, const double& u_max_lin, const double& radius)
    {
        k_ang_ = k_ang;
        k_lin_ = k_lin;
        x_tol_ang_ = x_tol_ang;
        x_tol_lin_ = x_tol_lin;
        u_max_ang_ = u_max_ang;
        u_max_lin_ = u_max_lin;
        radius_ = radius;
    }

    double Actor::calculate_time_to_wait()
    {
        return 0.5;
    }

    double Actor::calculate_time_to_rotate(const graph_vertex& vertex_parent, const graph_vertex& vertex_child)
    {
        auto diff = std::abs((M_PI / 180) * (vertex_parent.theta - vertex_child.theta));
        
        if (diff > M_PI) {
            diff -= 2 * M_PI;
        }
        diff = std::abs(diff);
        
        if (diff == 0) {
            return 0.0;
        }

        auto theta_p = diff;
        auto theta_c = 0.0;
        if (diff * k_ang_ > u_max_ang_) {
            theta_p = u_max_ang_ / k_ang_;
            theta_c = diff - theta_p;
        }

        const auto time_p = -std::log(x_tol_ang_ / theta_p) / k_ang_;
        const auto time_c = theta_c / u_max_ang_;
        return time_p + time_c;
    }

    double Actor::calculate_time_to_translate(const graph_vertex& vertex_parent, const graph_vertex& vertex_child)
    {
        const auto diff = std::sqrt(
            std::pow(vertex_parent.x - vertex_child.x, 2) +
            std::pow(vertex_parent.y - vertex_child.y, 2)
        );

        const auto time_c = diff / u_max_lin_;
        return time_c;
    }

    void Actor::generate_occupancies_using_graph_map(const Graph& graph, const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        if (x_mesh.n_rows != y_mesh.n_rows || x_mesh.n_cols != y_mesh.n_cols) {
            throw std::runtime_error("x_mesh and y_mesh must have the same dimensions");
        } else if (radius_ <= 0.0) {
            throw std::runtime_error("radius cannot be less than zero");
        }

        const int64_t n_rows = x_mesh.n_rows;
        const int64_t n_cols = x_mesh.n_cols;
        const int64_t n_elem = x_mesh.n_elem;
        occupancies_.reserve(n_elem);

        const auto resolution = std::min(x_mesh(0, 1) - x_mesh(0, 0), y_mesh(1, 0) - y_mesh(0, 0));
        const auto steps = static_cast<int64_t>(std::ceil(radius_ / resolution));
        for (int64_t iter = 0; iter < graph.n_vertices; ++iter) {
            const auto vertex_parent = graph.lookup_vertex(iter);
            auto index_row = static_cast<int64_t>((vertex_parent.y - y_mesh(0, 0)) / resolution);
            auto index_col = static_cast<int64_t>((vertex_parent.x - x_mesh(0, 0)) / resolution);
            index_row = std::max(int64_t(0), std::min(n_rows - 1, index_row));
            index_col = std::max(int64_t(0), std::min(n_cols - 1, index_col));

            const int64_t j_min = std::max(int64_t(0), index_row - steps);
            const int64_t j_max = std::min(n_rows - 1, index_row + steps);
            const int64_t k_min = std::max(int64_t(0), index_col - steps);
            const int64_t k_max = std::min(n_cols - 1, index_col + steps);

            for (int64_t jter = j_min; jter <= j_max; ++jter) {
                for (int64_t kter = k_min; kter <= k_max; ++kter) {
                    const auto x_child = x_mesh(jter, kter);
                    const auto y_child = y_mesh(jter, kter);
                    const auto diff = std::sqrt(
                        std::pow(vertex_parent.x - x_child, 2) +
                        std::pow(vertex_parent.y - y_child, 2)
                    );

                    if (diff <= radius_) {
                        auto indices = graph.lookup_map(x_child, y_child);
                        for (const auto& index : indices) {
                            auto occupancies = occupancies_[{vertex_parent.x, vertex_parent.y}];
                            if (std::find(occupancies.begin(), occupancies.end(), index) == occupancies.end()) {
                                occupancies_[{vertex_parent.x, vertex_parent.y}].push_back(index);
                            }
                        }
                    }
                }
            }
        }
    }

    void Actor::generate_transition_costs_by_edge(const Graph& graph, const std::vector<double>& threat)
    {
        costs_.clear();
        costs_.resize(graph.n_edges);

        for (int64_t iter = 0; iter < graph.n_edges; ++iter) {
            const auto edge = graph.lookup_edge(iter);
            const auto vertex_child = graph.lookup_vertex(edge.index_child);

            double cost = std::numeric_limits<double>::max();
            const auto search = occupancies_.find({vertex_child.x, vertex_child.y});
            if (search != occupancies_.end()) {
                cost = 0.0;
                for (const auto& index : search->second) {
                    if (index < static_cast<int64_t>(threat.size())) {
                        cost += threat[index]; // assumes that there is an equal number of vertices per (x, y) coordinate and that all (x, y) coordinates are visited once before subsequent variations of the edge are visited.
                    }
                }
            }
            costs_[iter] = cost;
        }
    }

    void Actor::generate_transition_times_by_edge(const Graph& graph)
    {
        times_.clear();
        times_.resize(graph.n_edges);

        for (int64_t iter = 0; iter < graph.n_edges; ++iter) {
            const auto edge = graph.lookup_edge(iter);

            double time = std::numeric_limits<double>::max();
            if (edge.type == "wait") {
                time = calculate_time_to_wait();
            } else if (edge.type == "rotate") {
                time = calculate_time_to_rotate(graph.lookup_vertex(edge.index_parent), graph.lookup_vertex(edge.index_child));
            } else if (edge.type == "translate") {
                time = calculate_time_to_translate(graph.lookup_vertex(edge.index_parent), graph.lookup_vertex(edge.index_child));
            }
            times_[iter] = time;
        }
    }

    void Actor::generate_transition_heuristic_by_edge(const Graph& graph)
    {
        heuristics_.clear();
        heuristics_.resize(graph.n_edges);

        const auto vertex_target = graph.lookup_vertex(index_target_);

        std::vector<double> diffs(graph.n_edges);
        for (int64_t iter = 0; iter < graph.n_edges; ++iter) {
            const auto edge = graph.lookup_edge(iter);
            const auto vertex_source = graph.lookup_vertex(edge.index_child);
            const auto diff = std::sqrt(
                std::pow(vertex_source.x - vertex_target.x, 2) +
                std::pow(vertex_source.y - vertex_target.y, 2)
            );
            diffs[iter] = diff;
        }

        const auto scale = *std::max_element(diffs.begin(), diffs.end());
        if (scale <= 0.0) {
            throw std::out_of_range("scale must be greater than zero");
        }

        for (int64_t iter = 0; iter < graph.n_edges; ++iter) {
            heuristics_[iter] = diffs[iter] * (costs_[iter] / scale);
        }
    }

    std::vector<int64_t> Actor::lookup_occupancies(const double& x, const double& y) const
    {
        return occupancies_.find({x, y})->second;
    }

    double Actor::lookup_cost(const double& index_edge) const
    {
        return costs_[index_edge];
    }

    double Actor::lookup_time(const double& index_edge) const
    {
        return times_[index_edge];
    }

    double Actor::lookup_heuristic(const double& index_edge) const
    {
        return heuristics_[index_edge];
    }

    void Actor::set_source(const int64_t& index_source)
    {
        index_source_ = index_source;
    }

    void Actor::set_target(const int64_t& index_target)
    {
        index_target_ = index_target;
    }

    void Actor::save_actor(const std::string& path_actor)
    {
        std::string path_new;
        if (!path_actor.empty() && path_actor[0] == '~') {
            const char* home = getenv("HOME");
            if (home) {
                path_new = std::string(home) + path_actor.substr(1);
            } else {
                throw std::runtime_error("could not determine the home directory");
            }
        }

        std::ofstream file(path_new);
        if (!file.is_open()) {
            throw std::runtime_error("could not open file for writing");
        }

        file << "k_ang" << "," << "k_lin" << "," << "x_tol_ang" << "," << "x_tol_lin" << "," << "u_max_ang" << "," << "u_max_lin" << "," << "radius" << "," << "t_wait" << "\n";

        file << k_ang_ << "," << k_lin_ << "," << x_tol_ang_ << "," << x_tol_lin_ << "," << u_max_ang_ << "," << u_max_lin_ << "," << radius_ << "," << calculate_time_to_wait() << "\n";

        file.close();
    }

    Actor generate_actor_turtlebot3(const std::string& type, const double& u_ratio, const double& r_ratio)
    {
        const double k_ang = 3.0;
        const double k_lin = 1.0;
        const double x_tol_ang = 0.01;
        const double x_tol_lin = 0.002;
        double u_max_ang;
        double u_max_lin;
        double radius;

        if (type == "burger") {
            u_max_ang = u_ratio * 2.84;
            u_max_lin = u_ratio * 0.22;
            radius = 0.105 * r_ratio;
        } else if (type == "waffle" || type == "wafflepi" || type == "waffle_pi") {
            u_max_ang = u_ratio * 1.82;
            u_max_lin = u_ratio * 0.26;
            radius = 0.220 * r_ratio;
        } else {
            u_max_ang = u_ratio * 1.82;
            u_max_lin = u_ratio * 0.22;
            radius = 0.220 * r_ratio;
        }

        auto actor = Actor(k_ang, k_lin, x_tol_ang, x_tol_lin, u_max_ang, u_max_lin, radius);

        return actor;
    }

} // namespace mess2_algorithms
