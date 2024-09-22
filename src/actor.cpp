
#include "mess2_algorithm_plugins/actor.hpp"

template <typename T>
T clamp(T value, T min_value, T max_value) {
    return std::max(min_value, std::min(value, max_value));
}

struct pair_hash {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1);
    }
};

namespace mess2_algorithms
{
    Actor::Actor() {};

    double Actor::get_time_to_wait() {
        return 1.0;
    }

    double Actor::get_time_to_rotate(const Vertex& vertex_parent, const Vertex& vertex_child) {
        const auto theta_parent = vertex_parent.get_theta() * (M_PI / 180);
        const auto theta_child = vertex_child.get_theta() * (M_PI / 180);
        auto dtheta = theta_child - theta_parent;

        if (dtheta > M_PI) {
            dtheta -= 2 * M_PI;
        } else if (dtheta < -M_PI) {
            dtheta += 2 * M_PI;
        }

        if (dtheta == 0.0) {
            return 0.0;
        }

        dtheta = std::abs(dtheta);
        auto dtheta_c = 0.0;
        auto dtheta_p = dtheta;
        if (dtheta * k_ang_ > u_max_ang_) {
            dtheta_p = u_max_ang_ / k_ang_;
            dtheta_c = dtheta - dtheta_p;
        }

        const auto time_c = dtheta_c / u_max_ang_;
        const auto time_p = -std::log(x_tol_ang_ / dtheta_p) / k_ang_;
        return time_c + time_p;
    }

    double Actor::get_time_to_translate(const Vertex& vertex_parent, const Vertex& vertex_child) {
        const auto x_parent = vertex_parent.get_x();
        const auto y_parent = vertex_parent.get_y();
        const auto x_child = vertex_child.get_x();
        const auto y_child = vertex_child.get_y();
        const auto distance = std::sqrt(
            std::pow(x_parent - x_child, 2) +
            std::pow(y_parent - y_child, 2)
        );

        const auto time_c = distance / u_max_lin_;
        return time_c;
    }

    std::tuple<double, double> Actor::get_cost_to_transition(const int64_t& index_edge)
    {
        const auto score = scores_[index_edge];
        const auto time = times_[index_edge];

        return std::tuple<double, double>{score * time, time};
    }

    void Actor::fill_occupancies_by_x_and_y(const arma::mat& x_mesh, const arma::mat& y_mesh, const Graph& graph)
    {
        if (x_mesh.n_rows != y_mesh.n_rows || x_mesh.n_cols != y_mesh.n_cols) {
            throw std::runtime_error("x_mesh and y_mesh must have the same dimensions");
        } else if (radius_ <= 0.0) {
            throw std::runtime_error("radius cannot be less than zero");
        }

        const auto vertices = graph.get_vertices();
        const auto map_vertices = graph.get_vertices_map();

        const int64_t n_rows = x_mesh.n_rows;
        const int64_t n_cols = x_mesh.n_cols;
        const int64_t n_elem = x_mesh.n_elem;
        const int64_t n_vertices = vertices.size();

        occupancies_.clear();
        occupancies_.reserve(n_elem);

        const auto resolution = std::min(x_mesh(0, 1) - x_mesh(0, 0), y_mesh(1, 0) - y_mesh(0, 0));
        const auto steps = static_cast<int64_t>(std::ceil(radius_ / resolution));
        
        for (int64_t iter = 0; iter < n_vertices; ++iter) {
            const auto vertex_parent = vertices[iter];
            const auto x_parent = vertex_parent.get_x();
            const auto y_parent = vertex_parent.get_y();

            auto row_index = static_cast<int64_t>((y_parent - y_mesh(0, 0)) / resolution);
            auto col_index = static_cast<int64_t>((x_parent - x_mesh(0, 0)) / resolution);

            row_index = clamp(row_index, int64_t(0), n_rows - 1);
            col_index = clamp(col_index, int64_t(0), n_cols - 1);

            for (int64_t jter = std::max(int64_t(0), row_index - steps); jter <= std::min(n_rows - 1, row_index + steps); ++jter) {
                for (int64_t kter = std::max(int64_t(0), col_index - steps); kter <= std::min(n_cols - 1, col_index + steps); ++kter) {
                    const auto x_child = x_mesh(jter, kter);
                    const auto y_child = y_mesh(jter, kter);

                    const auto distance = std::sqrt(
                        std::pow(x_parent - x_child, 2) +
                        std::pow(y_parent - y_child, 2)
                    );

                    if (distance <= radius_) {
                        const auto map = map_vertices.find({x_child, y_child});
                        if (map != map_vertices.end()) {
                            const std::vector<int64_t> indices = map->second;
                            const auto index = indices[0];

                            auto& occupancy = occupancies_[{x_parent, y_parent}];
                            if (std::find(occupancy.begin(), occupancy.end(), index) == occupancy.end()) {
                                occupancies_[{x_parent, y_parent}].push_back(index);
                            }
                        }
                    }
                }
            }
        }
    }

    void Actor::fill_scores_by_edges(const Graph& graph, const std::vector<double>& weights)
    {
        const auto edges = graph.get_edges();
        const auto vertices = graph.get_vertices();

        const auto n_edges = static_cast<int64_t>(edges.size());
        scores_.clear();
        scores_.resize(n_edges);

        for (int64_t iter = 0; iter < n_edges; ++iter) {
            const auto edge = edges[iter];
            const auto index_child = edge.get_index_child();
            const auto vertex_child = vertices[index_child];
            
            const auto found = occupancies_.find({vertex_child.get_x(), vertex_child.get_y()});

            double threat = 0.0;
            if (found != occupancies_.end()) {
                for (const auto& index : found->second) {
                    threat = threat + weights[index];
                }
            }
            scores_[iter] = threat;
        }
    }

    void Actor::fill_times_by_edges(const Graph& graph)
    {
        const auto edges = graph.get_edges();
        const auto vertices = graph.get_vertices();

        const auto n_edges = static_cast<int64_t>(edges.size());
        times_.clear();
        times_.resize(n_edges);

        for (int64_t iter = 0; iter < n_edges; ++iter) {
            const auto edge = edges[iter];
            const auto type = edge.get_type();
            
            double time;
            if (type=="wait") {
                time = get_time_to_wait();
            } else if (type=="rotate") {
                const auto vertex_parent = vertices[edge.get_index_parent()];
                const auto vertex_child = vertices[edge.get_index_child()];
                time = get_time_to_rotate(vertex_parent, vertex_child);
            } else if (type=="translate") {
                const auto vertex_parent = vertices[edge.get_index_parent()];
                const auto vertex_child = vertices[edge.get_index_child()];
                time = get_time_to_translate(vertex_parent, vertex_child);
            }

            times_[iter] = time;
        }
    }

    void Actor::define_actor(const double& k_ang, const double& k_lin, const double& x_tol_ang, const double& x_tol_lin, const double& u_max_ang, const double& u_max_lin, const double& radius)
    {
        k_ang_ = k_ang;
        k_lin_ = k_lin;
        x_tol_ang_ = x_tol_ang;
        x_tol_lin_ = x_tol_lin;
        u_max_ang_ = u_max_ang;
        u_max_lin_ = u_max_lin;
        radius_ = radius;
    }

    void Actor::fill_actor(const arma::mat& x_mesh, const arma::mat& y_mesh, const Graph& graph, const std::vector<double>& threat)
    {
        const auto edges = graph.get_edges();
        const auto vertices = graph.get_vertices();

        (void) fill_occupancies_by_x_and_y(x_mesh, y_mesh, graph);
        (void) fill_scores_by_edges(graph, threat);
        (void) fill_times_by_edges(graph);
    }

    std::vector<double> Actor::get_scores() {
        return scores_;
    }
    
    Actor generate_actor_turtlebot3(const std::string& turtlebot3_model, const double& u_ratio, const double& r_ratio)
    {
        Actor actor;

        double k_ang = 1.0;
        double k_lin = 1.0;
        double x_tol_ang = 0.01;
        double x_tol_lin = 0.01;
        double u_max_ang;
        double u_max_lin;
        double radius;

        if (turtlebot3_model=="burger") {
            u_max_ang = 2.84 * u_ratio;
            u_max_lin = 0.22 * u_ratio;
            radius = 0.105 * r_ratio;
        } else if (turtlebot3_model=="waffle" || turtlebot3_model=="waffle_pi" || turtlebot3_model=="wafflepi") {
            u_max_ang = 1.82 * u_ratio;
            u_max_lin = 0.26 * u_ratio;
            radius = 0.220 * r_ratio;
        } else {
            u_max_ang = 1.82 * u_ratio;
            u_max_lin = 0.22 * u_ratio;
            radius = 0.220 * r_ratio;
        }

        actor.define_actor(k_ang, k_lin, x_tol_ang, x_tol_lin, u_max_ang, u_max_lin, radius);

        return actor;
    }

} // namespace mess2_algorithms
