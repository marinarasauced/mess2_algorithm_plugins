
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
    Actor::Actor() {};

    double Actor::get_time_to_wait() {
        return 1.0;
    }

    double Actor::get_time_to_rotate(const Vertex& vertex_parent, const Vertex& vertex_child) {
        const auto theta_parent = vertex_parent.get_theta();
        const auto theta_child = vertex_child.get_theta();
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

    std::tuple<double, double> Actor::get_cost_to_transition(const Edge& edge)
    {
        const auto index_child = edge.get_index_child();
        const auto score = scores_[index_child];
        const auto time = times_[index_child];

        return std::tuple<double, double>{score, time};
    }

    void Actor::fill_occupancies_by_vertex(const std::vector<Vertex>& vertices, const double& radius)
    {
        const auto n_elem = static_cast<int64_t>(vertices.size());
        occupancies_.resize(n_elem);

        for (int64_t iter = 0; iter < n_elem; ++iter) {
            const auto vertex_parent = vertices[iter];
            const auto x_parent = vertex_parent.get_x();
            const auto y_parent = vertex_parent.get_y();

            for (int64_t jter = 0; jter < n_elem; ++jter) {
                const auto vertex_child = vertices[jter];
                const auto x_child = vertex_child.get_x();
                const auto y_child = vertex_child.get_y();

                const auto distance = std::sqrt(
                    std::pow(x_parent - x_child, 2) +
                    std::pow(y_parent - y_child, 2)
                );

                if (distance <= radius) {
                    occupancies_[iter].emplace_back(jter);
                }
            }
        }
    }

    void Actor::fill_threat_by_occupancies(const std::vector<double>& weights)
    {
        const auto n_elem_weights = static_cast<int64_t>(weights.size());
        const auto n_elem_occupancies = static_cast<int64_t>(occupancies_.size());

        if (n_elem_weights != n_elem_occupancies) {
            throw std::invalid_argument("weights and occupancies must have the same size.");
        }

        scores_.resize(n_elem_weights);
        for (int64_t iter = 0; iter < n_elem_weights; ++iter) {
            double threat_iter = 0.0;
            for (const auto& occupied : occupancies_[iter]) {
                threat_iter = threat_iter + weights[occupied];
            }
            scores_[iter] = threat_iter;
        }
    }

} // namespace mess2_algorithms
