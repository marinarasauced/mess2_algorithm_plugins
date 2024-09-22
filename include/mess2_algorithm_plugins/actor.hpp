#ifndef MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
#define MESS2_ALGORITHM_PLUGINS_ACTOR_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/graph.hpp"

struct hash_occupancies {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1);
    }
};

namespace mess2_algorithms
{
    // using occupancy = std::vector<std::vector<int64_t>>;
    using occupancy = std::unordered_map<std::pair<double, double>, std::vector<int64_t>, hash_occupancies>;

    class Actor
    {
    public:
        Actor();

        double get_time_to_wait();
        double get_time_to_rotate(const Vertex& vertex_parent, const Vertex& vertex_child);
        double get_time_to_translate(const Vertex& vertex_parent, const Vertex& vertex_child);
        std::tuple<double, double> get_cost_to_transition(const int64_t& index_edge);

        void define_actor(const double& k_ang, const double& k_lin, const double& x_tol_ang, const double& x_tol_lin, const double& u_max_ang, const double& u_max_lin, const double& radius);

        void fill_occupancies_by_x_and_y(const arma::mat& x_mesh, const arma::mat& y_mesh, const Graph& graph);
        void fill_scores_by_edges(const Graph& graph, const std::vector<double>& weights);
        void fill_times_by_edges(const Graph& graph);
        void fill_actor(const arma::mat& x_mesh, const arma::mat& y_mesh, const Graph& graph, const std::vector<double>& threat);

        std::vector<double> get_scores();

    private:
        occupancy occupancies_;
        std::vector<double> scores_;
        std::vector<double> times_;

        double k_ang_;
        double k_lin_;
        double x_tol_ang_;
        double x_tol_lin_;
        double u_max_ang_;
        double u_max_lin_;
        double radius_;
    };

    Actor generate_actor_turtlebot3(const std::string& turtlebot3_model, const double& u_ratio, const double& r_ratio);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
