#ifndef MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
#define MESS2_ALGORITHM_PLUGINS_ACTOR_HPP

#include "mess2_algorithm_plugins/common.hpp"

#include "mess2_algorithm_plugins/graph/_graph.hpp"

namespace mess2_algorithms
{
    using occupancy = std::vector<std::vector<int64_t>>;

    class Actor
    {
    public:
        Actor();

        double get_time_to_wait();
        double get_time_to_rotate(const Vertex& vertex_parent, const Vertex& vertex_child);
        double get_time_to_translate(const Vertex& vertex_parent, const Vertex& vertex_child);
        std::tuple<double, double> get_cost_to_transition(const int64_t& index_edge);

        void define_actor(const double& k_ang, const double& k_lin, const double& x_tol_ang, const double& x_tol_lin, const double& u_max_ang, const double& u_max_lin, const double& radius);
        void fill_actor(const Graph& graph, const std::vector<double>& threat);

    private:

        void fill_occupancies_by_vertex(const std::vector<Vertex>& vertices, const double& radius);
        void fill_scores_by_edges(const std::vector<Edge>& edges,const std::vector<double>& threat);
        void fill_times_by_edges(const std::vector<Edge>& edges, const std::vector<Vertex>& vertices);

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

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
