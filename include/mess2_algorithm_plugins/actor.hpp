#ifndef MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
#define MESS2_ALGORITHM_PLUGINS_ACTOR_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph.hpp"

namespace mess2_algorithms
{
    /**
     * @class Actor
     * @brief manages an actors attributes and specifically calculated transition scores and times.
     * 
     * this class provides the tools to calculate transition scores and times w.r.t to the actor's configurations.
     */
    class Actor
    {
    public:
        /**
         * @brief constructs a class instance of Actor.
         */
        Actor(const double& k_ang, const double& k_lin, const double& x_tol_ang, const double& x_tol_lin, const double& u_max_ang, const double& u_max_lin, const double& radius);

        /**
         * @brief calculates the time to wait between vertices.
         * 
         * this method calculates the wait time at an edge and is a static value.
         * 
         * @return the time to wait.
         */
        double calculate_time_to_wait();

        /**
         * @brief calculates the time to rotate between vertices.
         * 
         * this method assumes a linear model with a proportional state feedback controller for reducing the error between an iniial and terminal heading given that there is a maximum angular velocity.
         * 
         * @param vertex_parent the parent vertex.
         * @param vertex_child the child vertex.
         * @return the time to rotate.
         */
        double calculate_time_to_rotate(const graph_vertex& vertex_parent, const graph_vertex& vertex_child);

        /**
         * @brief calculates the time to translate between vertices.
         * 
         * this method calculates the time to translate between vertices assuming constant linear velocity with neglible angular velocity.
         * 
         * @param vertex_parent the parent vertex.
         * @param vertex_child the child vertex.
         * @return the time to rotate.
         */
        double calculate_time_to_translate(const graph_vertex& vertex_parent, const graph_vertex& vertex_child);

        /**
         * @brief generates a map of occupied vertex indicies by (x, y) coordinate.
         * 
         * this method generates an occupancy map where at a coordinate (x, y), all occupied vertices are listed.
         * 
         * @param graph the graph consiting of vertices, edges, etc.
         * @param x_mesh the two dimensional arma mat consisting of x values of the graph.
         * @param y_mesh the two dimensional arma mat consisting of y values of the graph.
         */
        void generate_occupancies_using_graph_map(const Graph& graph, const arma::mat& x_mesh, const arma::mat& y_mesh);

        /**
         * @brief generates transition costs by edge.
         * 
         * this method pregenerates the cost to transition along all valid edges where the cost is determined using the sum of threats in the occupied space.
         * 
         * @param graph the graph consiting of vertices, edges, etc.
         * @param x_mesh the two dimensional arma mat consisting of x values of the graph.
         * @param y_mesh the two dimensional arma mat consisting of y values of the graph.
         */
        void generate_transition_costs_by_edge(const Graph& graph, const std::vector<double>& threat);

        /**
         * @brief generates transition times by edge.
         * 
         * this method prefenerates the time to transition along all valid edges where the cost is determined using edge type.
         * 
         * @param graph the graph consiting of vertices, edges, etc.
         * @param threat the weight of each (x, y) coordinate.
         */
        void generate_transition_times_by_edge(const Graph& graph);

        /**
         * 
         */
        void generate_transition_heuristic_by_edge(const Graph& graph);

        /**
         * 
         */
        std::vector<int64_t> lookup_occupancies(const double& x, const double& y) const;

        /**
         * @brief looks up the transition cost of an edge.
         * 
         * @param index_edge the index of the edge to lookup.
         * @return the transition cost of the edge.
         */
        double lookup_cost(const double& index_edge) const;

        /**
         * @brief looks up the transition time of an edge.
         * 
         * @param index_edge the index of the edge to lookup.
         * @return the transition time of the edge.
         */
        double lookup_time(const double& index_edge) const;

        /**
         * 
         */
        double lookup_heuristic(const double& index_edge) const;

        /**
         * 
         */
        void set_source(const int64_t& index_source);

        /**
         * 
         */
        void set_target(const int64_t& index_target);

        int64_t index_source_;
        int64_t index_target_;

        /**
         * 
         */
        void save_actor(const std::string& path_actor);

    private:
        double k_ang_;
        double k_lin_;
        double x_tol_ang_;
        double x_tol_lin_;
        double u_max_ang_;
        double u_max_lin_;
        double radius_;

        graph_map occupancies_;
        std::vector<double> costs_;
        std::vector<double> times_;
        std::vector<double> heuristics_;
    };

    /**
     * 
     */
    Actor generate_actor_turtlebot3(const std::string& type, const double& u_ratio, const double& r_ratio);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ACTOR_HPP