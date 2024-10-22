#ifndef MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
#define MESS2_ALGORITHM_PLUGINS_ACTOR_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph.hpp"

namespace mess2_algorithms
{
    /**
     * @brief computes a list of keys that represent the actor's symbolic occupancies.
     * 
     * this method allows for a quick lookup of an actor's occupancies at any point, vertex, or edge via the key of the point plus the indices of each of the map's keys, respectively. this method is public allowing other libraries to lookup larger occupancy maps for multi-constraint collision prevention.
     * 
     * @param _graph the graph about which the occupancies are computed (assumed to be centered at [0.0, 0.0, 0.0]).
     * @param _radius the radius of which to generate the spherical occupancies.
     */
    std::list<std::shared_ptr<Key3D>> compute_occupancies_symbolically(const std::shared_ptr<Graph> &_graph, const double &_radius);


    /**
     * @brief defines an actor in a three-dimensional grid-based graph with custom dynamics and occupancies.
     * 
     * @param _graph the shared Graph pointer used to construct the actor.
     * @param _index_source the index of the actor's source vertex in the graph's vertices.
     * @param _index_target the index of the actor's target vertex in the graph's vertices.
     * @param _radius the radius of the actor assuming spherical occupancies.
     * @param _k_ang the angular control input gain.
     * @param _k_lin the linear control input gain.
     * @param _x_ang_tol the angular state tolerance during transitions.
     * @param _x_lin_tol the linear state tolerance during transitions.
     * @param _u_ang_max the maximum angular control input.
     * @param _u_lin_max the maximum linear control input.
     * 
     */
    class Actor
    {
    public:
        int index_source; // the index of the actor's source vertex in vertices.
        int index_target; // the index of the actor's target vertex in vertices.
        std::shared_ptr<Vertex> vertex_source; // the shared Vertex pointer of the actor's source.
        std::shared_ptr<Vertex> vertex_target; // the shared Vertex pointer of the actor's target.

        double runtime_build; // the runtime to build the actor.
        // double runtime_build_oat; // the runtime to build the obstacle avoidance table.


        /**
         * @brief constructs an actor instance with mess2 custom ugv dynamics.
         */
        Actor(const std::shared_ptr<Graph> &_graph, int _index_source, int _index_target, double _radius, double _k_ang = 1.0, double _k_lin = 1.0, double _x_ang_tol = 0.01, double _x_lin_tol = 0.01, double _u_ang_max = 1.0, double _u_lin_max = 1.0) : index_source(_index_source), index_target(_index_target), radius(_radius), k_ang(_k_ang), k_lin(_k_lin), x_ang_tol(_x_ang_tol), x_lin_tol(_x_lin_tol), u_ang_max(_u_ang_max), u_lin_max(_u_lin_max)
        {
            auto tic = std::clock();

            vertex_source = _graph->lookup_vertex(index_source);
            vertex_target = _graph->lookup_vertex(index_target);

            occupanices_symbolic = compute_occupancies_symbolically(_graph, radius);
            (void) compute_occupancies_weights_heuristics(_graph);
            (void) compute_times(_graph);
            auto toc = std::clock();
            runtime_build = (toc - tic) / CLOCKS_PER_SEC;

            // runtime_build = (toc1 - tic) / CLOCKS_PER_SEC;
            // runtime_build_oat
        };


        /**
         * @brief looks up a map of occupancies at a given point w.r.t both the graph pointer attribute and the symbolic occupancies map.
         * 
         * @param _graph the graph about which the occupancies are to be looked up.
         * @param _i the x-coordinate index of the point in the graph.
         * @param _j the y-coordinate index of the point in the graph.
         * @param _k the z-coordinate index of the point in the graph.
         * @param _map the map about which the respective occupancies are to be calculated; i.e., occupancies_symbolic or a different map for collision avoidance with a larger radius.
         * @return keys of the true occupied points in the graph.
         */
        std::list<std::shared_ptr<Key3D>> lookup_occupancies_symbolically(const std::shared_ptr<Graph> &_graph, int _i, int _j, int _k, const std::list<std::shared_ptr<Key3D>> &_map);


    private:
        double radius, k_ang, k_lin, x_ang_tol, x_lin_tol, u_ang_max, u_lin_max; // actor attribute for computations

        std::list<std::shared_ptr<Key3D>> occupanices_symbolic; // symbolic map of occupancies via key steps from any given index at i, j, k
        boost::unordered_map<int, std::list<std::shared_ptr<Key3D>>> occupancies_by_index_point; // unordered map of occupancies by index point

        boost::unordered_map<int, double> g_by_index_point; // unordered map of occupied weights by index point
        boost::unordered_map<int, double> h_by_index_point; // unordered map of heuristics by index point
        boost::unordered_map<int, double> t_by_index_edge; // unordered map of transition times by index edge

        
        /**
         * @brief computes the transition time of an edge of type WAIT.
         * 
         * @param _edge a shared pointer to a graph edge of type WAIT.
         * @return the time to wait (hard-coded).
         */
        double compute_time_wait(const std::shared_ptr<Edge> &_edge)
        {
            if (_edge->type != edge_type::WAIT) {
                return 0.0;
            }
            return 5.0;
        }

        /**
         * @brief computes the transition time of an edge of type ROTATE.
         * 
         * @param _edge a shared pointer to a graph edge of type ROTATE.
         * @return the time to rotate.
         */
        double compute_time_rotate(const std::shared_ptr<Edge> &_edge);

        /**
         * @brief computes the transition time of an edge of type TRANSLATE_IN_PLANE or TRANSLATE_OUT_OF_PLANE.
         * 
         * @param _edge a shared pointer to a graph edge of type TRANSLATE_IN_PLANE or TRANSLATE_OUT_OF_PLANE.
         * @return the time to translate.
         */
        double compute_time_translate(const std::shared_ptr<Edge> &_edge);


        /**
         * @brief computes the occupancies, occupied weights, heuristics of the actor at each point.
         * 
         * @param _graph the graph about which the attributes are computed.
         */
        void compute_occupancies_weights_heuristics(const std::shared_ptr<Graph> &_graph);


        /**
         * @brief computes the time to transition for each edge in a graph using the dynamics specified in compute_time_wait, compute_time_rotate, and compute_time_translate.
         * 
         * this method precomputes the time to transition between each of two vertices that comprise an edge for all edges in the graph pointer assigned to the class instance. computed times are stored in the attribute times and are looked up using the index of the edge in question.
         */
        void compute_times(const std::shared_ptr<Graph> &_graph);

    };    

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
