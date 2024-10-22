#ifndef MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
#define MESS2_ALGORITHM_PLUGINS_ACTOR_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph.hpp"

namespace mess2_algorithms
{
    /**
     * 
     */
    class Actor
    {
    public:
        int index_source; // the index of the actor's source vertex in vertices.
        int index_target; // the index of the actor's target vertex in vertices.

        double runtime_build; // the runtime to build the actor.
        double runtime_build_oat; // the runtime to build the obstacle avoidance table.


        /**
         * @brief constructs an actor instance with mess2 custom ugv dynamics.
         */
        // Actor(const std::shared_ptr<Graph> &_graph) {}; , int _index_source, int _index_target double _radius, double _k_ang = 1.0, double _k_lin = 1.0, double _x_ang_tol = 0.01, double _x_lin_tol = 0.01, double _u_ang_max = 1.0, double _u_lin_max = 1.0) : index_source(_index_source), index_target(_index_target
        Actor(const std::shared_ptr<Graph> &_graph, int _index_source, int _index_target, double _radius, double _k_ang = 1.0, double _k_lin = 1.0, double _x_ang_tol = 0.01, double _x_lin_tol = 0.01, double _u_ang_max = 1.0, double _u_lin_max = 1.0) : index_source(_index_source), index_target(_index_target), radius(_radius), k_ang(_k_ang), k_lin(_k_lin), x_ang_tol(_x_ang_tol), x_lin_tol(_x_lin_tol), u_ang_max(_u_ang_max), u_lin_max(_u_lin_max)
        {
            auto tic = std::clock();

            occupanices_symbolic = compute_occupancies_symbolically(_graph, radius);

            

            auto toc = std::clock();
            runtime_build = (toc - tic) / CLOCKS_PER_SEC;
        };


    // private:
        double radius, k_ang, k_lin, x_ang_tol, x_lin_tol, u_ang_max, u_lin_max; // actor attribute for computations

        std::list<std::shared_ptr<Key3D>> occupanices_symbolic; // symbolic map of occupancies via key steps from any given index at i, j, k

        /**
         * 
         */

    };


    /**
     * 
     */
    std::list<std::shared_ptr<Key3D>> compute_occupancies_symbolically(const std::shared_ptr<Graph> &_graph, const double &_radius);
    

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_ACTOR_HPP
