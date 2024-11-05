#ifndef MESS2_ALGORITHM_PLUGINS_INSTANCE_HPP
#define MESS2_ALGORITHM_PLUGINS_INSTANCE_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph.hpp"
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines an Instance consisting of a shared Graph pointer and a vector of shared Actor pointers.
     * 
     * @param _graph the shared graph pointer of the instance.
     */
    class Instance
    {
    public:
        std::shared_ptr<Graph> graph;
        std::vector<std::shared_ptr<Actor>> actors;
        int n_actors;
        std::vector<int> indices_actors;


        /**
         * @brief constructs an Instance instance from a shared Graph pointer.
         */
        Instance(const std::shared_ptr<Graph> _graph) : graph(_graph), n_actors(0) {};


        /**
         * @brief constructs an Instance instance from a shared Graph pointer and a vector of shared Actor pointers.
         */
        Instance(const std::shared_ptr<Graph> _graph, std::vector<std::shared_ptr<Actor>> _actors) : graph(_graph), actors(_actors), n_actors(static_cast<int>(actors.size())) {};


        /**
         * @brief adds an actor to an Instance's vector of actors.
         * 
         * @param _actor the shared Actor pointer to add to the instance.
         */
        void add_actor(std::shared_ptr<Actor> &_actor) {
            actors.push_back(_actor);
            n_actors += 1;
        }


        /**
         * @brief looks up an actor in actors.
         * 
         * @param _index_actor the index of the actor in actors to lookup.
         * @return the shared Actor pointer of the actor.
         */
        std::shared_ptr<Actor> lookup_actor(int _index_actor) {
            if (_index_actor < 0 || _index_actor >= n_actors) {
                return nullptr;
            }
            return actors[_index_actor];
        }

    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_INSTANCE_HPP
