#ifndef MESS2_ALGORITHM_PLUGINS_CONFLICT_HPP
#define MESS2_ALGORITHM_PLUGINS_CONFLICT_HPP

#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines the possible types of conflicts between actors.
     * 
     *  - MUTEX : the conflict involves two mutually exclusive actors.
     * 
     *  - TARGET : the conflict occurs when two actors aim for the same target location.
     * 
     *  - CORRIDOR : the conflict arises in a narrow corridor where two actors cannot pass simultaneously.
     * 
     *  - RECTANGLE : the conflict involves overlapping areas that form a rectangle.
     * 
     *  - STANDARD : the default conflict type, typically indicating a basic conflict scenario.
     * 
     *  - TYPE_COUNT : the total number of conflict types.
     */
    enum conflict_type { MUTEX, TARGET, CORRIDOR, RECTANGLE, STANDARD, TYPE_COUNT };


    /**
     * @brief defines the priority levels of conflicts between actors.
     * 
     *  - CARDINAL : 
     * 
     *  - SEMI : 
     * 
     *  - NON : 
     *
     *  - UNKNOWN : 
     * 
     *  - PRIORITY_COUNT : the total number of priorities.
     */
    enum conflict_priority { CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };


    /**
     * @brief defines the types of constraints applied to actors.
     * 
     *  - LEQLENGTH : actor's path must be less than time length t and cannot be at point after t.
     * 
     *  - GLENGTH : actor's path must be greater than time length t.
     * 
     *  - RANGE : 
     * 
     *  - BARRIER : 
     * 
     *  - POINT : 
     * 
     *  - VERTEX : 
     * 
     *  - EDGE : 
     * 
     *  - POSITIVE_POINT : 
     * 
     *  - POSITIVE_VERTEX : 
     * 
     *  - POSITIVE_EDGE : 
     * 
     *  - POSITIVE_BARRIER : 
     * 
     *  - POSITIVE_RANGE : 
     */
    // enum constraint_type { LEQLENGTH, GLENGTH, RANGE, BARRIER, POINT, VERTEX, EDGE, POSITIVE_POINT, POSITIVE_VERTEX, POSITIVE_EDGE, POSITIVE_BARRIER, POSITIVE_RANGE };
    enum constraint_type { POINT, VERTEX, EDGE, LEQLENGTH, GLENGTH, RANGE, BARRIER, POSITIVE_POINT, POSITIVE_VERTEX, POSITIVE_EDGE, POSITIVE_RANGE, POSITIVE_BARRIER };


    /**
     * @brief defines the structure of a constraint placed upon an actor.
     * 
     * this struct defines constraints and is templated to be applicable to different constraint types as follows : 
     * 
     *  - {index_actor, index_key, t1, t2, tX, POINT}
     * 
     *  - {index_actor, index_vertex, t1, t2, tX, VERTEX}
     * 
     *  - {index_actor, index_edge, t1, t2, tX, EDGE}
     * 
     * etc.
     */
    typedef std::tuple<int, int, double, double, double, constraint_type> Constraint;


    /**
     * 
     */
    class Conflict
    {
    public:
        int index_actor1; // index of the first actor in the conflict
        int index_actor2; // index of the second actor in the conflict
        std::list<Constraint> constraint1;
        std::list<Constraint> constraint2;
        conflict_type type; // the type of conflict
        conflict_priority priority_primary = conflict_priority::UNKNOWN; // the primary priority of resolving the conflict
        double priority_secondary = 0.0; // the secondary priority pf resolving the conflict in case of ties.

        /**
         * @brief constructs a Conflict instance with empty attributes.
         */
        Conflict() : index_actor1(-1), index_actor2(-1), constraint1(), constraint2(), type(conflict_type::STANDARD) {};


        /**
         * @brief defines the current conflict instance as a point conflict.
         * 
         * @param _index_actor1 the index of the first actor in the conflict.
         * @param _index_actor2 the index of the second actor in the conflict.
         * 
         */
        void define_as_point(int &_index_actor1, int &_index_actor2, int &_index_key1, int &_index_key2, double &_t11, double &_t12, const double &_t21, const double &_t22, const double &_radius)
        {
            constraint1.clear();
            constraint2.clear();
            this->index_actor1 = _index_actor1;
            this->index_actor2 = _index_actor2;
            this->constraint1.emplace_back(index_actor1, _index_key2, _t21, _t22, _radius, constraint_type::POINT);
            this->constraint2.emplace_back(index_actor2, _index_key1, _t11, _t12, _radius, constraint_type::POINT);
            type = conflict_type::STANDARD;
        }


        /**
         * @brief defines the current conflict instance as a vertex conflict.
         * 
         * this method may be obselete as two actors cannot occupy different vertices at the same point, so these conflicts may be imposed into point conflicts.
         * 
         * @param _index_actor1 the index of the first actor in the conflict.
         * @param _index_actor2 the index of the second actor in the conflict.
         * @param _index_vertex the index of the vertex at which the conflict takes place.
         * @param _t1 the time at which the conflict begins.
         * @param _t2 the time at which the conflict ends.
         */
        void define_as_vertex(int &_index_actor1, int &_index_actor2, int &_index_vertex, double &_t1, double &_t2, double &_tX)
        {
            constraint1.clear();
            constraint2.clear();
            this->index_actor1 = _index_actor1;
            this->index_actor2 = _index_actor2;
            this->constraint1.emplace_back(index_actor1, _index_vertex, _t1, _t2, _tX, constraint_type::VERTEX);
            this->constraint2.emplace_back(index_actor2, _index_vertex, _t1, _t2, _tX, constraint_type::VERTEX);
            type = conflict_type::STANDARD;
        }


        /**
         * @brief defines the current conflict instance as an edge conflict.
         * 
         * @param _index_actor1 the index of the first actor in the conflict.
         * @param _index_actor2 the index of the second actor in the conflict.
         * @param _index_edge the index of the edge at which the conflict takes place.
         * @param _t1 the time at which the conflict begins.
         * @param _t2 the time at which the conflict ends.
         */
        void define_as_edge(int &_index_actor1, int &_index_actor2, int &_index_edge, double &_t1, double &_t2, double &_tX)
        {
            constraint1.clear();
            constraint2.clear();
            this->index_actor1 = _index_actor1;
            this->index_actor2 = _index_actor2;
            this->constraint1.emplace_back(index_actor1, _index_edge, _t1, _t2, _tX, constraint_type::EDGE);
            this->constraint2.emplace_back(index_actor2, _index_edge, _t1, _t2, _tX, constraint_type::EDGE);
            type = conflict_type::STANDARD;
        }


        /**
         * 
         */
        void define_as_corridor(int &_index_actor1, int &_index_actor2, const std::list<Constraint> &_constraint1, const std::list<Constraint> &_constraint2)
        {
            this->index_actor1 = _index_actor1;
            this->index_actor2 = _index_actor2;
            this->constraint1 = _constraint1;
            this->constraint2 = _constraint2;
            type = conflict_type::CORRIDOR;
        }


        /**
         * 
         */
        bool define_as_rectangle(int &_index_actor1, int &_index_actor2, const std::list<Constraint> &_constraint1, const std::list<Constraint> &_constraint2)
        {
            this->index_actor1 = _index_actor1;
            this->index_actor2 = _index_actor2;
            this->constraint1 = _constraint1;
            this->constraint2 = _constraint2;
            type = conflict_type::RECTANGLE;
            return true;
        }


        /**
         * 
         */
        void define_as_target(int &_index_actor1, int &_index_actor2, int &_index_vertex, double &_t1, double &_t2, double &_tX)
        {
            constraint1.clear();
            constraint2.clear();
            this->index_actor1 = _index_actor1;
            this->index_actor2 = _index_actor2;
            this->constraint1.emplace_back(index_actor1, _index_vertex, _t1, _t2, _tX, constraint_type::LEQLENGTH);
            this->constraint2.emplace_back(index_actor2, _index_vertex, _t1, _t2, _tX, constraint_type::GLENGTH);
            type = conflict_type::TARGET;
        }


        /**
         * 
         */
        void define_as_mutex(int &_index_actor1, int &_index_actor2)
        {
            constraint1.clear();
            constraint2.clear();
            this->index_actor1 = _index_actor1;
            this->index_actor2 = _index_actor2;
            type = conflict_type::MUTEX;
            priority_primary = conflict_priority::CARDINAL;
        }
    };


    /**
     * @brief defines a comparative operator such if one conflict is less than another conflict.
     */
    bool operator < (const Conflict &_conflict1, const Conflict &_conflict2);


    /**
     * @brief defines a comparative operator such if one conflict is equal to another conflict.
     */
    bool operator == (const Conflict &_conflict1, const Conflict &_conflict2);


    /**
     * @brief defines a comparative operator such if one conflict is not equal to another conflict.
     */
    bool operator != (const Conflict &_conflict1, const Conflict &_conflict2);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_CONFLICT_HPP
