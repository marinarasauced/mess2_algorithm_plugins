#ifndef MESS2_ALGORITHM_PLUGINS_CONSTRAINT_HPP
#define MESS2_ALGORITHM_PLUGINS_CONSTRAINT_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    /**
     * @brief defines a constraint on a vertex such that it is occupied for t ε [t_init, t_term].
     */
    struct constraint_vertex {
        double t_init;
        double t_term;
    };

    /**
     * @brief aliases for vector forms of constraints on a vertex and vertices.
     */
    using constraints_vertex = std::vector<constraint_vertex>;
    using constraints_vertices = std::vector<constraints_vertex>;

    /**
     * @class ConstrainedVertices
     * @brief manages a set of vertices with varying amounts of constraints.
     * 
     * this class provides methods to constrain vertices, look up those constraints by index, and print information about the current constraints.
     */
    class ConstraintsVertices
    {
    public:
        /**
         * @brief constructs a class instance of ConstrainedVertices.
         * 
         * this constructor resizes the constraints attribute to match the number of vertices.
         * 
         * @param n_vertices the number of vertices that can be constrained.
         */
        ConstraintsVertices(const int64_t& n_vertices);

        /**
         * @brief adds a constraint to a vertex.
         * 
         * this method appends the constraints of a vertex with a new constraint.
         * 
         * @param index_vertex the index of the vertex that the constraint is to be appended to.
         * @param t_init the initial time of the constraint.
         * @param t_term the terminal time of the constraint.
         */
        void constrain_vertex(const int64_t& index_vertex, const double& t_init, const double& t_term);

        /**
         * @brief looks up the constraints of a vertex using its index.
         * 
         * this method retrieves the constraints of the vertex associated with the index.
         * 
         * @param index_vertex the index of the vertex to look up.
         * @return a vector of vertex constraints.
         */
        constraints_vertex lookup_vertex(const int64_t& index_vertex) const;

        /**
         * @brief prints informaton about the current class.
         */
        void info();

    private:
        constraints_vertices constraints_;
    };

    // void execute_conflict_search();

    // void execute_conflict_resolution();
}

#endif // MESS2_ALGORITHM_PLUGINS_CONSTRAINT_HPP