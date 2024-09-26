
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/constraint.hpp"

namespace mess2_algorithms
{
    ConstraintsVertices::ConstraintsVertices(const int64_t& n_vertices)
    {
        if (n_vertices <= 0) {
            throw std::runtime_error("number of vertices must be greater than zero");
        }

        constraints_.resize(n_vertices);
    };

    void ConstraintsVertices::constrain_vertex(const int64_t& index_vertex, const double& t_init, const double& t_term)
    {
        const constraint_vertex element = {t_init, t_term};
        constraints_[index_vertex].push_back(element);
    }

    constraints_vertex ConstraintsVertices::lookup_vertex(const int64_t& index_vertex) const
    {
        return constraints_[index_vertex];
    }

    void ConstraintsVertices::info()
    {
        throw std::runtime_error("constraint info not implemented");
    }

} // namespace mess2_algorithms
