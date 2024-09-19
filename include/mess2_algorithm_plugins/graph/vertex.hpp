#ifndef MESS2_ALGORITHM_PLUGINS_VERTEX_HPP
#define MESS2_ALGORITHM_PLUGINS_VERTEX_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    class Vertex
    {
    public:
        Vertex(const double& x, const double&y, const double& theta);

        double get_x_() const;
        double get_y_() const;
        double get_theta_() const;

    private:
        double x_;
        double y_;
        double theta_;
    };

    std::vector<Vertex> generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_VERTEX_HPP
