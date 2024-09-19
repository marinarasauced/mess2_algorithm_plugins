
#include "mess2_algorithm_plugins/common.hpp"
#include "mess2_algorithm_plugins/graph/vertex.hpp"

namespace mess2_algorithms
{
    Vertex::Vertex(const double& x, const double&y, const double& theta)
    {
        x_ = x;
        y_ = y;
        theta_ = theta;
    }

    double Vertex::get_x_() const {
        return x_;
    }

    double Vertex::get_y_() const {
        return y_;
    }

    double Vertex::get_theta_() const {
        return theta_;
    }

    std::vector<Vertex> generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh)
    {
        std::vector<Vertex> vertices;
        
        int64_t n_rows = x_mesh.n_rows;
        int64_t n_cols = y_mesh.n_cols;

        for (int64_t iter = 0; iter < 8; ++iter) {
            for (int64_t jter = 0; jter < n_rows; ++jter) {
                for (int64_t kter = 0; kter < n_cols; ++kter) {
                    vertices.emplace_back(Vertex(x_mesh(jter, kter), y_mesh(jter, kter), 45 * iter));
                }
            }
        }

        return vertices;
    }

} // namespace mess2_algorithms
