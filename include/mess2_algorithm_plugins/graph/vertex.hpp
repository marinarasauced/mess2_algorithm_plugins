#ifndef MESS2_ALGORITHM_PLUGINS_VERTEX_HPP
#define MESS2_ALGORITHM_PLUGINS_VERTEX_HPP

#include "mess2_algorithm_plugins/common.hpp"

struct hash_vertices {
    template <typename T1, typename T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
        return hash1 ^ (hash2 << 1);
    }
};

namespace mess2_algorithms
{
    class Vertex
    {
    public:
        Vertex(const double& x, const double&y, const double& theta);

        double get_x() const;
        double get_y() const;
        double get_theta() const;

    private:
        double x_;
        double y_;
        double theta_;
    };

    std::vector<Vertex> generate_vertices(const arma::mat& x_mesh, const arma::mat& y_mesh);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_VERTEX_HPP
