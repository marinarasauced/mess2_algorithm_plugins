
#include "mess2_algorithm_plugins/actor.hpp"

namespace mess2_algorithms
{
    std::list<std::shared_ptr<Key3D>> compute_occupancies_symbolically(const std::shared_ptr<Graph> &_graph, const double &_radius)
    {
        std::list<std::shared_ptr<Key3D>> occupancies;

        double res_x, res_y, res_z;
        int steps_x, steps_y, steps_z;

        if (_graph->n_i < 2) {
            res_x = 0.0;
            steps_x = 0;
        } else {
            res_x = (_graph->lookup_x(_graph->n_i - 1) - _graph->lookup_x(0));
            steps_x = static_cast<int>(std::ceil(_radius / res_x));
        }
        if (_graph->n_j < 2) {
            res_y = 0.0;
            steps_y = 0;
        } else {
            res_y = (_graph->lookup_y(_graph->n_j - 1) - _graph->lookup_y(0));
            steps_y = static_cast<int>(std::ceil(_radius / res_y));
        }
        if (_graph->n_k < 2) {
            res_z = 0.0;
            steps_z = 0;
        } else {
            res_z = (_graph->lookup_z(_graph->n_k - 1) - _graph->lookup_z(0));
            steps_z = static_cast<int>(std::ceil(_radius / res_z));
        }

        for (auto i = -steps_x; i <= steps_x; ++i) {
            for (auto j = -steps_y; j <= steps_y; ++j) {
                for (auto k = -steps_z; k <= steps_z; ++ k) {
                    const double child_x = i * res_x;
                    const double child_y = j * res_y;
                    const double child_z = k * res_z;
                    
                    const double dxyz = std::sqrt(std::pow(0.0 - child_x, 2) + std::pow(0.0 - child_y, 2) + std::pow(0.0 - child_z, 2));
                    if (dxyz <= _radius) {
                        auto key = std::make_shared<Key3D>();
                        key->index_key = occupancies.size();
                        key->i = i;
                        key->j = j;
                        key->k = k;
                        occupancies.push_back(key);
                    }
                }
            }
        }
        return occupancies;
    }
    

} // namespace mess2_algorithms
