
#include "mess2_algorithm_plugins/map.hpp"

namespace mess2_algorithms
{
    Map::Map()
    {
        height = 0;
        width = 0;
    }

    Map::~Map()
    {
        threat.clear();
    }

    bool Map::update_threat(std::vector<std::vector<double>> threat)
    {
        this->threat = threat;
        this->height = threat.size();
        this->width = threat[0].size();
    }

    double Map::get_threat(int i, int j) const
    {
        if (!are_indices_valid(i, j)) {
            return -1;
        }
        return threat[i][j];
    }

    bool Map::are_indices_valid(int i, int j) const
    {
        return (i < height && i >= 0 && j < width && j >= 0);
    }

} // namespace mess2_algorithms
