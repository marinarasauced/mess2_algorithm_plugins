#ifndef MESS2_ALGORITHM_PLUGINS_MAP_HPP
#define MESS2_ALGORITHM_PLUGINS_MAP_HPP

#include <vector>

namespace mess2_algorithms {

    class Map
    {
    public:
        Map();
        ~Map();

        bool update_threat(std::vector<std::vector<double>> threat);
        double get_threat(int i, int j) const;

        bool are_indices_valid(int i, int j) const;

        std::vector<std::vector<double>> threat;
        unsigned int height, width;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_MAP_HPP