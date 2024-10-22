
#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    bool are_paths_equal(const Path &_path1, const Path &_path2)
    {
        if (_path1.size() != _path2.size()) {
            return false;
        }
        for (auto iter = 0; iter < _path1.size(); ++iter) {
            if (_path1[iter].index_vertex != _path2[iter].index_vertex || _path1[iter].time != _path2[iter].time) {
                return false;
            }
        }
        return true;
    }

} // namespace mess2_algorithms
