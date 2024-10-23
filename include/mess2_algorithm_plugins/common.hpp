#ifndef MESS2_ALGORITHM_PLUGINS_COMMON_HPP
#define MESS2_ALGORITHM_PLUGINS_COMMON_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include </usr/include/boost/heap/pairing_heap.hpp>
#include </usr/include/boost/unordered_map.hpp>
#include </usr/include/boost/unordered_set.hpp>
#include </usr/include/yaml-cpp/yaml.h>

#define MAX_TIMESTEP std::numeric_limits<double>::max() / 2
#define MAX_COST std::numeric_limits<double>::max() / 2
#define MAX_NODES std::numeric_limits<int>::max() / 2
#define MAX_NUM_STATS 4000

namespace mess2_algorithms
{
    /**
     * @brief defines the structure of a path element consisting of a vertex index and time instance.
     * 
     * @param _index_vertex the index of the vertex of the path element.
     * @param _time the time instance of the path element.
     */
    struct PathElement
    {
        int index_vertex = -1;
        double time = 0.0;
        double score = 0.0;
        int mdd_width = 0;

        bool is_single() const { return mdd_width == 1; }
        PathElement(int _index_vertex = -1, double _time = 0.0, double _score = 0.0) : index_vertex(_index_vertex), time(_time), score(_score) {};
    };


    /**
     * @brief defines the structure of an actor's path.
     * 
     * this type consists of a vector of path elements, each with a vertex index and a time instance. this structure assumes that actor's transition instantaneously once they reach a vertex; i.e., an actor is always transitioning along edges, only stopping at vertices for an instant or once the target vertex is reached.
     */
    typedef std::vector<PathElement> Path;


    /**
     * @brief determines whether two paths are equal.
     * 
     * @param _path1 the first shared path pointer to compare.
     * @param _path2 the second shared path pointer to compare.
     * @return true if the paths are equal, false otherwise.
     */
    bool are_paths_equal(const Path &_path1, const Path &_path2);

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_COMMON_HPP
