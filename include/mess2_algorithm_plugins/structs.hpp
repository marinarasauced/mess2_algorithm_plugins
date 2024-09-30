#ifndef MESS2_ALGORITHM_PLUGINS_STRUCTS_HPP
#define MESS2_ALGORITHM_PLUGINS_STRUCTS_HPP

#include <string>

namespace mess2_algorithms {

    struct Point2D
    {
        double x;
        double y;
    };

    struct Vertex2D
    {
        Point2D* position;
        double rotation;
    };

    struct Edge
    {
        Vertex2D* source;
        Vertex2D* target;
        std::string type;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_STRUCTS_HPP