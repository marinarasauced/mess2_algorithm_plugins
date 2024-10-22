
#include "mess2_algorithm_plugins/cbs/node.hpp"

namespace mess2_algorithms
{
    void CBSNode::clear()
    {
        conflicts_known.clear();
        conflicts_unknown.clear();
    }

} // namespace mess2_algorithms
