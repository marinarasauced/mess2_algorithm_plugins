
#include "mess2_algorithm_plugins/conflict.hpp"

namespace mess2_algorithms
{
    bool operator < (const Conflict &_conflict1, const Conflict &_conflict2)
    {
        if (_conflict1.priority_primary == _conflict2.priority_primary)
        {
            if (_conflict1.type == _conflict2.type)
            {
                if (_conflict1.priority_secondary == _conflict2.priority_secondary)
                {
                    return std::rand() % 2;
                }
                return _conflict1.priority_secondary > _conflict2.priority_secondary;
            }
            return _conflict1.type > _conflict2.type;
        }
        return _conflict1.priority_primary > _conflict2.priority_primary;
    }

    bool operator == (const Conflict &_conflict1, const Conflict &_conflict2)
    {
        return ((
            _conflict1.index_actor1 == _conflict2.index_actor1 &&
            _conflict1.index_actor2 == _conflict2.index_actor2 &&
            _conflict1.constraint1 == _conflict2.constraint1 &&
            _conflict1.constraint2 == _conflict2.constraint2
        ) || (
            _conflict1.index_actor1 == _conflict2.index_actor2 &&
            _conflict1.index_actor2 == _conflict2.index_actor1 &&
            _conflict1.constraint1 == _conflict2.constraint2 &&
            _conflict1.constraint2 == _conflict2.constraint1
        ));
    }

    bool operator != (const Conflict &_conflict1, const Conflict &_conflict2)
    {
        return !(_conflict1 == _conflict2);
    }

} // namespace mess2_algorithms
