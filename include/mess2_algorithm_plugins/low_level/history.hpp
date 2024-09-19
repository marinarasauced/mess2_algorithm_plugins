#ifndef MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HISTORY_HPP
#define MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HISTORY_HPP

#include "mess2_algorithm_plugins/common.hpp"

namespace mess2_algorithms
{
    struct low_level_history {
        double score;
        double time;
        int64_t index_parent;
        int64_t index_history;
    };

    class LowLevelHistory
    {
    public:
        LowLevelHistory();

        void history_append(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history);
        void history_clear();
        low_level_history history_lookup(const int64_t& index_history);
    
    private:
        std::vector<low_level_history> history_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HISTORY_HPP
