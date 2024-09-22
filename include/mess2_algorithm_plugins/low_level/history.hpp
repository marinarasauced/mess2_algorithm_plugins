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
        std::string type;
    };

    class LowLevelHistory
    {
    public:
        LowLevelHistory();

        void append_history(const double& score, const double& time, const int64_t& index_parent, const int64_t& index_history, const std::string& type);
        void clear_history();
        low_level_history lookup_history(const int64_t& index_history);
        int64_t size_history();
    
    private:
        std::vector<low_level_history> history_;
    };

} // namespace mess2_algorithms

#endif // MESS2_ALGORITHM_PLUGINS_LOW_LEVEL_HISTORY_HPP
