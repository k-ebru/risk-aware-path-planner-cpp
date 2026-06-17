#include "smoothing.h"

SmoothedPath smooth_path(const std::vector<std::pair<int,int>>& raw_path,
                         const Grid& grid) {
    SmoothedPath result;
    result.original_length = static_cast<int>(raw_path.size());

    if (raw_path.size() < 3) {
        result.path = raw_path;
        result.smoothed_length = result.original_length;
        return result;
    }

    result.path.push_back(raw_path.front());
    int current = 0;

    while (current < static_cast<int>(raw_path.size()) - 1) {
        int farthest = static_cast<int>(raw_path.size()) - 1;
        bool found_shortcut = false;

        while (farthest > current + 1) {
            bool collision = grid.checkLineCollision(
                raw_path[current].first, raw_path[current].second,
                raw_path[farthest].first, raw_path[farthest].second);

            if (!collision) {
                result.path.push_back(raw_path[farthest]);
                current = farthest;
                found_shortcut = true;
                break;
            }
            --farthest;
        }

        if (!found_shortcut) {
            ++current;
            result.path.push_back(raw_path[current]);
        }
    }

    result.smoothed_length = static_cast<int>(result.path.size());
    return result;
}
