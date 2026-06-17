#include "smoothing.h"
#include <algorithm>

SmoothedPath smooth_path(const std::vector<std::pair<int,int>>& raw_path,
                         const Grid& grid,
                         double alpha) {
    SmoothedPath result;
    result.original_length = static_cast<int>(raw_path.size());

    if (raw_path.size() < 3) {
        result.path = raw_path;
        result.smoothed_length = result.original_length;
        return result;
    }

    // Compute max risk along the original path segments
    double path_max_risk = 0.0;
    for (auto& [x, y] : raw_path)
        path_max_risk = std::max(path_max_risk, grid.getRisk(x, y));

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
                // Risk-aware check: reject shortcuts through higher-risk zones
                if (alpha > 0.0) {
                    double shortcut_risk = grid.lineMaxRisk(
                        raw_path[current].first, raw_path[current].second,
                        raw_path[farthest].first, raw_path[farthest].second);

                    // Small tolerance (0.05) to avoid rejecting shortcuts due to float rounding
                    if (shortcut_risk > path_max_risk + 0.05) {
                        --farthest;
                        continue;
                    }
                }

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
