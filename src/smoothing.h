#pragma once

#include "grid.h"
#include <vector>
#include <utility>

struct SmoothedPath {
    std::vector<std::pair<int,int>> path;
    int original_length;
    int smoothed_length;
};

SmoothedPath smooth_path(const std::vector<std::pair<int,int>>& raw_path,
                         const Grid& grid,
                         double alpha = 0.0);
