#pragma once

#include "grid.h"
#include <vector>
#include <utility>

struct AStarResult {
    std::vector<std::pair<int,int>> path;
    double total_cost;
    bool found;
};

AStarResult astar_search(const Grid& grid,
                         std::pair<int,int> start,
                         std::pair<int,int> goal,
                         double alpha);
