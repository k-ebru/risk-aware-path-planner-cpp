#pragma once

#include "grid.h"
#include <vector>
#include <utility>

struct RRTNode {
    int x, y;
    int parent_index;
    double cost;
};

struct RRTResult {
    std::vector<std::pair<int,int>> path;
    double total_cost;
    bool found;
    int iterations_used;
};

RRTResult rrt_search(const Grid& grid,
                     std::pair<int,int> start,
                     std::pair<int,int> goal,
                     double alpha,
                     int max_iterations = 10000,
                     double goal_threshold = 1.5);
