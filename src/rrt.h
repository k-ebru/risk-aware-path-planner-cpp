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
    int tree_size;
};

RRTResult rrt_star_search(const Grid& grid,
                          std::pair<int,int> start,
                          std::pair<int,int> goal,
                          double alpha,
                          int max_iterations = 15000,
                          double goal_threshold = 2.0);
