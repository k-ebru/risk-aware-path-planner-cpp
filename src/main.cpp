#include "grid.h"
#include "astar.h"
#include "rrt.h"
#include "smoothing.h"
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>

namespace {

void build_environment(Grid& grid) {
    // Vertical wall with gap (gap at y=14..15)
    for (int y = 2; y < 14; ++y)
        grid.setObstacle(7, y);
    // Horizontal wall
    for (int x = 12; x < 18; ++x)
        grid.setObstacle(x, 10);
    // Small block
    for (int y = 3; y < 6; ++y)
        for (int x = 14; x < 17; ++x)
            grid.setObstacle(x, y);

    // High-risk zone 1: upper corridor (narrow passage risk)
    for (int y = 0; y < 5; ++y)
        for (int x = 8; x < 12; ++x)
            grid.setRisk(x, y, 0.8);

    // High-risk zone 2: central area
    for (int y = 7; y < 12; ++y)
        for (int x = 3; x < 7; ++x)
            grid.setRisk(x, y, 0.9);

    // High-risk zone 3: lower-right passage
    for (int y = 14; y < 18; ++y)
        for (int x = 10; x < 15; ++x)
            grid.setRisk(x, y, 0.7);

    // Moderate risk band
    for (int y = 12; y < 15; ++y)
        for (int x = 7; x < 10; ++x)
            grid.setRisk(x, y, 0.4);
}

double path_euclidean_length(const std::vector<std::pair<int,int>>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

} // namespace

int main(int argc, char* argv[]) {
    double alpha = 1.0;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--alpha") == 0 && i + 1 < argc) {
            alpha = std::stod(argv[i + 1]);
            ++i;
        } else if (std::strcmp(argv[i], "--help") == 0) {
            std::cout << "Usage: planner [--alpha <value>]\n"
                      << "  --alpha  Risk weight (default: 1.0). "
                      << "0 = shortest path, higher = more risk-averse\n";
            return 0;
        }
    }

    constexpr int grid_size = 20;
    Grid grid(grid_size, grid_size);
    build_environment(grid);

    std::pair<int,int> start = {1, 1};
    std::pair<int,int> goal  = {18, 18};

    std::cout << "Risk-Aware Path Planner\n"
              << "Grid: " << grid_size << "x" << grid_size
              << "  Start: (" << start.first << "," << start.second << ")"
              << "  Goal: (" << goal.first << "," << goal.second << ")"
              << "  Alpha: " << alpha << "\n";

    // --- A* search ---
    AStarResult astar_result = astar_search(grid, start, goal, alpha);
    if (astar_result.found) {
        grid.printGrid(astar_result.path, start, goal, "A* Raw Path");
        std::cout << "  A* cost: " << astar_result.total_cost
                  << "  |  path length: " << astar_result.path.size() << " cells\n";

        // Smooth the A* path
        SmoothedPath smoothed = smooth_path(astar_result.path, grid);
        grid.printGrid(smoothed.path, start, goal, "A* Smoothed Path");
        std::cout << "  Smoothed: " << smoothed.original_length
                  << " -> " << smoothed.smoothed_length << " waypoints"
                  << "  |  euclidean length: " << path_euclidean_length(smoothed.path) << "\n";
    } else {
        std::cout << "  A*: no path found\n";
    }

    // --- RRT search ---
    RRTResult rrt_result = rrt_search(grid, start, goal, alpha);
    if (rrt_result.found) {
        grid.printGrid(rrt_result.path, start, goal, "RRT Path");
        std::cout << "  RRT cost: " << rrt_result.total_cost
                  << "  |  path length: " << rrt_result.path.size() << " cells"
                  << "  |  iterations: " << rrt_result.iterations_used << "\n";
    } else {
        std::cout << "  RRT: no path found after " << rrt_result.iterations_used
                  << " iterations\n";
    }

    // --- Comparison ---
    if (astar_result.found && rrt_result.found) {
        std::cout << "\nComparison (alpha=" << alpha << "):\n"
                  << "  A* cost:  " << astar_result.total_cost << "\n"
                  << "  RRT cost: " << rrt_result.total_cost << "\n"
                  << "  A* is " << (astar_result.total_cost <= rrt_result.total_cost
                                    ? "optimal" : "suboptimal")
                  << " for this configuration.\n";
    }

    return 0;
}
