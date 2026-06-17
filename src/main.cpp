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
    // L-shaped wall: vertical segment
    for (int y = 0; y < 10; ++y)
        grid.setObstacle(10, y);
    // L-shaped wall: horizontal segment
    for (int x = 10; x < 16; ++x)
        grid.setObstacle(x, 10);

    // Small block in lower-right
    for (int y = 14; y < 17; ++y)
        for (int x = 14; x < 17; ++x)
            grid.setObstacle(x, y);

    // High-risk zone: the direct southern corridor (the "easy" detour)
    for (int y = 11; y < 16; ++y)
        for (int x = 4; x < 10; ++x)
            grid.setRisk(x, y, 0.9);

    // High-risk zone: upper-right area past the wall
    for (int y = 1; y < 6; ++y)
        for (int x = 11; x < 16; ++x)
            grid.setRisk(x, y, 0.8);

    // Moderate risk near goal approach
    for (int y = 16; y < 19; ++y)
        for (int x = 10; x < 14; ++x)
            grid.setRisk(x, y, 0.5);
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
