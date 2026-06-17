#include "grid.h"
#include "astar.h"
#include "rrt.h"
#include "smoothing.h"
#include <iostream>
#include <string>
#include <cstring>
#include <cmath>
#include <iomanip>

namespace {

double path_euclidean_length(const std::vector<std::pair<int,int>>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].first - path[i-1].first;
        double dy = path[i].second - path[i-1].second;
        len += std::sqrt(dx * dx + dy * dy);
    }
    return len;
}

double path_total_risk(const std::vector<std::pair<int,int>>& path, const Grid& grid) {
    double total = 0.0;
    for (auto& [x, y] : path)
        total += grid.getRisk(x, y);
    return total;
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

    constexpr int W = 30;
    constexpr int H = 30;
    Grid grid(W, H);

    // === Vertical wall at x=15 with 3 gaps ===
    // Gap A (y=4,5):   closest to start/goal — HIGH RISK passage
    // Gap B (y=14,15): medium distance — MEDIUM RISK passage
    // Gap C (y=25,26): farthest — SAFE passage
    for (int y = 0; y < H; ++y) {
        bool is_gap = (y == 4 || y == 5)
                   || (y == 14 || y == 15)
                   || (y == 25 || y == 26);
        if (!is_gap)
            grid.setObstacle(15, y);
    }

    // === Risk zones — narrow bands around the wall ===
    // Risk only near the crossing area so vertical travel is safe

    // Gap A crossing zone (y=1..7, x=10..20): HIGH RISK
    for (int y = 1; y <= 7; ++y)
        for (int x = 10; x <= 20; ++x)
            grid.setRisk(x, y, 0.9);

    // Gap B crossing zone (y=11..18, x=10..20): MEDIUM RISK
    for (int y = 11; y <= 18; ++y)
        for (int x = 10; x <= 20; ++x)
            grid.setRisk(x, y, 0.5);

    // Gap C crossing zone (y=22..28): SAFE — no risk
    // default 0.0

    std::pair<int,int> start = {2, 4};
    std::pair<int,int> goal  = {28, 4};

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Risk-Aware Path Planner\n"
              << "Grid: " << W << "x" << H
              << "  Start: (" << start.first << "," << start.second << ")"
              << "  Goal: (" << goal.first << "," << goal.second << ")"
              << "  Alpha: " << alpha << "\n"
              << "Gap A (y=4-5): HIGH risk 0.9  |  "
              << "Gap B (y=14-15): MED risk 0.5  |  "
              << "Gap C (y=25-26): SAFE 0.0\n"
              << "Legend: S=start G=goal #=wall *=path ~=high risk :=medium risk .=free\n";

    // --- A* search ---
    AStarResult astar_result = astar_search(grid, start, goal, alpha);
    if (astar_result.found) {
        grid.printGrid(astar_result.path, start, goal, "A* Path");
        double risk = path_total_risk(astar_result.path, grid);
        std::cout << "  cost: " << astar_result.total_cost
                  << "  |  cells: " << astar_result.path.size()
                  << "  |  risk exposure: " << risk << "\n";

        SmoothedPath smoothed = smooth_path(astar_result.path, grid, alpha);
        grid.printGrid(smoothed.path, start, goal, "A* Smoothed");
        double s_risk = path_total_risk(smoothed.path, grid);
        std::cout << "  waypoints: " << smoothed.original_length
                  << " -> " << smoothed.smoothed_length
                  << "  |  length: " << path_euclidean_length(smoothed.path)
                  << "  |  risk: " << s_risk << "\n";
    } else {
        std::cout << "  A*: no path found\n";
    }

    // --- RRT* search ---
    RRTResult rrt_result = rrt_star_search(grid, start, goal, alpha, 8000, 2.0);
    if (rrt_result.found) {
        grid.printGrid(rrt_result.path, start, goal, "RRT* Path");
        double risk = path_total_risk(rrt_result.path, grid);
        std::cout << "  cost: " << rrt_result.total_cost
                  << "  |  cells: " << rrt_result.path.size()
                  << "  |  risk: " << risk
                  << "  |  tree: " << rrt_result.tree_size
                  << "  |  iters: " << rrt_result.iterations_used << "\n";
    } else {
        std::cout << "  RRT*: no path found (" << rrt_result.iterations_used << " iters)\n";
    }

    // --- Summary ---
    std::cout << "\n--- Summary (alpha=" << alpha << ") ---\n";
    if (astar_result.found)
        std::cout << "  A*   cost: " << astar_result.total_cost
                  << "  |  risk: " << path_total_risk(astar_result.path, grid) << "\n";
    if (rrt_result.found)
        std::cout << "  RRT* cost: " << rrt_result.total_cost
                  << "  |  risk: " << path_total_risk(rrt_result.path, grid) << "\n";

    return 0;
}
