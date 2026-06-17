#include "rrt.h"
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>

namespace {

double distance(int x1, int y1, int x2, int y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

double edge_cost(const Grid& grid, int x1, int y1, int x2, int y2, double alpha) {
    double dist = distance(x1, y1, x2, y2);
    double avg_risk = (grid.getRisk(x1, y1) + grid.getRisk(x2, y2)) * 0.5;
    return dist + alpha * avg_risk * dist;
}

std::pair<int,int> steer(int from_x, int from_y, int to_x, int to_y, int step_size) {
    double dx = to_x - from_x;
    double dy = to_y - from_y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist <= step_size)
        return {to_x, to_y};

    double ratio = step_size / dist;
    int new_x = from_x + static_cast<int>(std::round(dx * ratio));
    int new_y = from_y + static_cast<int>(std::round(dy * ratio));
    return {new_x, new_y};
}

bool line_of_sight(const Grid& grid, int x1, int y1, int x2, int y2) {
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    int cx = x1, cy = y1;
    while (true) {
        if (!grid.isValid(cx, cy))
            return false;
        if (cx == x2 && cy == y2)
            break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; cx += sx; }
        if (e2 < dx)  { err += dx; cy += sy; }
    }
    return true;
}

} // namespace

RRTResult rrt_star_search(const Grid& grid,
                          std::pair<int,int> start,
                          std::pair<int,int> goal,
                          double alpha,
                          int max_iterations,
                          double goal_threshold) {
    RRTResult result;
    result.found = false;
    result.total_cost = 0.0;
    result.iterations_used = 0;
    result.tree_size = 0;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist_x(0, grid.getWidth() - 1);
    std::uniform_int_distribution<int> dist_y(0, grid.getHeight() - 1);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    std::vector<RRTNode> tree;
    tree.reserve(max_iterations);
    tree.push_back({start.first, start.second, -1, 0.0});

    constexpr int step_size = 2;
    constexpr double goal_sample_rate = 0.15;
    double rewire_radius = 5.0;

    int best_goal_idx = -1;
    double best_goal_cost = std::numeric_limits<double>::max();

    for (int iter = 0; iter < max_iterations; ++iter) {
        result.iterations_used = iter + 1;

        int sample_x, sample_y;
        if (unit(rng) < goal_sample_rate) {
            sample_x = goal.first;
            sample_y = goal.second;
        } else {
            sample_x = dist_x(rng);
            sample_y = dist_y(rng);
        }

        // Find nearest node by cost-weighted distance
        int nearest_idx = -1;
        double min_score = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
            double d = distance(tree[i].x, tree[i].y, sample_x, sample_y);
            double score = d + 0.3 * tree[i].cost;
            if (score < min_score) {
                min_score = score;
                nearest_idx = i;
            }
        }

        auto [new_x, new_y] = steer(tree[nearest_idx].x, tree[nearest_idx].y,
                                     sample_x, sample_y, step_size);

        if (!grid.isValid(new_x, new_y))
            continue;
        if (!line_of_sight(grid, tree[nearest_idx].x, tree[nearest_idx].y, new_x, new_y))
            continue;

        // Find best parent within rewire radius (RRT* extension)
        int best_parent = nearest_idx;
        double best_cost = tree[nearest_idx].cost +
                           edge_cost(grid, tree[nearest_idx].x, tree[nearest_idx].y,
                                     new_x, new_y, alpha);

        for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
            if (i == nearest_idx) continue;
            double d = distance(tree[i].x, tree[i].y, new_x, new_y);
            if (d > rewire_radius) continue;
            if (!line_of_sight(grid, tree[i].x, tree[i].y, new_x, new_y)) continue;

            double candidate_cost = tree[i].cost +
                                    edge_cost(grid, tree[i].x, tree[i].y, new_x, new_y, alpha);
            if (candidate_cost < best_cost) {
                best_cost = candidate_cost;
                best_parent = i;
            }
        }

        int new_idx = static_cast<int>(tree.size());
        tree.push_back({new_x, new_y, best_parent, best_cost});

        // Rewire existing nodes through the new node
        for (int i = 0; i < new_idx; ++i) {
            double d = distance(tree[i].x, tree[i].y, new_x, new_y);
            if (d > rewire_radius) continue;
            if (!line_of_sight(grid, new_x, new_y, tree[i].x, tree[i].y)) continue;

            double rewired_cost = best_cost +
                                  edge_cost(grid, new_x, new_y, tree[i].x, tree[i].y, alpha);
            if (rewired_cost < tree[i].cost) {
                tree[i].parent_index = new_idx;
                tree[i].cost = rewired_cost;
            }
        }

        // Check if we reached the goal
        if (distance(new_x, new_y, goal.first, goal.second) <= goal_threshold) {
            if (best_cost < best_goal_cost) {
                best_goal_cost = best_cost;
                best_goal_idx = new_idx;
                result.found = true;
            }
        }
    }

    result.tree_size = static_cast<int>(tree.size());

    if (result.found && best_goal_idx >= 0) {
        result.total_cost = best_goal_cost;
        int idx = best_goal_idx;
        while (idx != -1) {
            result.path.push_back({tree[idx].x, tree[idx].y});
            idx = tree[idx].parent_index;
        }
        std::reverse(result.path.begin(), result.path.end());
    }

    return result;
}
