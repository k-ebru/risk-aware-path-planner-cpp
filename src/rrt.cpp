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

int find_nearest(const std::vector<RRTNode>& tree, int x, int y) {
    int nearest = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
        double d = distance(tree[i].x, tree[i].y, x, y);
        if (d < min_dist) {
            min_dist = d;
            nearest = i;
        }
    }
    return nearest;
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

RRTResult rrt_search(const Grid& grid,
                     std::pair<int,int> start,
                     std::pair<int,int> goal,
                     double alpha,
                     int max_iterations,
                     double goal_threshold) {
    RRTResult result;
    result.found = false;
    result.total_cost = 0.0;
    result.iterations_used = 0;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist_x(0, grid.getWidth() - 1);
    std::uniform_int_distribution<int> dist_y(0, grid.getHeight() - 1);
    std::uniform_real_distribution<double> goal_bias(0.0, 1.0);

    std::vector<RRTNode> tree;
    tree.push_back({start.first, start.second, -1, 0.0});

    constexpr int step_size = 2;
    constexpr double goal_sample_rate = 0.1;

    for (int iter = 0; iter < max_iterations; ++iter) {
        result.iterations_used = iter + 1;

        int sample_x, sample_y;
        if (goal_bias(rng) < goal_sample_rate) {
            sample_x = goal.first;
            sample_y = goal.second;
        } else {
            sample_x = dist_x(rng);
            sample_y = dist_y(rng);
        }

        int nearest_idx = find_nearest(tree, sample_x, sample_y);
        const RRTNode& nearest = tree[nearest_idx];

        auto [new_x, new_y] = steer(nearest.x, nearest.y, sample_x, sample_y, step_size);

        if (!grid.isValid(new_x, new_y))
            continue;

        if (!line_of_sight(grid, nearest.x, nearest.y, new_x, new_y))
            continue;

        double step_dist = distance(nearest.x, nearest.y, new_x, new_y);
        double edge_cost = step_dist + alpha * grid.getRisk(new_x, new_y);
        double new_cost = nearest.cost + edge_cost;

        tree.push_back({new_x, new_y, nearest_idx, new_cost});

        if (distance(new_x, new_y, goal.first, goal.second) <= goal_threshold) {
            result.found = true;
            result.total_cost = new_cost;

            int idx = static_cast<int>(tree.size()) - 1;
            while (idx != -1) {
                result.path.push_back({tree[idx].x, tree[idx].y});
                idx = tree[idx].parent_index;
            }
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
    }

    return result;
}
