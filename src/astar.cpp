#include "astar.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace {

struct AStarNode {
    int x, y;
    double g_cost;
    double f_cost;

    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

double heuristic(int x1, int y1, int x2, int y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

int encode(int x, int y, int width) {
    return y * width + x;
}

} // namespace

AStarResult astar_search(const Grid& grid,
                         std::pair<int,int> start,
                         std::pair<int,int> goal,
                         double alpha) {
    AStarResult result;
    result.found = false;
    result.total_cost = 0.0;

    int w = grid.getWidth();

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
    std::unordered_map<int, double> g_costs;
    std::unordered_map<int, int> came_from;

    constexpr int dx[] = {0, 0, -1, 1};
    constexpr int dy[] = {-1, 1, 0, 0};

    int start_key = encode(start.first, start.second, w);
    g_costs[start_key] = 0.0;

    double h = heuristic(start.first, start.second, goal.first, goal.second);
    open.push({start.first, start.second, 0.0, h});

    while (!open.empty()) {
        AStarNode current = open.top();
        open.pop();

        if (current.x == goal.first && current.y == goal.second) {
            result.found = true;
            result.total_cost = current.g_cost;

            int key = encode(current.x, current.y, w);
            while (key != start_key) {
                int cx = key % w;
                int cy = key / w;
                result.path.push_back({cx, cy});
                key = came_from[key];
            }
            result.path.push_back(start);
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        int current_key = encode(current.x, current.y, w);

        if (current.g_cost > g_costs[current_key])
            continue;

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!grid.isValid(nx, ny))
                continue;

            double step_cost = 1.0 + alpha * grid.getRisk(nx, ny);
            double new_g = current.g_cost + step_cost;
            int neighbor_key = encode(nx, ny, w);

            if (g_costs.find(neighbor_key) == g_costs.end() || new_g < g_costs[neighbor_key]) {
                g_costs[neighbor_key] = new_g;
                came_from[neighbor_key] = current_key;
                double f = new_g + heuristic(nx, ny, goal.first, goal.second);
                open.push({nx, ny, new_g, f});
            }
        }
    }

    return result;
}
