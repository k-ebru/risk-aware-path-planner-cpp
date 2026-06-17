#include "grid.h"
#include <iostream>
#include <algorithm>
#include <set>

Grid::Grid(int width, int height)
    : width_(width), height_(height),
      risk_map_(height, std::vector<double>(width, 0.0)),
      obstacle_map_(height, std::vector<bool>(width, false)) {}

void Grid::setObstacle(int x, int y) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_)
        obstacle_map_[y][x] = true;
}

void Grid::setRisk(int x, int y, double risk) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_)
        risk_map_[y][x] = std::clamp(risk, 0.0, 1.0);
}

bool Grid::isValid(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_ && !obstacle_map_[y][x];
}

bool Grid::isObstacle(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_ && obstacle_map_[y][x];
}

double Grid::getRisk(int x, int y) const {
    if (x >= 0 && x < width_ && y >= 0 && y < height_)
        return risk_map_[y][x];
    return 1.0;
}

int Grid::getWidth() const { return width_; }
int Grid::getHeight() const { return height_; }

bool Grid::checkLineCollision(double x1, double y1, double x2, double y2, int steps) const {
    double dx = (x2 - x1) / steps;
    double dy = (y2 - y1) / steps;

    for (int i = 0; i <= steps; ++i) {
        int ix = static_cast<int>(x1 + dx * i);
        int iy = static_cast<int>(y1 + dy * i);
        if (ix < 0 || ix >= width_ || iy < 0 || iy >= height_)
            return true;
        if (obstacle_map_[iy][ix])
            return true;
    }
    return false;
}

void Grid::printGrid(const std::vector<std::pair<int,int>>& path,
                     std::pair<int,int> start,
                     std::pair<int,int> goal,
                     const std::string& label) const {
    std::set<std::pair<int,int>> path_set(path.begin(), path.end());

    std::cout << "\n  " << label << "\n";
    std::cout << "  ";
    for (int x = 0; x < width_; ++x)
        std::cout << (x % 10);
    std::cout << "\n";

    for (int y = 0; y < height_; ++y) {
        std::cout << (y % 10) << " ";
        for (int x = 0; x < width_; ++x) {
            if (x == start.first && y == start.second) {
                std::cout << 'S';
            } else if (x == goal.first && y == goal.second) {
                std::cout << 'G';
            } else if (obstacle_map_[y][x]) {
                std::cout << '#';
            } else if (path_set.count({x, y})) {
                std::cout << '*';
            } else if (risk_map_[y][x] >= 0.7) {
                std::cout << '~';
            } else {
                std::cout << '.';
            }
        }
        std::cout << "\n";
    }
}
