#pragma once

#include <vector>
#include <utility>
#include <string>

class Grid {
public:
    Grid(int width, int height);

    void setObstacle(int x, int y);
    void setRisk(int x, int y, double risk);
    bool isValid(int x, int y) const;
    bool isObstacle(int x, int y) const;
    double getRisk(int x, int y) const;
    int getWidth() const;
    int getHeight() const;

    bool checkLineCollision(double x1, double y1, double x2, double y2, int steps = 20) const;
    double lineMaxRisk(double x1, double y1, double x2, double y2, int steps = 20) const;

    void printGrid(const std::vector<std::pair<int,int>>& path,
                   std::pair<int,int> start,
                   std::pair<int,int> goal,
                   const std::string& label) const;

private:
    int width_;
    int height_;
    std::vector<std::vector<double>> risk_map_;
    std::vector<std::vector<bool>> obstacle_map_;
};
