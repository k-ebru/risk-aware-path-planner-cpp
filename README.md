# Risk-Aware Path Planner (C++)

2D grid-based path planning under risk, implementing A\* and RRT\* with configurable risk weighting. Built as a standalone prototype to explore decision-making trade-offs between path optimality and risk exposure in autonomous navigation scenarios.

## Build

```bash
mkdir build && cd build
cmake .. -G "MinGW Makefiles"   # or just: cmake ..
mingw32-make                    # or: make
```

## Run

```bash
./planner              # default alpha = 1.0
./planner --alpha 0    # pure shortest path (no risk penalty)
./planner --alpha 3    # balanced: avoids high risk
./planner --alpha 8    # strongly risk-averse
./planner --help
```

## How Alpha Controls Route Selection

The environment has a vertical wall with three gaps, each surrounded by a different risk level:

| Alpha | Route | Cost | Risk | Behavior |
|-------|-------|------|------|----------|
| 0 | Gap A (y=4-5) | 26.00 | 9.90 | Shortest path through high-risk zone |
| 3 | Gap B (y=14-15) | 44.78 | 3.50 | Balanced — avoids high risk, accepts medium |
| 8 | Gap C (y=25-26) | 52.77 | 0.00 | Long detour through safe corridor |

As alpha increases, the planner trades path length for safety.

## Design

- **Grid**: 30x30 with a vertical wall barrier and three crossing gaps at different risk levels
- **A\***: 8-connected search with cost `f = g + h`, where `g = move_cost + alpha * cell_risk`
- **RRT\***: sampling-based planner with cost-aware nearest selection, rewiring, and goal-biased sampling
- **Path Smoothing**: shortcut smoothing with risk-aware line-of-sight checks — rejects shortcuts through higher-risk zones
- **alpha** parameter controls the trade-off between path length and cumulative risk exposure

### Cost Function

```
g(n) = g(parent) + move_cost(n) + alpha * risk(n)
h(n) = Euclidean distance to goal
f(n) = g(n) + h(n)

move_cost = 1.0 (cardinal) or sqrt(2) (diagonal)
risk(n)   = cell risk value in [0.0, 1.0]
```

### Architecture

```
src/
├── grid.h/cpp       — Grid representation, obstacle/risk maps, collision & risk line checks
├── astar.h/cpp      — A* search with 8-connected movement and risk-weighted cost
├── rrt.h/cpp        — RRT* with rewiring, cost-aware nearest, Bresenham collision checking
├── smoothing.h/cpp  — Risk-aware shortcut path smoothing
└── main.cpp         — Environment setup, CLI argument parsing, algorithm comparison
```

## License

MIT
