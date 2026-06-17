# Risk-Aware Path Planner (C++)

2D grid-based path planning under risk, implementing A\* and RRT with configurable risk weighting. Built as a standalone prototype to explore decision-making trade-offs between path optimality and risk exposure in autonomous navigation scenarios.

## Build

```bash
mkdir build && cd build
cmake ..
make
```

## Run

```bash
./planner              # default alpha = 1.0
./planner --alpha 0.0  # pure shortest path (no risk penalty)
./planner --alpha 5.0  # strongly risk-averse
./planner --help
```

## Example Output

The planner outputs three views per run: A\* raw path, A\* smoothed path, and RRT path. With `alpha = 0`, A\* takes the shortest route through risk zones. With `alpha = 5`, it detours around them.

```
Risk-Aware Path Planner
Grid: 20x20  Start: (1,1)  Goal: (18,18)  Alpha: 1

  A* Raw Path          — full grid search result (8-connected)
  A* Smoothed Path     — shortcut smoothing with line-of-sight checks
  RRT Path             — sampling-based alternative

Comparison (alpha=1):
  A* cost:  38.2
  RRT cost: 44.7
  A* is optimal for this configuration.
```

See `examples/sample_output.txt` for full runs at alpha = 0, 1, and 5.

## Design

- **Grid**: 20x20 with static obstacles (`#`) and spatially varying risk costs 0.0–1.0 (`~` for high-risk)
- **A\***: 8-connected search with weighted cost `f = g + h`, where `g` includes `move_cost + alpha * cell_risk`
- **Path Smoothing**: shortcut smoothing using line-of-sight collision checks to remove unnecessary waypoints
- **RRT**: sampling-based planner with Bresenham collision checking, goal-biased sampling (10%), and risk-weighted edge costs
- **alpha** parameter controls the trade-off between path length and cumulative risk exposure

### Cost Function

```
g(n) = g(parent) + move_cost(n) + alpha * risk(n)
h(n) = Euclidean distance to goal
f(n) = g(n) + h(n)

move_cost = 1.0 (cardinal) or sqrt(2) (diagonal)
```

When `alpha = 0`, A\* reduces to standard shortest-path search. As `alpha` increases, the planner increasingly avoids high-risk cells, producing longer but safer routes.

### Architecture

```
src/
├── grid.h/cpp       — Grid representation, obstacle/risk maps, line collision check, ASCII renderer
├── astar.h/cpp      — A* search with 8-connected movement and risk-weighted cost
├── rrt.h/cpp        — RRT with Bresenham collision checking and risk-weighted edges
├── smoothing.h/cpp  — Shortcut path smoothing with line-of-sight verification
└── main.cpp         — Environment setup, CLI argument parsing, algorithm comparison
```

## License

MIT
