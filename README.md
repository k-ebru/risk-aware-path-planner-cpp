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

**alpha = 0.0** (shortest path, ignores risk):
```
Risk-Aware Path Planner
Grid: 20x20  Start: (1,1)  Goal: (18,18)  Alpha: 0

  A* Path
  01234567890123456789
0 ....................
1 .S.....*~~~~~~~~~~~~
2 .......#*~~~........
3 .......#.*~~..###...
4 .......#..*~..###...
5 .......#...*..###...
6 .......#....*......
7 .......#.....*......
8 .......#......*.....
9 .......#.......*....
0 .......#....########
1 ................*...
2 .................*..
3 ..................*.
4 ...................*
5 ...................* 
6 ...................* 
7 ...................*
8 ..................G.
9 ....................
  A* cost: 34  |  path length: 18 cells
```

**alpha = 5.0** (strongly risk-averse):
```
The planner routes around all high-risk zones (~), producing
a longer but safer path. The cost difference between A* and
RRT illustrates optimality vs sampling trade-offs.
```

See `examples/sample_output.txt` for full runs at alpha = 0, 1, and 5.

## Design

- **Grid**: 20x20 with static obstacles and spatially varying risk costs (0.0–1.0)
- **A\***: weighted cost function `f = g + h`, where `g` includes `alpha * cell_risk`
- **RRT**: sampling-based planner with risk-weighted edge cost and goal-biased sampling
- **alpha** parameter controls the trade-off between path length and cumulative risk exposure

### Cost Function

```
g(n) = g(parent) + 1.0 + alpha * risk(n)
h(n) = Euclidean distance to goal
f(n) = g(n) + h(n)
```

When `alpha = 0`, A\* reduces to standard shortest-path search. As `alpha` increases, the planner increasingly avoids high-risk cells, producing longer but safer routes.

### Architecture

```
src/
├── grid.h/cpp    — Grid representation, obstacle/risk maps, ASCII renderer
├── astar.h/cpp   — A* search with risk-weighted cost
├── rrt.h/cpp     — RRT with collision checking and risk-weighted edges
└── main.cpp      — Environment setup, CLI argument parsing, comparison
```

## License

MIT
