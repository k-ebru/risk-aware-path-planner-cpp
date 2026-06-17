#!/usr/bin/env python3
"""
Generate grid visualizations for different alpha values.
Usage: python scripts/visualize.py
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import heapq
import math
import os

GRID_W, GRID_H = 30, 30
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

def build_environment():
    """Reconstruct the grid environment (must match main.cpp)."""
    obstacles = np.zeros((GRID_H, GRID_W), dtype=bool)
    risk = np.zeros((GRID_H, GRID_W))

    for y in range(GRID_H):
        is_gap = y in (4, 5, 14, 15, 25, 26)
        if not is_gap:
            obstacles[y][15] = True

    for y in range(1, 8):
        for x in range(10, 21):
            risk[y][x] = 0.9

    for y in range(11, 19):
        for x in range(10, 21):
            risk[y][x] = 0.5

    return obstacles, risk

def astar(obstacles, risk, start, goal, alpha):
    """Simple A* matching the C++ implementation."""
    dx = [1, 0, -1, 0, 1, 1, -1, -1]
    dy = [0, 1, 0, -1, 1, -1, 1, -1]
    mc = [1.0, 1.0, 1.0, 1.0, math.sqrt(2), math.sqrt(2), math.sqrt(2), math.sqrt(2)]

    open_set = [(0.0, start)]
    g_cost = {start: 0.0}
    came_from = {}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))

        cx, cy = current
        for i in range(8):
            nx, ny = cx + dx[i], cy + dy[i]
            if nx < 0 or nx >= GRID_W or ny < 0 or ny >= GRID_H:
                continue
            if obstacles[ny][nx]:
                continue
            step = mc[i] + alpha * risk[ny][nx]
            new_g = g_cost[current] + step
            if (nx, ny) not in g_cost or new_g < g_cost[(nx, ny)]:
                g_cost[(nx, ny)] = new_g
                h = math.sqrt((nx - goal[0])**2 + (ny - goal[1])**2)
                heapq.heappush(open_set, (new_g + h, (nx, ny)))
                came_from[(nx, ny)] = current
    return []

def create_comparison_figure(alphas=[0, 3, 8]):
    """Create a side-by-side comparison of paths for different alpha values."""
    obstacles, risk = build_environment()
    start = (2, 4)
    goal = (28, 4)

    fig, axes = plt.subplots(1, 3, figsize=(18, 7))
    titles = [
        r"$\alpha$ = 0" + "\nShortest Path (ignores risk)",
        r"$\alpha$ = 3" + "\nBalanced (avoids high risk)",
        r"$\alpha$ = 8" + "\nSafest Route (zero risk)"
    ]

    for idx, (alpha, ax, title) in enumerate(zip(alphas, axes, titles)):
        grid_img = np.ones((GRID_H, GRID_W, 3))
        for y in range(GRID_H):
            for x in range(GRID_W):
                if obstacles[y][x]:
                    grid_img[y][x] = [0.3, 0.3, 0.3]
                elif risk[y][x] >= 0.7:
                    grid_img[y][x] = [1.0, 0.78, 0.78]
                elif risk[y][x] >= 0.3:
                    grid_img[y][x] = [1.0, 0.94, 0.75]

        ax.imshow(grid_img, interpolation='nearest', aspect='equal')

        path = astar(obstacles, risk, start, goal, alpha)
        if path:
            px = [p[0] for p in path]
            py = [p[1] for p in path]
            ax.plot(px, py, '-', color='#2060cc', linewidth=2.8, alpha=0.85, zorder=3)

        ax.plot(start[0], start[1], 'o', color='#22aa22', markersize=13, zorder=5,
                markeredgecolor='white', markeredgewidth=1.5)
        ax.plot(goal[0], goal[1], '^', color='#dd2222', markersize=13, zorder=5,
                markeredgecolor='white', markeredgewidth=1.5)

        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.set_xlim(-0.5, GRID_W - 0.5)
        ax.set_ylim(GRID_H - 0.5, -0.5)
        ax.set_xticks(range(0, GRID_W, 5))
        ax.set_yticks(range(0, GRID_H, 5))
        ax.grid(True, alpha=0.15)
        ax.set_xlabel('x')
        ax.set_ylabel('y')

    legend_elements = [
        mpatches.Patch(facecolor=[1.0, 0.78, 0.78], edgecolor='gray', label='High Risk (0.9)'),
        mpatches.Patch(facecolor=[1.0, 0.94, 0.75], edgecolor='gray', label='Medium Risk (0.5)'),
        mpatches.Patch(facecolor=[0.3, 0.3, 0.3], label='Wall'),
        plt.Line2D([0], [0], color='#2060cc', linewidth=2.5, label='A* Path'),
        plt.Line2D([0], [0], marker='o', color='#22aa22', linestyle='None', markersize=9, label='Start'),
        plt.Line2D([0], [0], marker='^', color='#dd2222', linestyle='None', markersize=9, label='Goal'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=6, fontsize=10,
               bbox_to_anchor=(0.5, -0.01))

    plt.suptitle('Risk-Aware Path Planning: Effect of Alpha Parameter',
                 fontsize=15, fontweight='bold', y=1.01)
    plt.tight_layout()

    out_path = os.path.join(PROJECT_DIR, "docs", "alpha_comparison.png")
    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"Saved: {out_path}")
    plt.close()

def create_environment_figure():
    """Create a figure showing the environment layout."""
    obstacles, risk = build_environment()
    start = (2, 4)
    goal = (28, 4)

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    grid_img = np.ones((GRID_H, GRID_W, 3))
    for y in range(GRID_H):
        for x in range(GRID_W):
            if obstacles[y][x]:
                grid_img[y][x] = [0.3, 0.3, 0.3]
            elif risk[y][x] >= 0.7:
                grid_img[y][x] = [1.0, 0.73, 0.73]
            elif risk[y][x] >= 0.3:
                grid_img[y][x] = [1.0, 0.92, 0.72]

    ax.imshow(grid_img, interpolation='nearest', aspect='equal')
    ax.plot(start[0], start[1], 'o', color='#22aa22', markersize=15, zorder=5,
            markeredgecolor='white', markeredgewidth=2)
    ax.plot(goal[0], goal[1], '^', color='#dd2222', markersize=15, zorder=5,
            markeredgecolor='white', markeredgewidth=2)

    ax.annotate('Gap A\n(HIGH risk)', xy=(15, 4.5), fontsize=9, fontweight='bold',
                ha='center', va='center', color='darkred',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.85))
    ax.annotate('Gap B\n(MED risk)', xy=(15, 14.5), fontsize=9, fontweight='bold',
                ha='center', va='center', color='#cc6600',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.85))
    ax.annotate('Gap C\n(SAFE)', xy=(15, 25.5), fontsize=9, fontweight='bold',
                ha='center', va='center', color='green',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.85))

    ax.annotate('Start', xy=(start[0], start[1]), xytext=(start[0], start[1] - 2),
                fontsize=10, fontweight='bold', ha='center', color='#22aa22',
                arrowprops=dict(arrowstyle='->', color='#22aa22', lw=1.5))
    ax.annotate('Goal', xy=(goal[0], goal[1]), xytext=(goal[0], goal[1] - 2),
                fontsize=10, fontweight='bold', ha='center', color='#dd2222',
                arrowprops=dict(arrowstyle='->', color='#dd2222', lw=1.5))

    ax.set_title('Environment: 30x30 Grid with Wall and Risk Zones',
                 fontsize=14, fontweight='bold')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_xticks(range(0, GRID_W, 5))
    ax.set_yticks(range(0, GRID_H, 5))
    ax.grid(True, alpha=0.15)

    legend_elements = [
        mpatches.Patch(facecolor=[1.0, 0.73, 0.73], edgecolor='gray', label='High Risk (0.9)'),
        mpatches.Patch(facecolor=[1.0, 0.92, 0.72], edgecolor='gray', label='Medium Risk (0.5)'),
        mpatches.Patch(facecolor=[0.3, 0.3, 0.3], label='Wall / Obstacle'),
        mpatches.Patch(facecolor='white', edgecolor='gray', label='Free Space'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9)

    out_path = os.path.join(PROJECT_DIR, "docs", "environment.png")
    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
    print(f"Saved: {out_path}")
    plt.close()

if __name__ == "__main__":
    create_environment_figure()
    create_comparison_figure()
    print("Done!")
