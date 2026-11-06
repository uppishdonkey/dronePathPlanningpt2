
import csv
import heapq
import math
import time
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ==========================================
# CONFIGURATION
# ==========================================

MAP_SIZE = 10.0
RESOLUTION = 0.25
GRID_SIZE = int(MAP_SIZE / RESOLUTION)

START = (-4.0, -4.0)
GOAL = (4.0, 4.0)

OBSTACLE_NAMES = [
    "/Cylinder1",
    "/Cylinder2",
    "/Cylinder3"
]

OBSTACLE_RADIUS = 0.4
DRONE_RADIUS = 0.15
SAFETY_MARGIN = 0.35

# Used by the planner
PLANNING_RADIUS = OBSTACLE_RADIUS + SAFETY_MARGIN

# Used to detect real collisions
COLLISION_RADIUS = OBSTACLE_RADIUS + DRONE_RADIUS

NOISE_LEVELS = [0.0, 0.1, 0.2, 0.3, 0.5]
TRIALS_PER_LEVEL = 100
RANDOM_SEED = 42

OUTPUT = Path(__file__).resolve().parent / "results"
OUTPUT.mkdir(exist_ok=True)

rng = np.random.default_rng(RANDOM_SEED)

# ==========================================
# CONNECT TO COPPELIASIM
# ==========================================

client = RemoteAPIClient()
sim = client.require("sim")

true_obstacles = []

for name in OBSTACLE_NAMES:
    handle = sim.getObject(name)
    pos = sim.getObjectPosition(handle, -1)
    true_obstacles.append((pos[0], pos[1]))

true_obstacles = np.array(true_obstacles)

print("True obstacle positions:")
print(true_obstacles)


# ==========================================
# GRID CONVERSIONS
# ==========================================

def world_to_grid(x, y):
    return (
        int(math.floor((x + MAP_SIZE / 2) / RESOLUTION)),
        int(math.floor((y + MAP_SIZE / 2) / RESOLUTION))
    )


def grid_to_world(gx, gy):
    return (
        -MAP_SIZE / 2 + (gx + 0.5) * RESOLUTION,
        -MAP_SIZE / 2 + (gy + 0.5) * RESOLUTION
    )


def build_grid(obstacles):
    grid = np.zeros(
        (GRID_SIZE, GRID_SIZE),
        dtype=np.uint8
    )

    for gy in range(GRID_SIZE):
        for gx in range(GRID_SIZE):
            x, y = grid_to_world(gx, gy)

            for ox, oy in obstacles:
                if math.hypot(x - ox, y - oy) <= PLANNING_RADIUS:
                    grid[gy, gx] = 1
                    break

    return grid


# ==========================================
# A* ALGORITHM
# ==========================================

def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def astar(grid, start, goal):
    height, width = grid.shape

    for cell in (start, goal):
        x, y = cell
        if not (0 <= x < width and 0 <= y < height):
            return None
        if grid[y, x] == 1:
            return None

    frontier = [(heuristic(start, goal), start)]
    cost = {start: 0.0}
    parent = {}

    directions = [
        (-1, 0), (1, 0),
        (0, -1), (0, 1),
        (-1, -1), (-1, 1),
        (1, -1), (1, 1)
    ]

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            path = [current]

            while current in parent:
                current = parent[current]
                path.append(current)

            return path[::-1]

        for dx, dy in directions:
            nx = current[0] + dx
            ny = current[1] + dy

            if not (0 <= nx < width and 0 <= ny < height):
                continue

            if grid[ny, nx] == 1:
                continue

            if dx != 0 and dy != 0:
                if grid[current[1], nx] == 1:
                    continue
                if grid[ny, current[0]] == 1:
                    continue

            neighbor = (nx, ny)

            new_cost = (
                cost[current] + math.hypot(dx, dy)
            )

            if new_cost < cost.get(neighbor, float("inf")):
                cost[neighbor] = new_cost
                parent[neighbor] = current

                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))

    return None


# ==========================================
# PATH METRICS
# ==========================================

def path_length(path):
    if len(path) < 2:
        return 0.0

    return sum(
        math.dist(a, b)
        for a, b in zip(path[:-1], path[1:])
    )


def segment_distance(point, a, b):
    """Minimum distance from a point to a line segment."""
    p = np.array(point, dtype=float)
    a = np.array(a, dtype=float)
    b = np.array(b, dtype=float)

    ab = b - a
    denominator = np.dot(ab, ab)

    if denominator == 0:
        return float(np.linalg.norm(p - a))

    t = np.clip(
        np.dot(p - a, ab) / denominator,
        0.0,
        1.0
    )

    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


def check_collision(path, obstacles):
    """Check the full route against true obstacles."""
    for a, b in zip(path[:-1], path[1:]):
        for obstacle in obstacles:
            if segment_distance(obstacle, a, b) <= COLLISION_RADIUS:
                return True

    return False


# ==========================================
# RUN EXPERIMENTS
# ==========================================

results = []
example_maps = {}

start_cell = world_to_grid(*START)
goal_cell = world_to_grid(*GOAL)

for noise_level in NOISE_LEVELS:
    print(f"\nTesting noise level: {noise_level}")

    for trial in range(TRIALS_PER_LEVEL):

        # Noisy map: true positions + Gaussian error
        noisy_obstacles = true_obstacles + rng.normal(
            loc=0.0,
            scale=noise_level,
            size=true_obstacles.shape
        )

        grid = build_grid(noisy_obstacles)

        start_time = time.perf_counter()

        cells = astar(grid, start_cell, goal_cell)

        planning_time = time.perf_counter() - start_time

        if cells is None:
            results.append({
                "noise": noise_level,
                "trial": trial,
                "success": 0,
                "collision": 0,
                "path_found": 0,
                "path_length": float("nan"),
                "planning_time_ms": planning_time * 1000,
                "map_error": float(np.mean(
                    np.linalg.norm(
                        noisy_obstacles - true_obstacles,
                        axis=1
                    )
                ))
            })
            continue

        # Convert grid path to world coordinates
        path = [START] + [
            grid_to_world(x, y)
            for x, y in cells
        ] + [GOAL]

        collision = check_collision(path, true_obstacles)

        error = np.linalg.norm(
            noisy_obstacles - true_obstacles,
            axis=1
        ).mean()

        results.append({
            "noise": noise_level,
            "trial": trial,
            "success": int(not collision),
            "collision": int(collision),
            "path_found": 1,
            "path_length": path_length(path),
            "planning_time_ms": planning_time * 1000,
            "map_error": float(error)
        })

        if trial == 0:
            example_maps[noise_level] = (
                grid.copy(),
                path,
                noisy_obstacles.copy()
            )

# ==========================================
# SAVE CSV
# ==========================================

csv_file = OUTPUT / "noise_results.csv"

with open(csv_file, "w", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=results[0].keys()
    )
    writer.writeheader()
    writer.writerows(results)

print(f"\nSaved results to {csv_file}")


# ==========================================
# SUMMARY AND GRAPHS
# ==========================================

summary = []

for level in NOISE_LEVELS:
    subset = [r for r in results if r["noise"] == level]

    success_rate = np.mean(
        [r["success"] for r in subset]
    ) * 100

    collision_rate = np.mean(
        [r["collision"] for r in subset]
    ) * 100

    valid_lengths = [
        r["path_length"]
        for r in subset
        if r["path_found"] == 1
    ]

    avg_length = (
        np.mean(valid_lengths)
        if valid_lengths else float("nan")
    )

    avg_planning_ms = np.mean(
        [r["planning_time_ms"] for r in subset]
    )

    summary.append((
        level,
        success_rate,
        collision_rate,
        avg_length,
        avg_planning_ms
    ))

    print(
        f"Noise={level:.1f} | "
        f"Success={success_rate:.1f}% | "
        f"Collision={collision_rate:.1f}% | "
        f"Avg length={avg_length:.2f}m | "
        f"Planning={avg_planning_ms:.2f}ms"
    )

levels = [s[0] for s in summary]
success_rates = [s[1] for s in summary]
collision_rates = [s[2] for s in summary]
path_lengths = [s[3] for s in summary]

plt.figure(figsize=(8, 5))
plt.plot(levels, success_rates, "o-", label="Success rate")
plt.plot(levels, collision_rates, "s-", label="Collision rate")
plt.xlabel("Map noise standard deviation (m)")
plt.ylabel("Rate (%)")
plt.title("Navigation Performance vs Map Noise")
plt.ylim(0, 105)
plt.grid(alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig(OUTPUT / "performance_vs_noise.png", dpi=150)

plt.figure(figsize=(8, 5))
plt.plot(levels, path_lengths, "o-")
plt.xlabel("Map noise standard deviation (m)")
plt.ylabel("Mean planned path length (m)")
plt.title("Path Length vs Map Noise")
plt.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(OUTPUT / "path_length_vs_noise.png", dpi=150)

# Example maps for visual comparison
fig, axes = plt.subplots(
    1, len(NOISE_LEVELS),
    figsize=(17, 4),
    sharex=True,
    sharey=True
)

for ax, level in zip(axes, NOISE_LEVELS):
    if level not in example_maps:
        ax.set_title(f"Noise {level}\nNo path")
        continue

    grid, path, noisy = example_maps[level]

    ax.imshow(
        grid,
        origin="lower",
        extent=[-5, 5, -5, 5],
        cmap="gray_r"
    )

    # True obstacle outlines
    for ox, oy in true_obstacles:
        circle = plt.Circle(
            (ox, oy),
            COLLISION_RADIUS,
            fill=False,
            color="red",
            linestyle="--"
        )
        ax.add_patch(circle)

    px, py = zip(*path)
    ax.plot(px, py, "b-", linewidth=1.5)
    ax.scatter(*START, color="red", s=25)
    ax.scatter(*GOAL, color="green", s=25)
    ax.set_title(f"Noise = {level}")
    ax.set_aspect("equal")

plt.tight_layout()
plt.savefig(OUTPUT / "example_noisy_maps.png", dpi=150)

plt.show()
