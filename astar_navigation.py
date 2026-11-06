
import heapq
import math
import time

import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ==========================================
# 1. ENVIRONMENT SETTINGS
# ==========================================

MAP_SIZE = 10.0
RESOLUTION = 0.25
GRID_SIZE = int(MAP_SIZE / RESOLUTION)

START = (-4.0, -4.0)
GOAL = (4.0, 4.0)
FLIGHT_HEIGHT = 1.0

# Match the cylinders in your CoppeliaSim scene
OBSTACLE_NAMES = [
    "/Cylinder1",
    "/Cylinder2",
    "/Cylinder3"
]

# Cylinder radius = 0.4 meters
# Extra safety clearance = 0.35 meters
OBSTACLE_RADIUS = 0.4
SAFETY_MARGIN = 0.35
BLOCKED_RADIUS = OBSTACLE_RADIUS + SAFETY_MARGIN


# ==========================================
# 2. CONNECT TO COPPELIASIM
# ==========================================

client = RemoteAPIClient()
sim = client.require("sim")

drone = sim.getObject("/Drone")

obstacles = []

for name in OBSTACLE_NAMES:
    handle = sim.getObject(name)
    position = sim.getObjectPosition(handle, -1)

    obstacles.append((position[0], position[1]))
    print(f"{name}: {position}")

print("Connected and loaded obstacle positions.")


# ==========================================
# 3. CREATE OCCUPANCY GRID
# ==========================================

grid = np.zeros(
    (GRID_SIZE, GRID_SIZE),
    dtype=np.uint8
)


def world_to_grid(x, y):
    gx = int((x + MAP_SIZE / 2) / RESOLUTION)
    gy = int((y + MAP_SIZE / 2) / RESOLUTION)
    return gx, gy


def grid_to_world(gx, gy):
    x = -MAP_SIZE / 2 + (gx + 0.5) * RESOLUTION
    y = -MAP_SIZE / 2 + (gy + 0.5) * RESOLUTION
    return x, y


for gy in range(GRID_SIZE):
    for gx in range(GRID_SIZE):

        x, y = grid_to_world(gx, gy)

        for ox, oy in obstacles:

            distance = math.hypot(x - ox, y - oy)

            if distance <= BLOCKED_RADIUS:
                grid[gy, gx] = 1
                break

print("Occupancy grid created.")


# ==========================================
# 4. A* PATHFINDING
# ==========================================

def heuristic(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1]
    )


def astar(grid, start, goal):

    height, width = grid.shape

    if grid[start[1], start[0]] == 1:
        raise RuntimeError("Start is inside an obstacle.")

    if grid[goal[1], goal[0]] == 1:
        raise RuntimeError("Goal is inside an obstacle.")

    open_set = [(heuristic(start, goal), start)]

    came_from = {}
    g_score = {start: 0.0}

    directions = [
        (-1, 0), (1, 0),
        (0, -1), (0, 1),
        (-1, -1), (-1, 1),
        (1, -1), (1, 1)
    ]

    while open_set:

        _, current = heapq.heappop(open_set)

        if current == goal:

            path = [current]

            while current in came_from:
                current = came_from[current]
                path.append(current)

            return path[::-1]

        for dx, dy in directions:

            nx = current[0] + dx
            ny = current[1] + dy

            if not (0 <= nx < width and 0 <= ny < height):
                continue

            if grid[ny, nx] == 1:
                continue

            # Prevent cutting diagonally through blocked corners
            if dx != 0 and dy != 0:
                if grid[current[1], nx] == 1:
                    continue
                if grid[ny, current[0]] == 1:
                    continue

            neighbor = (nx, ny)

            new_cost = (
                g_score[current] + math.hypot(dx, dy)
            )

            if new_cost < g_score.get(
                neighbor, float("inf")
            ):

                came_from[neighbor] = current
                g_score[neighbor] = new_cost

                priority = (
                    new_cost + heuristic(neighbor, goal)
                )

                heapq.heappush(
                    open_set,
                    (priority, neighbor)
                )

    return None


start_cell = world_to_grid(*START)
goal_cell = world_to_grid(*GOAL)

path_cells = astar(grid, start_cell, goal_cell)

if path_cells is None:
    raise RuntimeError("No collision-free path found.")

path_world = [
    grid_to_world(gx, gy)
    for gx, gy in path_cells
]

print(f"A* found {len(path_world)} waypoints.")


# ==========================================
# 5. DISPLAY OCCUPANCY GRID AND PATH
# ==========================================

plt.figure(figsize=(8, 8))

plt.imshow(
    grid,
    origin="lower",
    extent=[-5, 5, -5, 5],
    cmap="gray_r",
    interpolation="nearest"
)

px = [p[0] for p in path_world]
py = [p[1] for p in path_world]

plt.plot(px, py, "b-", linewidth=2, label="A* path")

plt.scatter(
    [START[0]], [START[1]],
    color="red", s=80, label="Start"
)

plt.scatter(
    [GOAL[0]], [GOAL[1]],
    color="green", s=80, label="Goal"
)

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Occupancy Grid and A* Navigation")
plt.legend()
plt.grid(alpha=0.2)
plt.axis("equal")

plt.savefig("astar_path.png", dpi=150)
plt.show(block=False)
plt.pause(0.5)


# ==========================================
# 6. MOVE DRONE ALONG PLANNED PATH
# ==========================================

print("Starting A* navigation...")

# Non-dynamic Drone sphere is used as a visual proxy.
sim.setObjectPosition(
    drone, -1,
    [START[0], START[1], FLIGHT_HEIGHT]
)

# Move to the center of the first grid cell,
# then follow the planned waypoints.
route = [START] + path_world + [GOAL]

SPEED = 1.0  # meters per second
UPDATE_TIME = 0.03

for index in range(len(route) - 1):

    x1, y1 = route[index]
    x2, y2 = route[index + 1]

    distance = math.hypot(x2 - x1, y2 - y1)

    steps = max(
        1,
        math.ceil(distance / (SPEED * UPDATE_TIME))
    )

    for step in range(1, steps + 1):

        t = step / steps

        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)

        sim.setObjectPosition(
            drone,
            -1,
            [x, y, FLIGHT_HEIGHT]
        )

        time.sleep(UPDATE_TIME)

print("Drone reached the goal using A*!")

plt.show()
