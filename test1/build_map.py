from shared.environment import (MAP_WIDTH, MAP_HEIGHT, DRONE_DIAMETER, DRONE_STARTS, TARGETS,
                        BOXES, CYLINDERS, BOX_SIZE, CYLINDER_DIAMETER)

from shared.helpers import world_to_cell, is_inside_grid
from test1.files import POINT_CLOUD_FILE, MAP_FILE

import numpy as np
import math

# dividing map into cells 
CELL_SIZE = 0.25
GRID_ROWS = int(MAP_HEIGHT / CELL_SIZE)
GRID_COLS = int(MAP_WIDTH / CELL_SIZE)

# pts lower than this are the floor
FLOOR_CUTOFF = 0.2
# set points to be counted as an obstacle
MIN_POINTS = 5

# load point cloud
pts = np.load(POINT_CLOUD_FILE)
print(f'loaded {len(pts)} points from {POINT_CLOUD_FILE}')

# start w/ empty grid of zeros
point_counts = np.zeros((GRID_ROWS, GRID_COLS), dtype = int)

points_used = 0
for x, y, z in pts:
    # skip the floor
    if z < FLOOR_CUTOFF:
        continue

    # get the row and column of the cell that contains this point
    row, col = world_to_cell(x, y)

    if not is_inside_grid(row, col):
        continue

    point_counts[row, col] = point_counts[row, col] + 1
    points_used += 1

print(f"{points_used} points are above the floor (obst. pts)")
# import sys
# np.set_printoptions(threshold=sys.maxsize)
# print(point_counts)

obstacle_cells = point_counts >= MIN_POINTS
stray_cells = (point_counts > 0) & (point_counts < MIN_POINTS)
print(f"obstacle cells: {obstacle_cells.sum()}, ignored {stray_cells.sum()} cells")

# margin between drones and obstacles
SAFETY_MARGIN = 0.35
DRONE_RADIUS = DRONE_DIAMETER / 2

INFLATION = DRONE_RADIUS + SAFETY_MARGIN

# start w/ copy then block evert cell within 
blocked = obstacle_cells.copy()
#
reach = math.ceil(INFLATION / CELL_SIZE)

for r in range(GRID_ROWS):
    for c in range(GRID_COLS):
        if not obstacle_cells[r, c]:
            continue

        for r_offset in range(-reach, reach+1):
            for c_offset in range(-reach, reach+1):
                dist = math.hypot(r_offset, c_offset) * CELL_SIZE
                if dist > INFLATION:
                    continue

                neighbor_r = r + r_offset
                neighbor_c = c + c_offset

                if is_inside_grid(neighbor_r, neighbor_c):
                    blocked[neighbor_r, neighbor_c] = True


# blocking off a strip along the floor edge that is one drone radius wide
# so drone does not hanf over edge
edge_cells = math.ceil(DRONE_RADIUS / CELL_SIZE)

# bottom, top row
blocked[:edge_cells, :] = True
blocked[-edge_cells:, :] = True

# left, right col
blocked[:, :edge_cells] = True
blocked[:, -edge_cells:] = True

blocked_percent = 100 * blocked.sum() / blocked.size
print(f"grew obstacles by {INFLATION:.2f}")
print(f"{blocked.sum()} of {blocked.size} cells"
      f" are blocked ({blocked_percent:.0f}% of the map)")


all_free = True
for n in DRONE_STARTS:

    start_r, start_c = world_to_cell(*DRONE_STARTS[n])
    target_r, target_c = world_to_cell(*TARGETS[n])

    if blocked[start_r, start_c]:
        all_free = False
        print(f"drone {n} start pos. is inside a blocked cell")
    if blocked[target_r, target_c]:
        all_free = False
        print(f"target {n} is inside a blocked cell")


if all_free:
    print("no drone is blocked")

np.save(MAP_FILE, blocked)
print(f"saved the map to {MAP_FILE}")