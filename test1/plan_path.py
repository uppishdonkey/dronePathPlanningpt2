from environment import DRONE_STARTS, TARGETS, MIN_DRONE_GAP
from helpers import (world_to_cell, cells_near, cell_distance, cell_to_world, 
                     tooCloseToPlannedDrones)
from aliases import CellPath, WorldPath, CellCoordinates, TimedCell

import numpy as np
import heapq as hq
import math
import json


MAP_FILE = 'map.npy'
PATHS_FILE = 'paths.json'

blocked = np.load(MAP_FILE)
GRID_ROWS, GRID_COLS = blocked.shape
print(f"loaded {MAP_FILE}: {GRID_ROWS} rows x {GRID_COLS} cols")

# give up on a drone if it can't reach a target within this many ticks
MAX_TICKS = 800

# wait, or move to one of 8 neighbors
# each one is (row change, col change)
MOVES = [
    (0, 0),                                   # wait
    (1, 0), (-1, 0), (0, 1), (0, -1),         # up, down, right, left
    (1, 1), (1, -1), (-1, 1), (-1, -1),       # diagonals
]


def is_free(r: int, c: int) -> bool:
    r_inside = 0 <= r < GRID_ROWS
    c_inside = 0 <= c < GRID_COLS

    inside = r_inside and c_inside

    return inside and not blocked[r, c]

def plan_drone(start: CellCoordinates, target: CellCoordinates, 
               planned_paths: list[CellPath], parked_cells: set[CellCoordinates]):
    """
    like normal A*, buy each spot in search is (r, c, time tick) instead of (r, c)
    so the search knows when the drone is in each cell.
        This allows current drone to go around / wait to pass
    """

    # last time tick any planned drone is near this target
        # drone can only finish after that b/c it will stay parked at its target for
        # rest of the test
    last_busy_t = -1
    for other_path in planned_paths:
        for t, other_cell in enumerate(other_path):
            if cell_distance(other_cell, target) < MIN_DRONE_GAP:
                last_busy_t = max(last_busy_t, t)

    # minimum ticks to reach target
    def ticks_heuristic(cell: CellCoordinates) -> int:
        """chebyshev distance as A* heuristic, the max of row/col differences
        is equiv. to min ticks needed (diag cover both dimensions)"""
        dr = abs(cell[0] - target[0])
        dc = abs(cell[1] - target[1])

        return max(dr, dc)

    # starting rows and columns
    # handing back the most promising location first
    sr, sc = start

    # ticks so far + guess of ticks left, tick, row, col
    unexplored = [(ticks_heuristic(start), 0, sr, sc)]

    came_from: dict[TimedCell, TimedCell] = {}
    explored: set[TimedCell] = {(sr, sc, 0)} # path will be traced back at the end

    # while there are still unexplorec cells
    while unexplored:
        _, t, r, c = hq.heappop(unexplored)

        # reached the target, and no earlier drone needs spot later on
        if (r, c) == target and t > last_busy_t:
            path: CellPath = [(r, c)]
            spot: TimedCell = (r, c, t)

            while spot in came_from:
                spot = came_from[spot]

                path.append( (spot[0], spot[1]) )

            path.reverse()
            return path

        if t >= MAX_TICKS:
            continue

        for r_change, c_change in MOVES:

            next_r = r + r_change
            next_c = c + c_change

            next_t = t + 1
            next_cell = (next_r, next_c)

            if (next_r, next_c, next_t) in explored:
                continue

            if not is_free(next_r, next_c):
                continue

            is_diag = r_change != 0 and c_change != 0

            # diagonal cannot cut the corner of a blocked cell
            if is_diag:
                isRowDirectionFree = is_free(r + r_change, c)
                isColDirectionFree = is_free(r, c + c_change)

                if not isRowDirectionFree or not isColDirectionFree:
                    continue

            # stay away from drines still parked at their starts
            if next_cell in parked_cells:
                continue

            if tooCloseToPlannedDrones(next_cell, next_t, planned_paths):
                continue

            explored.add((next_r, next_c, next_t))
            came_from[(next_r, next_c, next_t)] = (r, c, t)

            estimate = next_t + ticks_heuristic(next_cell)
            hq.heappush(unexplored, (estimate, next_t, next_r, next_c))

    return None


# plan all 5 drones, 1 each turn
drone_numbers = sorted(DRONE_STARTS)
planned_paths: list[CellPath] = []
paths_in_meters: dict[int, WorldPath] = {}

for n_drone in drone_numbers:
    start = world_to_cell(*DRONE_STARTS[n_drone])
    target = world_to_cell(*TARGETS[n_drone])

    # drones that haven't been planned yet are still sitting at their start
    parked_cells = set()
    for other_drone in drone_numbers:
        if other_drone > n_drone:
            other_start = world_to_cell(*DRONE_STARTS[other_drone])

            neighbors = cells_near(other_start)

            parked_cells.update(neighbors)

    print(f"drone {n_drone}: start cell {start}, target cell {target}, avoiding {len(parked_cells)} parked cells")

    path = plan_drone(start, target, planned_paths, parked_cells)

    if path is None:
        print(f"Drone{n_drone}: no path found")
        continue

    planned_paths.append(path)

    # convert cells to world positions, 1/tick
        # 1st and last are the exact start and target
    positions: WorldPath = []
    for r, c in path:

        world_coords = cell_to_world(r, c)
        positions.append(list(world_coords))

    positions[0] = DRONE_STARTS[n_drone]
    positions[-1] = TARGETS[n_drone]

    paths_in_meters[n_drone] = positions

    length = 0.0
    for i in range(len(positions) - 1):
        curr_pos = positions[i]
        next_pos = positions[i + 1]

        pos_distance = math.dist(curr_pos, next_pos) 
        length += pos_distance

    straight_line = math.dist(DRONE_STARTS[n_drone], TARGETS[n_drone])
    print(f"\tpath {length:.1f} m (straight line {straight_line:.1f} m)")

    waits = 0
    for i in range(len(path) - 1):
        if path[i] == path[i + 1]:
            waits = waits + 1
    print(f"\t-> path found: {len(path) - 1} ticks, waited {waits} ticks")

with open(PATHS_FILE, "w") as f:
    paths_dict = {}
    for num, pos in paths_in_meters.items():
        paths_dict[str(num)] = pos

    json.dump(paths_dict, f, indent=2)