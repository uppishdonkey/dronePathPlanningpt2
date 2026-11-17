from environment import MAP_WIDTH, MAP_HEIGHT, DRONE_DIAMETER, DRONE_STARTS, TARGETS, MIN_DRONE_GAP
from helpers import world_to_cell, cells_near, cell_distance
from aliases import Path, Positions, CellCoordinates, TimedCell
import numpy as np
import heapq as hq


MAP_FILE = 'map.npy'
blocked = np.load("map.npy")
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

# plan all 5 drones, 1 each turn
drone_numbers = sorted(DRONE_STARTS)
planned_paths: list[Path] = []
paths_in_meters: dict[int, Positions] = {}

# where a planned drone is at a given tick, after path ends
# it stays at its targetr
def position_at(path: Path, t: int) -> CellCoordinates:
    if t < 0:
        t = 0

    if t >= len(path):
        return path[-1] # return the last cell in the path (ie target)
    
    return path[t]

def is_free(r: int, c: int) -> bool:
    r_inside = 0 <= r < GRID_ROWS
    c_inside = 0 <= c < GRID_COLS

    inside = r_inside and c_inside

    return inside and not blocked[r, c]

# 
def tooCloseToPlannedDrones(cell: CellCoordinates, 
                            t: int, planned_paths: list[Path]) -> bool:
    # for each drone alr planned
    for p in planned_paths:
        prev_t = t - 1
        next_t = t + 1

        # check 3 time steps before, now, after
        for other_t in (prev_t, t, next_t):

            # where is other drone at this time
            other_cell = position_at(p, other_t)

            # if too close to drone
            if cell_distance(cell, other_cell) < MIN_DRONE_GAP:
                return True
            
    # safe distance
    return False


def plan_drone(start: CellCoordinates, target: CellCoordinates, 
               planned_paths: list[Path], parked_cells: set[CellCoordinates]):
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
            path: Path = [(r, c)]
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

    waits = 0
    for i in range(len(path) - 1):
        if path[i] == path[i + 1]:
            waits = waits + 1
    print(f"   -> path found: {len(path) - 1} ticks, waited {waits} ticks")

    # convert cells to world positions, 1/tick
        # 1st and last are the exact start and target

