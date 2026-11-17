from environment import MAP_WIDTH, MAP_HEIGHT, DRONE_DIAMETER, DRONE_STARTS, TARGETS
from helpers import world_to_cell, cells_near
from aliases import Path, Positions
import numpy as np


MAP_FILE = 'map.npy'
blocked = np.load("map.npy")
GRID_ROWS, GRID_COLS = blocked.shape
print(f"loaded {MAP_FILE}: {GRID_ROWS} rows x {GRID_COLS} cols")

# plan all 5 drones, 1 each turn
drone_numbers = sorted(DRONE_STARTS)
planned_paths: list[Path] = []
paths_in_meters: dict[int, Positions] = {}

def plan_drone(start: CellCoordinates):
    pass

for n_drone in drone_numbers:
    start = world_to_cell(*DRONE_STARTS[n])
    target = world_to_cell(*TARGETS[n])

    # drones that haven't been planned yet are still sitting at their start
    parked_cells = set()
    for other_drone in drone_numbers:
        if other_drone > n_drone:
            other_start = world_to_cell(*DRONE_STARTS[other_drone])

            neighbors = cells_near(other_start)
            parked_cells.update(neighbors)
    print(f"drone {n}: start cell {start}, target cell {target}, avoiding {len(parked_cells)} parked cells")


