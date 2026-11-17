from test1.environment import MAP_WIDTH, MAP_HEIGHT, DRONE_DIAMETER, DRONE_STARTS, TARGETS
from test1.build_map import world_to_cell, CellCoordinates
from types import Path
import numpy as np




MAP_FILE = 'map.npy'
blocked = np.load("map.npy")
GRID_ROWS, GRID_COLS = blocked.shape
print(f"loaded {MAP_FILE}: {GRID_ROWS} rows x {GRID_COLS} cols")


drone_numbers = sorted(DRONE_STARTS)
planned_paths: list[Path] = []
paths_in_meters: dict[int, list]