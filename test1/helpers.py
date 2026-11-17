from environment import (MAP_WIDTH, MAP_HEIGHT, DRONE_DIAMETER, DRONE_STARTS, TARGETS,
                        BOXES, CYLINDERS, BOX_SIZE, CYLINDER_DIAMETER, MIN_DRONE_GAP)
from aliases import CellCoordinates   
import math

CELL_SIZE = 0.25
GRID_ROWS = int(MAP_HEIGHT / CELL_SIZE)
GRID_COLS = int(MAP_WIDTH / CELL_SIZE)


def world_to_cell(x: float, y: float) -> CellCoordinates:
    """
    the map is centered at (0, 0), so adding 1/2 the width/height shifts
    its bottom-left corner to (0, 0). Dividing by the cell size gives
    how many cells over (col) and up (row) the point is
    """

    # .floor() b/c we want the cell that contains the point, not the next cell over
    # must be an int b/c np.zeros() requires int indices
    column = math.floor((x + MAP_WIDTH / 2) / CELL_SIZE)
    row = math.floor((y + MAP_HEIGHT / 2) / CELL_SIZE)

    return row, column

def is_inside_grid(row: int, col: int) -> bool:
    row_check = 0 <= row < GRID_ROWS
    col_check = 0 <= col < GRID_COLS
    return row_check and col_check


def cell_distance(cell_a: CellCoordinates, cell_b: CellCoordinates) -> float:
    dr = cell_a[0] - cell_b[0]
    dc = cell_a[1] - cell_b[1]

    return math.hypot(dr, dc) * CELL_SIZE

# used for drones not planned with, returns set of close cells
def cells_near(center: CellCoordinates) -> set[CellCoordinates]:
    reach = math.ceil(MIN_DRONE_GAP / CELL_SIZE)
    nearby = set()

    for r_offset in range(-reach, reach+1):
        for c_offset in range(-reach, reach+1): 

            new_r = center[0] + r_offset
            new_c = center[1] + c_offset

            cell = (new_r, new_c)

            if cell_distance(cell, center) < MIN_DRONE_GAP:
                nearby.add(cell)

    return nearby
