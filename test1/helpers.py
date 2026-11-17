from test1.environment import (MAP_WIDTH, MAP_HEIGHT, DRONE_DIAMETER, DRONE_STARTS, TARGETS,
                        BOXES, CYLINDERS, BOX_SIZE, CYLINDER_DIAMETER)
from types import CellCoordinates   
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

