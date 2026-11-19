from environment import *
from aliases import CellCoordinates, Position2D, CellPath
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

# convert a cell's row and column to the world coordinates of its center
    # ex. col * CELL_SIZE is the cell's left edge
    # adding 0.5 a cell moves to its middle, and subtracting half map size
    # moves back to world coordinates
def cell_to_world(row: int, col: int) -> Position2D:
    center_x = (col + 1/2) * CELL_SIZE
    x = -MAP_WIDTH / 2 + center_x

    center_y = (row + 1/2) * CELL_SIZE
    y = -MAP_HEIGHT / 2 + center_y

    return x, y

# where a planned drone is at a given tick, after path ends
# it stays at its targetr
def position_at(path: CellPath, t: int) -> CellCoordinates:
    if t < 0:
        t = 0

    if t >= len(path):
        return path[-1] # return the last cell in the path (ie target)
    
    return path[t]


def tooCloseToPlannedDrones(cell: CellCoordinates, 
                            t: int, planned_paths: list[CellPath]) -> bool:
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

def distanceToNearestObstacle(position: Position2D) -> float:
    
    half_l, half_w = BOX_SIZE[0] / 2, BOX_SIZE[1] / 2
    cyl_radius = CYLINDER_DIAMETER / 2

    x, y = position
    nearest = float("inf")

    for bx, by in BOXES:
        # dist from box center to point
        dx = abs(x - bx)
        dy = abs(y - by)

        # how far outside the box boundary in each direction
        outside_x = max(dx - half_l, 0)
        outside_y = max(dy - half_w, 0)

        # shortest dist to the box perimeter
        box_dist = math.hypot(outside_x, outside_y)

        nearest = min(nearest, box_dist)

    for cx, cy in CYLINDERS:
        cyl_center: Position2D = (cx, cy)

        # distance from point to center
        center_dist = math.dist(position, cyl_center)

        # distance from point to cylinder surface, subtract the radius
        cyl_dist = center_dist - cyl_radius

        nearest = min(nearest, cyl_dist)

    return nearest