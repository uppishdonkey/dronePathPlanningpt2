import os
from shared.aliases import Position2D, DroneNumber
# ==========================================
# 1. ENVIRONMENT SETTINGS
# ==========================================
OUTPUT_SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                            "multi_drone_course.ttt")

MAP_WIDTH = 35.0
MAP_HEIGHT = 20.0
FLIGHT_HEIGHT = 1.0
OBSTACLE_HEIGHT = 2.0    # taller than FLIGHT_HEIGHT so LiDAR can see them

BOX_SIZE = (2.0, 0.75)
CYLINDER_DIAMETER = 1.0
DRONE_DIAMETER = 0.4
DRONE_RADIUS = DRONE_DIAMETER / 2

# obstacles added to make the course busier (drawn in their own color)
ADDED_BOXES: list[Position2D] = [(-10, -5.0)]
ADDED_CYLINDERS: list[Position2D] = [(2.5, -5.0)]
ADDED_OBSTACLE_COLOR = [0.25, 0.25, 0.25]


BOXES: list[Position2D] = [
    (-5.0, 7.5), (2.5, 5.0), (-12.5, 5.0), (12.5, 2.5), (0.0, 0.0),
    (-12.5, 0.0), (-2.5, -2.5), (7.5, -2.5), (-5.0, -5.0), (15.0, -5.0),
] + ADDED_BOXES

CYLINDERS: list[Position2D] = [
    (-10.0, 5.0), (7.5, 5.0), (-2.5, 2.5), (5.0, 2.5),
    (-7.5, 0.0), (2.5, -2.5), (2.5, -7.5), (10.0, -7.5),
] + ADDED_CYLINDERS

# drone 1 -> target 1, drone 2 -> target 2, etc.
DRONE_STARTS: dict[DroneNumber, Position2D] = {
    1: (2.5, 7.5),
    2: (15.0, 5.0),
    3: (-12.5, 7.5),
    4: (-10.0, -7.5),
    5: (12.5, -7.5),
}
TARGETS: dict[DroneNumber, Position2D]= {
    1: (-15.0, -7.5),
    2: (-7.5, 7.5),
    3: (5.0, -7.5),
    4: (15.0, 7.5),
    5: (-15.0, 0.0),
}

# where original scanner drone starts for Test 1
SCANNER_START = (-15.0, -5.0)

OBSTACLE_COLOR = [1.0, 0.0, 0.0]
FLOOR_COLOR = [1.0, 1.0, 1.0]
TARGET_COLOR = [0, 1.0, 0]
DRONE_COLOR = [0.0, 0.0, 0.0]
SCANNER_COLOR = [0.0, 0.4, 1.0]

# the closest two drone's centers are allowed to get
#   1.0 - 0.4 = 0.6 gap between drone edges
MIN_DRONE_GAP = 1.0