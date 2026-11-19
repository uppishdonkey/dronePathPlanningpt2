import numpy as np

# grid (whole numbers) used by map and A*
CellCoordinates = tuple[int, int]
TimedCell = tuple[int, int, int]  # one "spot" in the time-aware A* search

CellPath = list[CellCoordinates]

SensorHandle = int
LidarHandle = int
DroneHandle = int
DroneNumber = int  

# world (floats) in meters, (0,0) is center of floor

# (x, y) in meters: a spot on the floor, ignoring height
Position2D = tuple[float, float]

Position3D = list[float] 
Waypoints = list[Position2D]
WorldPath = list[Position2D]


PointCloud = np.ndarray
Offset2D = tuple[float, float]
Voxel = tuple[int, int, int]
MatchCell = tuple[int, int]