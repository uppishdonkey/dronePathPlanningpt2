from shared.environment import (MAP_WIDTH, MAP_HEIGHT, FLIGHT_HEIGHT, 
                         OBSTACLE_HEIGHT, SCANNER_START, BOXES, CYLINDERS)
from shared.sim_tools import *
from shared.aliases import *
from test1.files import POINT_CLOUD_FILE, SLAM_FILE
import numpy as np
import math
from typing import Dict
#import os

# draw the point cloud inside Coppelia as it scans
SHOW_POINTS_IN_SCENE = False
WALL_CLEARANCE = 0.5

VOXEL_SIZE = 0.05

# y value of each lane 
#   obstacle rows sit at y = -7.5, -5, ... 7.5, so each
#   lane is halfway between two rows (ie 1.25 from the row on either side)
LANE_YS = [-8.75, -3.75, 1.25, 6.25, 8.75]
#LANE_YS = [-8.75]
SCAN_SPEED = 1.5

MOVE_NOISE = 0.01
MATCH_RANGE = 0.15
MATCH_STEP = 0.05
MATCH_CELL = 0.1
MATCH_EVERY = 2

# https://manual.coppeliarobotics.com/en/sim/simGetObjectPosition.htm
scanner_drone = find("Drone")
start_pos: Position3D = sim.getObjectPosition(scanner_drone, WORLD_HANDLE)
estimated_pos = list(start_pos)

# move other drones off the course so lidar scanner doesn't map them
other_drones: Dict[DroneHandle, Position3D] = {}
for i in range(1,6):
    handle = find(f"Drone{i}")
    if handle != -1: # if drone handle exists, add to dictionary using [x,y,z] pos as key
        other_drones[handle] = sim.getObjectPosition(handle, WORLD_HANDLE)
        sim.setObjectPosition(handle, WORLD_HANDLE, [0,0,-99])

velodyne, cameras, camera_scripts = make_lidar(scanner_drone, SHOW_POINTS_IN_SCENE)

# x value where the scanner turns around at each end of a lane
x_turn = (MAP_WIDTH / 2) - WALL_CLEARANCE      # 17.5 - 0.5 = 17.0

# corners of the route, in the order the scanner visits them
waypoints: list[Position3D] = [[start_pos[0], start_pos[1]]]
for lane_number, y in enumerate(LANE_YS):
    if lane_number % 2 == 0:          # lanes 0, 2, 4: fly left to right
        waypoints.append([-x_turn, y])
        waypoints.append([x_turn, y])
    else:                             # lanes 1, 3: fly right to left
        waypoints.append([x_turn, y])
        waypoints.append([-x_turn, y])

route_length = 0.0
for i in range(len(waypoints) - 1):
    corner = waypoints[i]
    next_corner = waypoints[i + 1]

    piece_length = math.dist(corner, next_corner)
    route_length = route_length + piece_length

# how long the scan will take: time = distance / speed
num_lanes = len(LANE_YS)
flight_time = route_length / SCAN_SPEED

print(f"Route: {num_lanes} lanes, {route_length:.0f} m, "
      f"about {flight_time:.0f} s of simulated flight")

def shift_size(shift:Offset2D) -> float:
    return abs(shift[0]) + abs(shift[1])
    
def find_best_shift(obst_pts: PointCloud, guess: Position3D, 
                    known_cells: set[MatchCell]) -> Offset2D:

    if len(known_cells) == 0 or len(obst_pts) < 50:
        return 0.0, 0.0


    def score(shift_x: float, shift_y: float) -> int:
        xs = obst_pts[:, 0] + guess[0] + shift_x
        ys = obst_pts[:, 1] + guess[1] + shift_y

        r = np.round(ys / MATCH_CELL).astype(int)
        c = np.round(xs / MATCH_CELL).astype(int)
        cells: set[MatchCell] = set(zip(r, c))

        return len(cells & known_cells)

    best_shift: Offset2D = (0.0, 0.0)
    best_score = score(0.0, 0.0)

    shifts = np.arange(-MATCH_RANGE, MATCH_RANGE + 0.001, MATCH_STEP)
    all_shifts = []
    for sx in shifts:
        for sy in shifts:
            all_shifts.append((sx, sy))
        
    all_shifts.sort(key=shift_size)

    for sx, sy in all_shifts:
        n_score = score(sx, sy)

        if n_score > best_score:
            best_score = n_score
            best_shift = (sx, sy)
    
    return best_shift


voxels: set[Voxel] = set()
known_cells: set[MatchCell]= set()
true_path: WorldPath = []
estimated_path: WorldPath = []

set_stepping(True)
sim.startSimulation()
dt = sim.getSimulationTimeStep()

prev_x, prev_y = start_pos[0], start_pos[1]
steps = 0
try:
    for(x1, y1), (x2, y2) in zip(waypoints, waypoints[1:]):
        # distance between the two waypoints,
        #   and how far the scanner drone will move in on step of the sim
        distance = math.dist((x1, y1), (x2, y2))
        distance_per_step = SCAN_SPEED * dt

        steps_needed = distance / distance_per_step
        whole_steps = math.ceil(steps_needed)

        # number of moves made by scanner
        n = max(1, whole_steps)
        
        for k in range(1, n+1):
            # t is the difference between the two waypoints
            t = k / n

            dx = x2 - x1
            dy = y2 - y1

            curr_x = x1 + (t * dx)
            curr_y = y1 + (t * dy)

            curr_position: Position3D = [curr_x, curr_y, FLIGHT_HEIGHT]
            sim.setObjectPosition(scanner_drone, WORLD_HANDLE, curr_position)

            move_x = curr_x - prev_x
            move_y = curr_y - prev_y

            estimated_pos[0] += move_x + np.random.normal(0, MOVE_NOISE)
            estimated_pos[1] += move_y + np.random.normal(0, MOVE_NOISE)

            prev_x, prev_y = curr_x, curr_y

            relative_pts: PointCloud = read_lidar(velodyne, dt)

            if len(relative_pts) > 0:
                # only obstacle hits are useful
                heights = relative_pts[:, 2] + FLIGHT_HEIGHT
                is_above = heights > 0.2
                obst_pts: PointCloud = relative_pts[is_above]
                
                # every MATCH_EVERY steps, correct the pos. estimate by matching against known map
                if steps % MATCH_EVERY == 0:
                    correction: Offset2D = find_best_shift(obst_pts, 
                                                           estimated_pos, known_cells)
                    estimated_pos[0] += correction[0]
                    estimated_pos[1] += correction[1]

                pos_arr = np.array(estimated_pos)
                world_pts: PointCloud = relative_pts + pos_arr

                voxel_nums = np.round(world_pts / VOXEL_SIZE).astype(np.int32)
                unique_cubes = np.unique(voxel_nums, axis=0)

                for v in unique_cubes:
                    voxels.add(tuple(v))

                obst_xy = obst_pts[:, :2]
                pos_xy = np.array(estimated_pos[:2])

                obst_pos = obst_xy + pos_xy
                for x, y in obst_pos:
                        cell_x = round(x / MATCH_CELL)
                        cell_y = round(y / MATCH_CELL)
                        known_cells.add((cell_y, cell_x))

            true_path.append( (curr_x, curr_y) )
            estimated_path.append( (estimated_pos[0], estimated_pos[1]) )
            
            step()
            steps += 1

        print(f" reached ({x2:.1f}, {y2:.1f}), {len(voxels)} points so far")

finally:
    remove_lidar(velodyne, cameras, camera_scripts)
    sim.stopSimulation()
    while sim.getSimulationState() != sim.simulation_stopped:
        pass
    set_stepping(False)

    sim.setObjectPosition(scanner_drone, WORLD_HANDLE, start_pos)
    for handle, saved in other_drones.items():
        sim.setObjectPosition(handle, WORLD_HANDLE, saved)

print(f"scan finished after {steps * dt:.0f} s of simulated flight.")

# turn the set of cubes back into [x,y,z] positions and save them
points = np.array(sorted(voxels), dtype=np.float64) * VOXEL_SIZE
np.save(POINT_CLOUD_FILE, points)
print(f"saved {len(points)} points to {POINT_CLOUD_FILE}")

true_path = np.array(true_path)
estimated_path = np.array(estimated_path)
err = np.linalg.norm(true_path - estimated_path, axis=1)
print(f"position err: avg {err.mean():.3f} m, worst {err.max():.3f} m,")
print(f"\tat the end {err[-1]:.3f} ")
np.save(SLAM_FILE, np.stack([true_path, estimated_path]))
