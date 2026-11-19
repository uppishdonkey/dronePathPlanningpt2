from coppeliasim_zmqremoteapi_client import RemoteAPIClient #type: ignore
from environment import (MAP_WIDTH, MAP_HEIGHT, FLIGHT_HEIGHT, 
                         OBSTACLE_HEIGHT, SCANNER_START, BOXES, CYLINDERS)

from aliases import *
import numpy as np
import math
import matplotlib.pyplot as plt
from typing import Sequence, Dict, List
#import os

# draw the point cloud inside Coppelia as it scans
SHOW_POINTS_IN_SCENE = False
# how fast the lidar spins (rotations per second) trying 5-10
LIDAR_FREQUENCY = 5.0

# the minimum and maximum distance the lidar sensor will detect
# near clipping set to 0.3 b/c drones have diameter of 0.4 ie radius of 0.2
NEAR_CLIPPING_PLANE = 0.3
FAR_CLIPPING_PLANE = 20.0

WALL_CLEARANCE = 0.5

VOXEL_SIZE = 0.05

# y value of each lane 
#   obstacle rows sit at y = -7.5, -5, ... 7.5, so each
#   lane is halfway between two rows (ie 1.25 from the row on either side)
LANE_YS = [-8.75, -3.75, 1.25, 6.25, 8.75]
#LANE_YS = [-8.75]
SCAN_SPEED = 1.5

CAMERA_SCRIPT = """
sim = require('sim')
simVision = require('simVision')

function sysCall_vision(inData)
    simVision.sensorDepthMapToWorkImg(inData.handle)
    local trigger, packet = simVision.velodyneDataFromWorkImg(inData.handle, {256, 16}, 0.5236)
    return {trigger = false, packedPackets = {packet}}
end
"""

MOVE_NOISE = 0.01
MATCH_RANGE = 0.15
MATCH_STEP = 0.05
MATCH_CELL = 0.1
MATCH_EVERY = 2

# connect to Coppelia
client = RemoteAPIClient()
sim = client.require("sim")
simVision = client.require("simVision")
WORLD_HANDLE = sim.handle_world

def find(name: str) -> Sequence[float]: # return [x,y,z] position of an obj or -1 if DNE
    # noError(bool) will throw silent error if an obj was not found
    return sim.getObject("/" + name, {"noError":True})

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

# https://manual.coppeliarobotics.com/en/sim/simCreateVisionSensor.htm

# sensor X and Y resolutions
resolutionX = 256
resolutionY = 96

view_angle = math.radians(90)
cosmetic_sensor_size = 0.05

# option bits
# 1=explictly handled (manual control), 2=perspective camera (cone vision), 
# 4=view frustrum not shown (hide the camera's view cone in the scene)
sensor_options = 1 + 2 #+ 4
int_params = [resolutionX, resolutionY, 0, 0]
float_params = [NEAR_CLIPPING_PLANE, FAR_CLIPPING_PLANE, view_angle, cosmetic_sensor_size,
                0, 0 , 0, 0, 0, 0, 0]

# make a sensor at the scanner, looking along the given direction (a unit vector)
def make_camera(direction: Sequence[float], origin: Sequence[float]) -> SensorHandle:
    camera: SensorHandle = sim.createVisionSensor(sensor_options, int_params, float_params)

    z = np.array(direction, dtype = float)
    y = np.array([0.0, 0.0, 1.0])
    x = np.cross(y,z)

    matrix = [x[0], y[0], z[0], origin[0],
              x[1], y[1], z[1], origin[1],
              x[2], y[2], z[2], origin[2]]

    sim.setObjectMatrix(camera, WORLD_HANDLE, matrix)
    sim.setObjectParent(camera, scanner_drone, True)
    return camera

# https://manual.coppeliarobotics.com/en/simVision.htm

directions = {
    "x_pos":[1, 0, 0],
    "y_pos":[0, 1, 0],
    "x_neg": [-1, 0, 0],
    "y_neg": [0, -1, 0],
}

cameras: Dict[str, SensorHandle] = {}
camera_scripts: List[int] = []
for name, direction in directions.items():
    camera = make_camera(direction, start_pos)
    sim.setObjectAlias(camera, f"LidarCamera_{name}")
    cameras[name] = camera

    # attach the Lua script to this camera
    script = sim.createScript(sim.scripttype_simulation, CAMERA_SCRIPT, 0, "lua")
    sim.setObjectParent(script, camera, True)
    camera_scripts.append(script)

# velodyne options
# 1 = don't draw points, 2 = draw current sweep of points
velodyne_options = 2 if SHOW_POINTS_IN_SCENE else 1
point_size = 2

velodyne: LidarHandle = simVision.createVelodyneVPL16(list(cameras.values()), LIDAR_FREQUENCY, velodyne_options, point_size)

# plan lawnmower route
# turn around past the farthest obstacle while staying on the floor

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

# 
def set_stepping(on: bool):
    try:
        sim.setStepping(on)
    except Exception:
        client.setStepping(on)

def step():
    try:
        sim.step()
    except Exception:
        client.step()


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

            # read this steps slice of the spin, in world coordinates
            flags = sim.handleflag_codedstring # + sim.handleflag_abscoords
            data = simVision.handleVelodyneVPL16(velodyne + flags, dt)

            if data:
                if isinstance(data, (bytes, bytearray)):
                    pts: PointCloud = np.frombuffer(data, dtype=np.float32).reshape(-1,3)
                else:
                    pts: PointCloud = np.array(data, dtype=np.float32).reshape(-1,3)

                # cube_numbers = np.round(points / VOXEL_SIZE).astype(np.int32)
                # unique_cubes = np.unique(cube_numbers, axis=0)

                # for cube in unique_cubes:
                #     voxels.add(tuple(cube))


                # reorder from front camera axes to world directions x,y,z
                pz = pts[:, 2]
                px = pts[:, 0]
                py = pts[:, 1]

                relative_pts = np.column_stack((pz, px, py))

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
    simVision.destroyVelodyneVPL16(velodyne)
    sim.removeObjects(list(cameras.values()) + camera_scripts) 
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
np.save("point_cloud.npy", points)
print(f"saved {len(points)} points to point_cloud.npy")

true_path = np.array(true_path)
estimated_path = np.array(estimated_path)
err = np.linalg.norm(true_path - estimated_path, axis=1)
print(f"position err: avg {err.mean():.3f} m, worst {err.max():.3f} m,")
print(f"\tat the end {err[-1]:.3f} ")
np.save("slam_paths.npy", np.stack([true_path, estimated_path]))


# # testing
# print("waypoints:")
# for p in waypoints:
#     print("  ", p)

# for x, y in BOXES + CYLINDERS:
#     plt.plot(x, y, "rs")
# route_x = [point[0] for point in waypoints]
# route_y = [point[1] for point in waypoints]
# plt.plot(route_x, route_y, "b.-")
# plt.gca().set_aspect("equal")
# plt.title("scanner route (blue) and obstacle centers (red)")
# plt.show()

# input("view scene in application, press enter to clean up\n")

# # clean up
# simVision.destroyVelodyneVPL16(velodyne)
# sim.removeObjects(list(cameras.values()))
# for handle, position in other_drones.items():
#     sim.setObjectPosition(handle, WORLD_HANDLE, position)
# print("Cleaned up.")