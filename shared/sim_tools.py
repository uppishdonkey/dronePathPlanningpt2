from coppeliasim_zmqremoteapi_client import RemoteAPIClient #type: ignore
from aliases import *
import math
import numpy as np
import csv

client = RemoteAPIClient()
sim = client.require("sim")
simVision = client.require("simVision")
WORLD_HANDLE = sim.handle_world

# lidar settings
LIDAR_FREQUENCY = 5.0
NEAR_CLIPPING_PLANE = 0.3
FAR_CLIPPING_PLANE = 20.0
RESOLUTION_X = 256
RESOLUTION_Y = 96

CAMERA_SCRIPT = """
sim = require('sim')
simVision = require('simVision')

function sysCall_vision(inData)
    simVision.sensorDepthMapToWorkImg(inData.handle)
    local trigger, packet = simVision.velodyneDataFromWorkImg(inData.handle, {256, 16}, 0.5236)
    return {trigger = false, packedPackets = {packet}}
end
"""


def find(name: str) -> int:
    """return the handle of an object, or -1 if it doesn't exist"""
    return sim.getObject("/" + name, {"noError": True})


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


# one camera on a drone, looking along direction
def make_camera(drone: DroneHandle, direction: list[float]) -> SensorHandle:
    # option bits
    # 1=explictly handled (manual control), 2=perspective camera (cone vision),
    # 4=view frustrum not shown (hide the camera's view cone in the scene)
    sensor_options = 1 + 2 #+ 4

    int_params = [RESOLUTION_X, RESOLUTION_Y, 0, 0]
    float_params = [NEAR_CLIPPING_PLANE, FAR_CLIPPING_PLANE, math.radians(90), 0.05,
                    0, 0, 0, 0, 0, 0, 0]
    camera: SensorHandle = sim.createVisionSensor(sensor_options, int_params, float_params)

    origin = sim.getObjectPosition(drone, WORLD_HANDLE)
    z = np.array(direction, dtype=float)
    y = np.array([0.0, 0.0, 1.0])
    x = np.cross(y, z)
    matrix = [x[0], y[0], z[0], origin[0],
              x[1], y[1], z[1], origin[1],
              x[2], y[2], z[2], origin[2]]
    sim.setObjectMatrix(camera, WORLD_HANDLE, matrix)
    sim.setObjectParent(camera, drone, True)
    return camera


# a full Velodyne on a drone. Returns everything needed to read it and remove it later
def make_lidar(drone: DroneHandle, show_points: bool):
    directions = {"x_pos": [1, 0, 0], "y_pos": [0, 1, 0],
                  "x_neg": [-1, 0, 0], "y_neg": [0, -1, 0]}
    drone_name = sim.getObjectAlias(drone)

    cameras: list[SensorHandle] = []
    scripts: list[int] = []

    for name, direction in directions.items():
        camera = make_camera(drone, direction)
        sim.setObjectAlias(camera, f"{drone_name}_LidarCamera_{name}")   # unique per drone
        cameras.append(camera)

        script = sim.createScript(sim.scripttype_simulation, CAMERA_SCRIPT, 0, "lua")
        sim.setObjectParent(script, camera, True)
        scripts.append(script)

    options = 2 if show_points else 1
    velodyne: LidarHandle = simVision.createVelodyneVPL16(cameras, LIDAR_FREQUENCY, options, 2)
    return velodyne, cameras, scripts


# this step's lidar points, measured from the drone, in world directions [x, y, z]
def read_lidar(velodyne: LidarHandle, dt: float) -> PointCloud:
    data = simVision.handleVelodyneVPL16(velodyne + sim.handleflag_codedstring, dt)
    if not data:
        return np.empty((0, 3))

    if isinstance(data, (bytes, bytearray)):
        camera_pts = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
    else:
        camera_pts = np.array(data, dtype=np.float32).reshape(-1, 3)

    # the front camera's axes are turned, reorder to world directions
    return np.column_stack((camera_pts[:, 2], camera_pts[:, 0], camera_pts[:, 1]))

def remove_lidar(velodyne: LidarHandle, cameras: list[SensorHandle], scripts: list[int]):
    simVision.destroyVelodyneVPL16(velodyne)
    sim.removeObjects(cameras + scripts)

def save_results(rows: list[dict], filename: str):
    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)