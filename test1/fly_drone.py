from coppeliasim_zmqremoteapi_client import RemoteAPIClient #type: ignore
from aliases import WorldPath, DroneHandle, Position3D, DroneNumber, Position2D
from environment import BOX_SIZE, CYLINDER_DIAMETER, FLIGHT_HEIGHT

import json



PATHS_FILE = 'paths.json'

# how long one planning tick last in seconds
#   drone moves one 0.25 m cell per tick, 0.25 s per tick allows for 1 m/s
TICK_SECONDS = 0.50


client = RemoteAPIClient()
sim = client.require("sim")
WORLD_HANDLE = sim.handle_world

with open(PATHS_FILE) as f:
    loaded = json.load(f)


def find(name: str) -> DroneHandle:
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

def position_during_t(path: WorldPath, t: int, t_progress: float) -> Position2D:
    last_t = len(path) - 1
    if t >= last_t: # if past the end of path, return final position (ie arrived)
        return path[-1][0], path[-1][1]

    # get the current and next points
    x1, y1 = path[t]
    x2, y2 = path[t+1]

    # weighted average based on progress (0.0=start, 1.0=end)
    x = x1 + t_progress * (x2 - x1)
    y = y1 + t_progress * (y2 - y1)

    return x, y

# create a dict mapping drone numbers to their paths in meters
paths: dict[int, WorldPath] = {}
for key, pos in loaded.items():
    paths[int(key)] = pos

drone_numbers = sorted(paths)
print(f"loaded paths for drones {drone_numbers} from {PATHS_FILE}")

drones: dict[int, DroneHandle] = {}
for n in drone_numbers:
    handle = find(f"Drone{n}")
    if handle == -1:
        raise SystemExit(f"could not find /Drone{n}")
    drones[n] = handle

# remember where all drones start, to return to original positons after flight
saved_pos: dict[DroneHandle, Position3D] = {}
for h in drones.values():
    saved_pos[h] = sim.getObjectPosition(h, WORLD_HANDLE)

# move scanner drone off course
scanner = find("Drone")
if scanner != -1:
    saved_pos[scanner] = sim.getObjectPosition(scanner, WORLD_HANDLE)
    sim.setObjectPosition(scanner, WORLD_HANDLE, [0,0,-99])

# holds the closest distance any other drone or obstacle got
closest_drone: dict[DroneNumber, float] = {}
closest_obst: dict[DroneNumber, float] = {}

# holds flag whether a drone hit another drone or an obstacle
hit_drone: dict[DroneNumber, bool] = {}
hit_obst: dict[DroneNumber, bool] = {}


for n in drone_numbers:
    # setting to infinity so any new recorded distance will be closer
    closest_drone[n] = float("inf")
    closest_obst[n] = float("inf")

    # setting to False so that if a collison happens it will be set to True
    hit_drone[n] = False
    hit_obst[n] = False


set_stepping(True)
sim.startSimulation()
dt = sim.getSimulationTimeStep()

# how manu sim steps make up one tick
steps_per_t = max(1, round(TICK_SECONDS / dt))

# flight can be said to end when the drone w/ the longest path arrives
longest_path = max(len(path) for path in paths.values())
total_t = longest_path - 1
print(f"flying {len(drone_numbers)} drones for {total_t} ticks "
      f"({total_t * TICK_SECONDS:.1f} s of simulated time)")

try:
    for t in range(total_t):
        for s in range(1, steps_per_t + 1):
            t_progress = s / steps_per_t

            # real time position of every drone at this moment
            curr: dict[int, Position2D] = {}

            for n in drone_numbers:
                x, y = position_during_t(paths[n], t, t_progress,)

                position: Position3D = [x, y, FLIGHT_HEIGHT]
                sim.setObjectPosition(drones[n], WORLD_HANDLE, position)

            step()


finally:
    sim.stopSimulation()

    while sim.getSimulationState() != sim.simulation_stopped:
        pass
    set_stepping(False)

    for h, p in saved_pos.items():
        sim.setObjectPosition(h, WORLD_HANDLE, p)

            

# print("drone handles:", drones)
# for h, p in saved_pos.items():
#     print(f"  handle {h}: {p}")

# for h, p in saved_pos.items():
#     sim.setObjectPosition(h, WORLD_HANDLE, p)
# print("put everything back")