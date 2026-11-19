from sim_tools import *
from aliases import WorldPath, DroneHandle, Position3D, DroneNumber, Position2D
from environment import *
from helpers import *

import json
import math
import os

PATHS_FILE = 'paths.json'
RESULTS_FILE = 'test1_results.csv'

PLOTS_FOLDER = "plots"
FLOWN_FILE = os.path.join(PLOTS_FOLDER, "flown_paths.json")

# how long one planning tick last in seconds
#   drone moves one 0.25 m cell per tick, 0.25 s per tick allows for 1 m/s
TICK_SECONDS = 0.25

with open(PATHS_FILE) as f:
    loaded = json.load(f)

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

drone_nums = sorted(paths)
print(f"loaded paths for drones {drone_nums} from {PATHS_FILE}")

drones: dict[int, DroneHandle] = {}
for n in drone_nums:
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

flown: dict[DroneNumber, list[Position2D]] = {}
for n in drone_nums:
    flown[n] = []
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
print(f"flying {len(drone_nums)} drones for {total_t} ticks "
      f"({total_t * TICK_SECONDS:.1f} s of simulated time)")

try:
    for t in range(total_t):
        for s in range(1, steps_per_t + 1):
            t_progress = s / steps_per_t

            # real time position of every drone at this moment
            curr: dict[int, Position2D] = {}

            for n in drone_nums:
                
                real_pos = sim.getObjectPosition(drones[n], WORLD_HANDLE)
                flown[n].append((real_pos[0], real_pos[1]))

                x, y = position_during_t(paths[n], t, t_progress,)
                curr[n] = (x, y)
                position: Position3D = [x, y, FLIGHT_HEIGHT]
                sim.setObjectPosition(drones[n], WORLD_HANDLE, position)

            for i in range(len(drone_nums)):
                for j in range(i+1, len(drone_nums)):

                    a = drone_nums[i]
                    b = drone_nums[j]

                    dist = math.dist(curr[a], curr[b])

                    closest_drone[a] = min(closest_drone[a], dist)
                    closest_drone[b] = min(closest_drone[b], dist)

                    if dist < DRONE_DIAMETER:
                        hit_drone[a] = True
                        hit_drone[b] = True

            for n in drone_nums:
                x, y = curr[n]
                pos = (x,y)
                clearance = distanceToNearestObstacle(pos)

                closest_obst[n] = min(closest_obst[n], clearance)

                if clearance < DRONE_RADIUS:
                    hit_obst[n] = True
            step()

        if t % 20 == 0:
            print(f"\ttick {t} of {total_t}")


finally:
    sim.stopSimulation()

    while sim.getSimulationState() != sim.simulation_stopped:
        pass
    set_stepping(False)

    for h, p in saved_pos.items():
        sim.setObjectPosition(h, WORLD_HANDLE, p)

# saving results summary
rows = []
for n in drone_nums:
    path = paths[n]

    length = 0.0
    for i in range(len(path) - 1):
        length += math.dist(path[i], path[i+1])

    # did it end at its target
    reached = math.dist(path[-1], TARGETS[n]) < 0.1
    collided = hit_drone[n] or hit_obst[n]
    success = reached and not collided

    arrival_time = (len(path) - 1) * TICK_SECONDS

    rows.append({
        "drone": n,
        "success": success,
        "reached_target": reached,
        "hit_drone": hit_drone[n],
        "hit_obstacle": hit_obst[n],
        "path_length_m": round(length, 2),
        "straight_line_m": round(math.dist(DRONE_STARTS[n], TARGETS[n]), 2),
        "arrival_time_s": arrival_time,
        "closest_drone_m": round(closest_drone[n], 2),
        "closest_obstacle_m": round(closest_obst[n], 2),
    })

    print(f"drone {n}: {'SUCCESS' if success else 'FAILED'}, "
              f"{length:.1f} m in {arrival_time:.1f} s, "
              f"closest drone {closest_drone[n]:.2f} m, closest obstacle {closest_obst[n]:.2f} m")
    
save_results(rows, RESULTS_FILE)

successes = sum(1 for row in rows if row["success"])
print(f"{successes} of {len(rows)} drones succeeded. "
      f"all drones arrived after {total_t * TICK_SECONDS:.1f} s.")
print(f"saved results to {RESULTS_FILE}")

os.makedirs(PLOTS_FOLDER, exist_ok=True)   # make plots/ if it isn't there yet
flown_dict = {}
for n in drone_nums:
    flown_dict[str(n)] = flown[n]
with open(FLOWN_FILE, "w") as f:
    json.dump(flown_dict, f, indent=2)
print(f"saved flown paths to {FLOWN_FILE}")



# print("drone handles:", drones)
# for h, p in saved_pos.items():
#     print(f"  handle {h}: {p}")

# for h, p in saved_pos.items():
#     sim.setObjectPosition(h, WORLD_HANDLE, p)
# print("put everything back")