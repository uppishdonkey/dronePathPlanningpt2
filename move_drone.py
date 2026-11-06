
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Connect to the open CoppeliaSim application
client = RemoteAPIClient()
sim = client.require('sim')

# Find our drone placeholder
drone = sim.getObject('/Drone')

# Starting position and destination
start = [-4.0, -4.0, 1.0]
goal = [4.0, 4.0, 1.0]

# Number of movement increments
steps = 200

print("Connected to CoppeliaSim!")

# Begin at the starting position
sim.setObjectPosition(drone, -1, start)

print("Moving drone...")

# Move along a straight line
for i in range(steps + 1):
    t = i / steps

    x = start[0] + t * (goal[0] - start[0])
    y = start[1] + t * (goal[1] - start[1])
    z = start[2] + t * (goal[2] - start[2])

    sim.setObjectPosition(drone, -1, [x, y, z])
    time.sleep(0.03)

print("Drone reached the destination!")
