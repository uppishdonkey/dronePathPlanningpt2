"""
  1. Open drone_navigation.ttt in CoppeliaSim (simulation STOPPED).
  2. In Terminal, from the repo folder (with your .venv activated):
         python3 setup_environment.py
  3. The course is built and saved to multi_drone_course.ttt.
     drone_navigation.ttt on disk is not changed.

Run this once. It expects Jake's original scene, so running it again on
multi_drone_course.ttt would add a second copy of the course.

"""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import os

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

BOXES = [
    (-5.0, 7.5), (2.5, 5.0), (-12.5, 5.0), (12.5, 2.5), (0.0, 0.0),
    (-12.5, 0.0), (-2.5, -2.5), (7.5, -2.5), (-5.0, -5.0), (15.0, -5.0),
]
CYLINDERS = [
    (-10.0, 5.0), (7.5, 5.0), (-2.5, 2.5), (5.0, 2.5),
    (-7.5, 0.0), (2.5, -2.5), (2.5, -7.5), (10.0, -7.5),
]

# drone 1 -> target 1, drone 2 -> target 2, etc.
DRONE_STARTS = {
    1: (2.5, 7.5),
    2: (15.0, 5.0),
    3: (-12.5, 7.5),
    4: (-10.0, -7.5),
    5: (12.5, -7.5),
}
TARGETS = {
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

# ==========================================
# 2. CONNECT TO COPPELIASIM
# ==========================================
client = RemoteAPIClient()
sim = client.require("sim")

def find(name):
    """Return the handle of an object, or -1 if it doesn't exist."""
    return sim.getObject("/" + name, {"noError": True})

for name in ("Drone", "Cylinder1", "EnvFloor"):
    if find(name) != -1:
        raise SystemExit(
            f"The active scene already contains /{name}.\n"
            "Use File > New scene in CoppeliaSim, then run this again.")

# create a box or cylinder
def make_shape(kind, size, position, name, color, solid=True):
    if kind == "box":
        shape_type = sim.primitiveshape_cuboid
    elif kind == "cylinder":
        shape_type = sim.primitiveshape_cylinder
    else:
        shape_type = sim.primitiveshape_spheroid
    
    handle = sim.createPrimitiveShape(shape_type, size, 0)
    sim.setObjectPosition(handle, -1, position)
    sim.setObjectAlias(handle, name)
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, color)

    # keep it fixed in place (not pushed around by physics)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1 if solid else 0)

    # Solid things can be collided with and seen by sensors; markers cannot.
    if solid:
        props = (sim.objectspecialproperty_collidable | 
                 sim.objectspecialproperty_measurable | 
                 sim.objectspecialproperty_detectable)
    else:
        props = 0
    
    sim.setObjectSpecialProperty(handle, props)
    return handle


# ==========================================
# 3. FLOOR
# ==========================================

default_floor = find("Floor")
if default_floor != -1:
    sim.removeObjects(sim.getObjectsInTree(default_floor, sim.handle_all, 0))
    print("removed the new scene's default floor")

make_shape("box", [MAP_WIDTH, MAP_HEIGHT, 0.1], [0, 0, -0.05], "EnvFloor", FLOOR_COLOR)

# ==========================================
# 4. OBSTACLES
# ==========================================

h = OBSTACLE_HEIGHT
for i, (x, y) in enumerate(BOXES, start=1):
    make_shape("box", [BOX_SIZE[0], BOX_SIZE[1], h],
               [x, y, h / 2], f"Box{i}", OBSTACLE_COLOR)
 
d = CYLINDER_DIAMETER
for i, (x, y) in enumerate(CYLINDERS, start=1):
    make_shape("cylinder", [d, d, h], [x, y, h / 2], f"Cylinder{i}", OBSTACLE_COLOR)
  
# ==========================================
# 5. TARGET MARKERS
# ==========================================
for number, (x, y) in TARGETS.items():
    make_shape("cylinder", [0.6, 0.6, 0.02], [x, y, 0.01],
               f"Target{number}", TARGET_COLOR, solid=False)
 
# ==========================================
# 6. DRONES
# ==========================================
size = [DRONE_DIAMETER] * 3
 
sx, sy = SCANNER_START
make_shape("sphere", size, [sx, sy, FLIGHT_HEIGHT], "Drone", SCANNER_COLOR)
 
for number, (x, y) in DRONE_STARTS.items():
    make_shape("sphere", size, [x, y, FLIGHT_HEIGHT], f"Drone{number}", DRONE_COLOR)
 

# ==========================================
# 7. SAVE
# ==========================================
sim.saveScene(OUTPUT_SCENE)
print(f"saved the course to {OUTPUT_SCENE}")