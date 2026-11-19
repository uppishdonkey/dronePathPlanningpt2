from sim_tools import *
from environment import *
# ==========================================
# 2. CONNECT TO COPPELIASIM
# ==========================================

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
                 sim.objectspecialproperty_detectable |
                 sim.objectspecialproperty_renderable)
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
#     make_shape("box", [BOX_SIZE[0], BOX_SIZE[1], h],
#                [x, y, h / 2], f"Box{i}", OBSTACLE_COLOR)
    if (x, y) in ADDED_BOXES:
        color = ADDED_OBSTACLE_COLOR
    else:
        color = OBSTACLE_COLOR

    make_shape("box", [BOX_SIZE[0], BOX_SIZE[1], h],
                [x , y , h/2], f"Box{i}", color)
 
d = CYLINDER_DIAMETER
for i, (x, y) in enumerate(CYLINDERS, start=1):
    # make_shape("cylinder", [d, d, h], [x, y, h / 2], f"Cylinder{i}", OBSTACLE_COLOR)
    if (x, y) in ADDED_CYLINDERS:
        color = ADDED_OBSTACLE_COLOR
    else:
        color = OBSTACLE_COLOR

    make_shape("cylinder", [d, d, h], [x, y, h / 2], 
               f"Cylinder{i}", color)


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

# scanner drone
sx, sy = SCANNER_START
make_shape("sphere", size, [sx, sy, FLIGHT_HEIGHT], "Drone", SCANNER_COLOR)
 
for number, (x, y) in DRONE_STARTS.items():
    make_shape("sphere", size, [x, y, FLIGHT_HEIGHT], f"Drone{number}", DRONE_COLOR)
 

# ==========================================
# 7. SAVE
# ==========================================
sim.saveScene(OUTPUT_SCENE)
print(f"saved the course to {OUTPUT_SCENE}")