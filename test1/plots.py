import os
import json
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D

from environment import *
from aliases import *
from helpers import (CELL_SIZE, GRID_ROWS, GRID_COLS, world_to_cell,
                     is_inside_grid, distanceToNearestObstacle)

# from scan_map.py
LANE_YS = [-8.75, -3.75, 1.25, 6.25, 8.75]
WALL_CLEARANCE = 0.5
SCAN_SPEED = 1.5

# from build_map.py
FLOOR_CUTOFF = 0.2
MIN_POINTS = 5

# the Velodyne VPL-16 that sim_tools.make_lidar builds
LIDAR_RANGE = 20.0
LIDAR_NEAR = 0.3
LIDAR_BEAMS = 16
LIDAR_SPREAD_DEG = 30.0      # 16 beams spread from -15 to +15 degrees

# one simulation step (sim.getSimulationTimeStep() in scan_map.py)
SIM_DT = 0.05

PLOTS_FOLDER = "plots"
POINT_CLOUD_FILE = "point_cloud.npy"
MAP_FILE = "map.npy"
PATHS_FILE = "paths.json"
SLAM_FILE = "slam_paths.npy"
FLOWN_FILE = "paths.json"

# one color per drone
DRONE_PLOT_COLORS: dict[DroneNumber, str] = {
    1: "#2a78d6",   # blue
    2: "#eb6834",   # orange
    3: "#1baf7a",   # aqua
    4: "#eda100",   # yellow
    5: "#e87ba4",   # pink
}
SCANNER_PLOT_COLOR = "#2a78d6"
ESTIMATE_PLOT_COLOR = "#eb6834"
OBSTACLE_PLOT_COLOR = "#d03b3b"
ADDED_PLOT_COLOR = "#555555"
TEXT_COLOR = "#52514e"

def file_exists(filename: str) -> bool:
    if os.path.exists(filename):
        return True
    print(f"  skipped: {filename} not found")
    return False

def new_figure(title: str):
    # 35 x 20 m floor, so a wide figure
    fig, ax = plt.subplots(figsize=(12, 7.5))
    ax.set_title(title, loc="left", fontsize=13)
    ax.set_xlim(-MAP_WIDTH / 2, MAP_WIDTH / 2)
    ax.set_ylim(-MAP_HEIGHT / 2, MAP_HEIGHT / 2)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", color=TEXT_COLOR)
    ax.set_ylabel("y (m)", color=TEXT_COLOR)
    ax.tick_params(colors=TEXT_COLOR)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    return fig, ax

def save_figure(fig, filename: str):
    os.makedirs(PLOTS_FOLDER, exist_ok=True)
    full_name = os.path.join(PLOTS_FOLDER, filename)
    fig.savefig(full_name, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  saved {full_name}")

def draw_obstacles(ax, filled: bool = True, color: str = OBSTACLE_PLOT_COLOR):
    """draw the TRUE obstacle shapes (from environment.py)"""
    length, width = BOX_SIZE
    radius = CYLINDER_DIAMETER / 2

    for bx, by in BOXES:
        is_added = (bx, by) in ADDED_BOXES
        box_color = ADDED_PLOT_COLOR if (is_added and filled) else color
        corner = (bx - length / 2, by - width / 2)
        ax.add_patch(Rectangle(corner, length, width, fill=filled,
                               facecolor=box_color, edgecolor=box_color,
                               linewidth=1.5, zorder=3))

    for cx, cy in CYLINDERS:
        is_added = (cx, cy) in ADDED_CYLINDERS
        cyl_color = ADDED_PLOT_COLOR if (is_added and filled) else color
        ax.add_patch(Circle((cx, cy), radius, fill=filled,
                            facecolor=cyl_color, edgecolor=cyl_color,
                            linewidth=1.5, zorder=3))

def draw_starts_and_targets(ax, label_them: bool = True):
    """circle = where a drone starts, star = its target"""
    for n in DRONE_STARTS:
        sx, sy = DRONE_STARTS[n]
        tx, ty = TARGETS[n]
        color = DRONE_PLOT_COLORS[n]

        ax.scatter(sx, sy, s=120, marker="o", color=color,
                   edgecolor="white", linewidth=1.5, zorder=6)
        ax.scatter(tx, ty, s=220, marker="*", color=color,
                   edgecolor="white", linewidth=1.0, zorder=6)

        if label_them:
            ax.annotate(f"D{n}", (sx, sy), xytext=(7, 7), textcoords="offset points",
                        fontsize=9, color=TEXT_COLOR, zorder=7)
            ax.annotate(f"T{n}", (tx, ty), xytext=(7, 7), textcoords="offset points",
                        fontsize=9, color=TEXT_COLOR, zorder=7)


def drone_legend_items() -> list:
    items = []
    for n in DRONE_STARTS:
        items.append(Line2D([], [], color=DRONE_PLOT_COLORS[n], linewidth=2,
                            label=f"Drone {n}"))
    items.append(Line2D([], [], marker="o", color="grey", linestyle="",
                        markersize=8, label="start"))
    items.append(Line2D([], [], marker="*", color="grey", linestyle="",
                        markersize=12, label="target"))
    return items


def put_legend_outside(ax, items: list):
    ax.legend(handles=items, loc="upper left", bbox_to_anchor=(1.01, 1.0),
              frameon=False, fontsize=9)


def scanner_route() -> Waypoints:
    """the same lawnmower corners scan_map.py builds"""
    x_turn = MAP_WIDTH / 2 - WALL_CLEARANCE

    route: Waypoints = [SCANNER_START]
    for lane_number, y in enumerate(LANE_YS):
        if lane_number % 2 == 0:          # lanes 0, 2, 4: left to right
            route.append((-x_turn, y))
            route.append((x_turn, y))
        else:                             # lanes 1, 3: right to left
            route.append((x_turn, y))
            route.append((-x_turn, y))
    return route


def points_along_route(route: Waypoints, spacing: float) -> list[Position2D]:
    """spots every `spacing` meters along the route"""
    spots: list[Position2D] = []
    for (x1, y1), (x2, y2) in zip(route, route[1:]):
        piece_length = math.dist((x1, y1), (x2, y2))
        n = max(1, math.ceil(piece_length / spacing))
        for k in range(n):
            t = k / n
            spots.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    spots.append(route[-1])
    return spots


def draw_point_cloud(ax, pts: PointCloud, max_points: int = 300_000):
    """top-down view: floor points light grey, obstacle points colored by height"""
    # too many points makes the png huge, so only keep some
    if len(pts) > max_points:
        keep = np.random.default_rng(0).choice(len(pts), max_points, replace=False)
        pts = pts[keep]

    is_floor = pts[:, 2] < FLOOR_CUTOFF
    floor_pts = pts[is_floor]
    obst_pts = pts[~is_floor]

    ax.scatter(floor_pts[:, 0], floor_pts[:, 1], s=0.2, color="#dcdcdc",
               rasterized=True, zorder=1)

    # draw low points first so the tall ones end up on top
    order = np.argsort(obst_pts[:, 2])
    obst_pts = obst_pts[order]
    dots = ax.scatter(obst_pts[:, 0], obst_pts[:, 1], s=0.5, c=obst_pts[:, 2],
                      cmap="Blues", vmin=-0.5, vmax=OBSTACLE_HEIGHT + 0.2,
                      rasterized=True, zorder=2)
    bar = plt.colorbar(dots, ax=ax, fraction=0.025, pad=0.01)
    bar.set_label("point height (m)", color=TEXT_COLOR)


def obstacle_cells_from_points(pts: PointCloud) -> np.ndarray:
    """same counting build_map.py does, but with numpy instead of a loop"""
    obst_pts = pts[pts[:, 2] >= FLOOR_CUTOFF]

    cols = np.floor((obst_pts[:, 0] + MAP_WIDTH / 2) / CELL_SIZE).astype(int)
    rows = np.floor((obst_pts[:, 1] + MAP_HEIGHT / 2) / CELL_SIZE).astype(int)

    inside = (rows >= 0) & (rows < GRID_ROWS) & (cols >= 0) & (cols < GRID_COLS)
    rows, cols = rows[inside], cols[inside]

    point_counts = np.zeros((GRID_ROWS, GRID_COLS), dtype=int)
    np.add.at(point_counts, (rows, cols), 1)    # +1 for every point in its cell

    return point_counts >= MIN_POINTS


def draw_grid_image(ax, grid: np.ndarray, color: str, zorder: int = 1):
    """paint every True cell of a (row, col) grid in one color"""
    picture = np.zeros((GRID_ROWS, GRID_COLS, 4))
    rgba = matplotlib.colors.to_rgba(color)
    picture[grid] = rgba

    # origin="lower" so row 0 is at the bottom (y = -10), like world_to_cell
    ax.imshow(picture, origin="lower", interpolation="nearest", zorder=zorder,
              extent=(-MAP_WIDTH / 2, MAP_WIDTH / 2, -MAP_HEIGHT / 2, MAP_HEIGHT / 2))


def layout_grid_preview():
    """the course: obstacles on the 2.5 m grid, starts, targets, scanner start"""
    fig, ax = new_figure("Course layout (2.5 m grid)")

    # 2.5 m grid lines
    for x in np.arange(-17.5, 17.5 + 0.01, 2.5):
        ax.axvline(x, color="#e6e6e6", linewidth=0.8, zorder=0)
    for y in np.arange(-10.0, 10.0 + 0.01, 2.5):
        ax.axhline(y, color="#e6e6e6", linewidth=0.8, zorder=0)

    draw_obstacles(ax)

    # dashed straight line from each start to its target
    for n in DRONE_STARTS:
        sx, sy = DRONE_STARTS[n]
        tx, ty = TARGETS[n]
        ax.plot([sx, tx], [sy, ty], linestyle="--", linewidth=1.2,
                color=DRONE_PLOT_COLORS[n], alpha=0.7, zorder=4)

    draw_starts_and_targets(ax)

    ax.scatter(*SCANNER_START, s=160, marker="s", color="black", zorder=6)
    ax.annotate("scanner", SCANNER_START, xytext=(8, -12), textcoords="offset points",
                fontsize=9, color=TEXT_COLOR)

    ax.set_xticks(np.arange(-17.5, 17.5 + 0.01, 2.5))
    ax.set_yticks(np.arange(-10.0, 10.0 + 0.01, 2.5))
    ax.tick_params(labelsize=8)

    items = drone_legend_items()
    items.append(Rectangle((0, 0), 1, 1, color=OBSTACLE_PLOT_COLOR, label="obstacle"))
    items.append(Rectangle((0, 0), 1, 1, color=ADDED_PLOT_COLOR, label="added obstacle"))
    items.append(Line2D([], [], marker="s", color="black", linestyle="",
                        markersize=8, label="scanner start"))
    put_legend_outside(ax, items)

    save_figure(fig, "layout_grid_preview.png")


def ray_hits_box(px, py, dir_x, dir_y, box_center: Position2D) -> np.ndarray:
    """flat (2D) distance along each ray to a box, inf if it misses"""
    half_l, half_w = BOX_SIZE[0] / 2, BOX_SIZE[1] / 2
    bx, by = box_center

    # "slab" method: when does the ray enter/leave the box's x range and y range
    with np.errstate(divide="ignore", invalid="ignore"):
        t_x1 = (bx - half_l - px) / dir_x
        t_x2 = (bx + half_l - px) / dir_x
        t_y1 = (by - half_w - py) / dir_y
        t_y2 = (by + half_w - py) / dir_y

    t_enter = np.maximum(np.minimum(t_x1, t_x2), np.minimum(t_y1, t_y2))
    t_leave = np.minimum(np.maximum(t_x1, t_x2), np.maximum(t_y1, t_y2))

    hit = (t_enter <= t_leave) & (t_enter > 0)
    return np.where(hit, t_enter, np.inf)


def ray_hits_cylinder(px, py, dir_x, dir_y, center: Position2D) -> np.ndarray:
    """flat (2D) distance along each ray to a cylinder, inf if it misses"""
    radius = CYLINDER_DIAMETER / 2
    to_center_x = center[0] - px
    to_center_y = center[1] - py

    # closest approach of the ray to the cylinder's center
    along = to_center_x * dir_x + to_center_y * dir_y
    closest_sq = (to_center_x ** 2 + to_center_y ** 2) - along ** 2
    inside_sq = radius ** 2 - closest_sq

    hit = (inside_sq >= 0) & (along > 0)
    t_hit = along - np.sqrt(np.maximum(inside_sq, 0))
    return np.where(hit & (t_hit > 0), t_hit, np.inf)


def fake_lidar_sweep(px: float, py: float) -> PointCloud:
    """
    one perfect 360 degree sweep of a 16-beam lidar at (px, py, FLIGHT_HEIGHT),
    using the TRUE obstacles. No noise, so it's the best scan possible.
    """
    azimuths = np.radians(np.arange(0, 360, 1.0))
    elevations = np.radians(np.linspace(-LIDAR_SPREAD_DEG / 2, LIDAR_SPREAD_DEG / 2,
                                        LIDAR_BEAMS))
    az, el = np.meshgrid(azimuths, elevations)
    az, el = az.ravel(), el.ravel()

    dir_x, dir_y = np.cos(az), np.sin(az)

    # flat distance to the first obstacle wall each ray hits
    wall_dist = np.full(len(az), np.inf)
    for box in BOXES:
        wall_dist = np.minimum(wall_dist, ray_hits_box(px, py, dir_x, dir_y, box))
    for cyl in CYLINDERS:
        wall_dist = np.minimum(wall_dist, ray_hits_cylinder(px, py, dir_x, dir_y, cyl))

    # height of the ray when it reaches that wall
    wall_z = FLIGHT_HEIGHT + wall_dist * np.tan(el)
    hits_wall = np.isfinite(wall_dist) & (wall_z >= 0) & (wall_z <= OBSTACLE_HEIGHT)

    # downward rays reach the floor at this flat distance
    with np.errstate(divide="ignore"):
        floor_dist = np.where(el < 0, FLIGHT_HEIGHT / np.tan(-el), np.inf)

    # whatever the ray reaches first
    use_floor = floor_dist < np.where(hits_wall, wall_dist, np.inf)
    flat_dist = np.where(use_floor, floor_dist, np.where(hits_wall, wall_dist, np.inf))
    hit_z = np.where(use_floor, 0.0, wall_z)

    # real distance (not flat) must be between near and far clipping
    real_dist = flat_dist / np.cos(el)
    seen = np.isfinite(flat_dist) & (real_dist >= LIDAR_NEAR) & (real_dist <= LIDAR_RANGE)

    xs = px + flat_dist[seen] * dir_x[seen]
    ys = py + flat_dist[seen] * dir_y[seen]
    zs = hit_z[seen]

    # the floor stops at the map edge
    on_map = (np.abs(xs) <= MAP_WIDTH / 2) & (np.abs(ys) <= MAP_HEIGHT / 2)
    return np.column_stack((xs, ys, zs))[on_map]


def expected_scan_preview():
    """what a perfect scan along the lawnmower route should look like"""
    route = scanner_route()
    spots = points_along_route(route, spacing=1.0)    # one sweep every meter

    all_sweeps = []
    for px, py in spots:
        all_sweeps.append(fake_lidar_sweep(px, py))
    pts = np.vstack(all_sweeps)

    # 5 cm voxels like scan_map.py: keep one point per little cube
    voxels = np.unique(np.round(pts / 0.05).astype(int), axis=0)
    pts = voxels * 0.05
    print(f"  expected scan: {len(pts)} points from {len(spots)} sweeps")

    fig, ax = new_figure("Expected scan (perfect lidar, true obstacles)")
    draw_point_cloud(ax, pts)
    route_x = [p[0] for p in route]
    route_y = [p[1] for p in route]
    ax.plot(route_x, route_y, color="black", linewidth=1.0, alpha=0.6, zorder=4)
    draw_obstacles(ax, filled=False)

    items = [Line2D([], [], color="black", label="scanner route"),
             Rectangle((0, 0), 1, 1, fill=False, edgecolor=OBSTACLE_PLOT_COLOR,
                       label="true obstacle")]
    ax.legend(handles=items, loc="upper left", bbox_to_anchor=(0.0, -0.08),
              ncol=2, frameon=False, fontsize=9)
    save_figure(fig, "expected_scan_preview.png")


def scan_preview():
    """the real point cloud scan_map.py saved, with the flown route on top"""
    if not file_exists(POINT_CLOUD_FILE):
        return
    pts: PointCloud = np.load(POINT_CLOUD_FILE)

    fig, ax = new_figure(f"Real scan: {len(pts):,} points (point_cloud.npy)")
    draw_point_cloud(ax, pts)

    items = []
    if os.path.exists(SLAM_FILE):
        true_path = np.load(SLAM_FILE)[0]
        ax.plot(true_path[:, 0], true_path[:, 1], color="black",
                linewidth=1.0, alpha=0.6, zorder=4)
        items.append(Line2D([], [], color="black", label="scanner route"))

    draw_obstacles(ax, filled=False)
    items.append(Rectangle((0, 0), 1, 1, fill=False, edgecolor=OBSTACLE_PLOT_COLOR,
                           label="true obstacle"))
    ax.legend(handles=items, loc="upper left", bbox_to_anchor=(0.0, -0.08),
              ncol=2, frameon=False, fontsize=9)
    save_figure(fig, "scan_preview.png")

def lidar_top_view():
    """the original scan picture: floor grey, obstacle hits yellow (low) to red (high)"""
    if not file_exists(POINT_CLOUD_FILE):
        return
    pts: PointCloud = np.load(POINT_CLOUD_FILE)

    # keep the png small: only draw some of the points
    if len(pts) > 200_000:
        keep = np.random.default_rng(0).choice(len(pts), 200_000, replace=False)
        pts = pts[keep]

    is_floor = pts[:, 2] < FLOOR_CUTOFF
    floor_pts = pts[is_floor]
    obst_pts = pts[~is_floor]

    fig, ax = plt.subplots(figsize=(12, 7))
    ax.scatter(floor_pts[:, 0], floor_pts[:, 1], s=0.3, c="lightgrey",
               label="floor", rasterized=True)
    dots = ax.scatter(obst_pts[:, 0], obst_pts[:, 1], s=4, c=obst_pts[:, 2],
                      cmap="autumn_r", vmin=FLOOR_CUTOFF, vmax=OBSTACLE_HEIGHT,
                      label="obstacles", rasterized=True)

    route = scanner_route()
    route_x = [p[0] for p in route]
    route_y = [p[1] for p in route]
    ax.plot(route_x, route_y, "b--o", linewidth=1, markersize=4, label="scanner route")

    bar = fig.colorbar(dots, ax=ax, fraction=0.025, pad=0.02)
    bar.set_label("height of hit (m)")

    ax.set_title("LiDAR point cloud (top view)")
    ax.set_xlim(-MAP_WIDTH / 2, MAP_WIDTH / 2)
    ax.set_ylim(-MAP_HEIGHT / 2, MAP_HEIGHT / 2)
    ax.set_aspect("equal")
    ax.legend(loc="upper right")

    save_figure(fig, "lidar_top_view.png")


def route_check():
    """is the lawnmower route safe? colors any part closer than a drone radius"""
    route = scanner_route()
    spots = points_along_route(route, spacing=0.05)

    gaps = [distanceToNearestObstacle(spot) for spot in spots]
    closest = min(gaps)
    closest_spot = spots[gaps.index(closest)]

    # a gap smaller than the drone's radius means the drone's body touches it
    too_close = [spot for spot, gap in zip(spots, gaps) if gap < DRONE_RADIUS]

    length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    minutes = length / SCAN_SPEED / 60

    fig, ax = new_figure(f"Scanner route check: {length:.0f} m, about "
                         f"{minutes:.1f} min at {SCAN_SPEED} m/s")
    draw_obstacles(ax)

    route_x = [p[0] for p in route]
    route_y = [p[1] for p in route]
    ax.plot(route_x, route_y, color=SCANNER_PLOT_COLOR, linewidth=2, zorder=4)
    ax.scatter(route_x, route_y, s=25, color=SCANNER_PLOT_COLOR, zorder=5)

    # number each corner so it's easy to follow
    for i, (x, y) in enumerate(route):
        ax.annotate(str(i), (x, y), xytext=(4, 4), textcoords="offset points",
                    fontsize=8, color=TEXT_COLOR)

    if too_close:
        bad_x = [p[0] for p in too_close]
        bad_y = [p[1] for p in too_close]
        ax.scatter(bad_x, bad_y, s=10, color="black", zorder=6)

    ax.scatter(*closest_spot, s=150, facecolor="none", edgecolor="black",
               linewidth=1.5, zorder=6)
    ax.annotate(f"closest: {closest:.2f} m", closest_spot, xytext=(10, -14),
                textcoords="offset points", fontsize=9, color=TEXT_COLOR)

    items = [Line2D([], [], color=SCANNER_PLOT_COLOR, linewidth=2, marker="o",
                    markersize=5, label="scanner route"),
             Rectangle((0, 0), 1, 1, color=OBSTACLE_PLOT_COLOR, label="obstacle")]
    if too_close:
        items.append(Line2D([], [], marker="o", color="black", linestyle="",
                            markersize=4, label=f"closer than {DRONE_RADIUS} m (hit)"))
    put_legend_outside(ax, items)

    if too_close:
        print(f"  WARNING: {len(too_close)} route spots hit an obstacle")
    print(f"  closest the route gets to an obstacle: {closest:.2f} m")
    save_figure(fig, "route_check.png")


def map_preview():
    """the map plan_path.py uses: black = seen obstacle, grey = safety buffer"""
    if not file_exists(MAP_FILE) or not file_exists(POINT_CLOUD_FILE):
        return
    blocked = np.load(MAP_FILE)
    obstacle_cells = obstacle_cells_from_points(np.load(POINT_CLOUD_FILE))

    blocked_percent = 100 * blocked.sum() / blocked.size
    fig, ax = new_figure(f"Map: {blocked.sum()} of {blocked.size} cells blocked "
                         f"({blocked_percent:.0f}%)")

    buffer_cells = blocked & ~obstacle_cells
    draw_grid_image(ax, buffer_cells, "#c8c8c8", zorder=1)
    draw_grid_image(ax, obstacle_cells, "#222222", zorder=2)
    draw_obstacles(ax, filled=False)
    draw_starts_and_targets(ax)

    start_and_target = drone_legend_items()[-2:]    # just the start / target markers
    items = [Rectangle((0, 0), 1, 1, color="#222222", label="obstacle cell (scan)"),
             Rectangle((0, 0), 1, 1, color="#c8c8c8", label="safety buffer"),
             Rectangle((0, 0), 1, 1, fill=False, edgecolor=OBSTACLE_PLOT_COLOR,
                       label="true obstacle")] + start_and_target
    put_legend_outside(ax, items)

    # any start or target sitting in a blocked cell can't be planned
    for n in DRONE_STARTS:
        for name, spot in (("start", DRONE_STARTS[n]), ("target", TARGETS[n])):
            row, col = world_to_cell(*spot)
            if is_inside_grid(row, col) and blocked[row, col]:
                print(f"  WARNING: drone {n} {name} is in a blocked cell")

    save_figure(fig, "map_preview.png")


def paths_preview():
    """every planned path from paths.json on top of the map"""
    if not file_exists(PATHS_FILE):
        return
    with open(PATHS_FILE) as f:
        paths_dict = json.load(f)

    fig, ax = new_figure("Planned paths (paths.json)")

    if os.path.exists(MAP_FILE):
        draw_grid_image(ax, np.load(MAP_FILE), "#dcdcdc", zorder=1)
    draw_obstacles(ax)

    for key, path in paths_dict.items():
        n = int(key)
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]
        ax.plot(path_x, path_y, color=DRONE_PLOT_COLORS[n], linewidth=2, zorder=4)

        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        print(f"  drone {n}: {len(path)} ticks, {length:.1f} m")

    draw_starts_and_targets(ax)

    items = drone_legend_items()
    items.append(Rectangle((0, 0), 1, 1, color="#dcdcdc", label="blocked (map)"))
    put_legend_outside(ax, items)
    save_figure(fig, "paths_preview.png")

def expected_vs_actual_paths():
    """planned paths (paths.json) vs where the drones really flew (flown_paths.json)"""
    if not file_exists(PATHS_FILE) or not file_exists(FLOWN_FILE):
        return
    with open(PATHS_FILE) as f:
        planned_dict = json.load(f)
    with open(FLOWN_FILE) as f:
        flown_dict = json.load(f)

    fig, ax = new_figure("Expected (planned) vs actual (flown) paths")
    draw_obstacles(ax)

    for key in planned_dict:
        n = int(key)
        color = DRONE_PLOT_COLORS[n]

        planned = np.array(planned_dict[key])
        ax.plot(planned[:, 0], planned[:, 1], color=color, linewidth=1.5,
                linestyle="--", zorder=4)

        if key not in flown_dict:
            print(f"  drone {n}: no flown path saved")
            continue
        flown = np.array(flown_dict[key])
        ax.plot(flown[:, 0], flown[:, 1], color=color, linewidth=2.5,
                alpha=0.6, zorder=5)

        # how far each flown spot is from the closest planned spot
        gaps = []
        for spot in flown:
            distances = np.linalg.norm(planned - spot, axis=1)
            gaps.append(distances.min())
        print(f"  drone {n}: flown path is at most {max(gaps):.2f} m "
              f"off the plan (avg {np.mean(gaps):.2f} m)")

    draw_starts_and_targets(ax)

    items = drone_legend_items()
    items.append(Line2D([], [], color="grey", linewidth=1.5, linestyle="--",
                        label="expected (planned)"))
    items.append(Line2D([], [], color="grey", linewidth=2.5, alpha=0.6,
                        label="actual (flown)"))
    put_legend_outside(ax, items)
    save_figure(fig, "expected_vs_actual_paths.png")


def load_slam_paths():
    """slam_paths.npy holds [true_path, estimated_path], each N x 2"""
    both = np.load(SLAM_FILE)
    true_path = both[0]
    estimated_path = both[1]
    return true_path, estimated_path


def slam_points_vs_obstacles():
    """diagnosis 1: do the mapped obstacle points line up with the real ones?"""
    if not file_exists(POINT_CLOUD_FILE):
        return
    pts: PointCloud = np.load(POINT_CLOUD_FILE)
    obst_pts = pts[pts[:, 2] >= FLOOR_CUTOFF]

    # for each obstacle point: how far is it from the nearest true obstacle?
    # (sample so this stays quick)
    rng = np.random.default_rng(0)
    sample_size = min(3000, len(obst_pts))
    sample = obst_pts[rng.choice(len(obst_pts), sample_size, replace=False)]
    misses = [distanceToNearestObstacle((x, y)) for x, y, _ in sample]
    misses = np.maximum(misses, 0)    # inside an obstacle counts as 0

    fig, ax = new_figure(f"SLAM check 1: mapped obstacle points vs true shapes "
                         f"(median miss {np.median(misses) * 100:.0f} cm)")
    ax.scatter(obst_pts[:, 0], obst_pts[:, 1], s=0.3, color=SCANNER_PLOT_COLOR,
               rasterized=True, zorder=2)
    draw_obstacles(ax, filled=False)

    items = [Line2D([], [], marker="o", color=SCANNER_PLOT_COLOR, linestyle="",
                    markersize=4, label="mapped point"),
             Rectangle((0, 0), 1, 1, fill=False, edgecolor=OBSTACLE_PLOT_COLOR,
                       label="true obstacle")]
    put_legend_outside(ax, items)

    print(f"  obstacle points off the true shapes: median "
          f"{np.median(misses):.3f} m, 95% within {np.percentile(misses, 95):.3f} m")
    save_figure(fig, "slam_points_vs_obstacles.png")


def slam_true_vs_estimated_path():
    """diagnosis 2: where the scanner really flew vs where it thought it was"""
    if not file_exists(SLAM_FILE):
        return
    true_path, estimated_path = load_slam_paths()

    fig, ax = new_figure("SLAM check 2: true path vs estimated path")
    draw_obstacles(ax, filled=True)
    ax.plot(true_path[:, 0], true_path[:, 1], color=SCANNER_PLOT_COLOR,
            linewidth=2.0, zorder=4, label="true path")
    ax.plot(estimated_path[:, 0], estimated_path[:, 1], color=ESTIMATE_PLOT_COLOR,
            linewidth=1.2, linestyle="--", zorder=5, label="estimated path")

    # mark the worst moment
    errors = np.linalg.norm(estimated_path - true_path, axis=1)
    worst = int(np.argmax(errors))
    ax.plot([true_path[worst, 0], estimated_path[worst, 0]],
            [true_path[worst, 1], estimated_path[worst, 1]],
            color="black", linewidth=2, zorder=6)
    ax.annotate(f"worst: {errors[worst]:.2f} m", true_path[worst], xytext=(8, 8),
                textcoords="offset points", fontsize=9, color=TEXT_COLOR)

    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=9)
    save_figure(fig, "slam_true_vs_estimated_path.png")


def slam_error_over_time():
    """how far off the scanner's position guess was, second by second"""
    if not file_exists(SLAM_FILE):
        return
    true_path, estimated_path = load_slam_paths()

    error_x = estimated_path[:, 0] - true_path[:, 0]
    error_y = estimated_path[:, 1] - true_path[:, 1]
    error = np.hypot(error_x, error_y)
    seconds = np.arange(len(error)) * SIM_DT

    avg_error = error.mean()
    worst_error = error.max()
    end_error = error[-1]

    fig, ax = plt.subplots(figsize=(12, 4.5))
    ax.set_title(f"SLAM position error over time: avg {avg_error:.2f} m, "
                 f"worst {worst_error:.2f} m, end {end_error:.2f} m",
                 loc="left", fontsize=13)

    ax.axhline(0, color="#bdbdbd", linewidth=1)
    ax.axhline(CELL_SIZE, color="#9e9e9e", linewidth=1, linestyle=":",
               label=f"one map cell ({CELL_SIZE} m)")

    ax.plot(seconds, error, color="black", linewidth=2, label="total error")
    ax.plot(seconds, error_x, color=DRONE_PLOT_COLORS[1], linewidth=1.2,
            label="x error")
    ax.plot(seconds, error_y, color=DRONE_PLOT_COLORS[2], linewidth=1.2,
            label="y error")

    ax.set_xlabel("time (s)", color=TEXT_COLOR)
    ax.set_ylabel("error (m)", color=TEXT_COLOR)
    ax.set_xlim(0, seconds[-1])
    ax.tick_params(colors=TEXT_COLOR)
    ax.grid(axis="y", color="#eeeeee")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False, fontsize=9)

    print(f"  SLAM error: avg {avg_error:.3f} m, worst {worst_error:.3f} m, "
          f"end {end_error:.3f} m")
    save_figure(fig, "slam_error_over_time.png")


if __name__ == "__main__":
    all_plots = [
        layout_grid_preview,
        expected_scan_preview,
        scan_preview,
        lidar_top_view,
        route_check,
        map_preview,
        paths_preview,
        expected_vs_actual_paths,
        slam_points_vs_obstacles,
        slam_true_vs_estimated_path,
        slam_error_over_time,
    ]
    for make_plot in all_plots:
        print(f"{make_plot.__name__}:")
        make_plot()