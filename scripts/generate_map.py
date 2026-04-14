#!/usr/bin/env python3
"""
Generate an occupancy grid map (PGM + YAML) directly from the apartment.world SDF file.
This avoids the need for manual SLAM mapping by reading wall/furniture geometry
and rendering them as occupied cells.
"""

import numpy as np
from PIL import Image
import xml.etree.ElementTree as ET
import math
import os

# ── Map parameters ───────────────────────────────────────────────────────────
RESOLUTION = 0.05       # meters per pixel
PADDING = 2.0           # extra meters around the world bounds
WALL_INFLATE = 0.0      # optional inflation in meters (0 = exact geometry)

# World bounds (from the apartment.world: walls at ±5)
WORLD_X_MIN = -5.0 - PADDING
WORLD_X_MAX =  5.0 + PADDING
WORLD_Y_MIN = -5.0 - PADDING
WORLD_Y_MAX =  5.0 + PADDING

# Map dimensions in pixels
MAP_WIDTH  = int((WORLD_X_MAX - WORLD_X_MIN) / RESOLUTION)
MAP_HEIGHT = int((WORLD_Y_MAX - WORLD_Y_MIN) / RESOLUTION)

# Origin: bottom-left corner of the map in world coordinates
ORIGIN_X = WORLD_X_MIN
ORIGIN_Y = WORLD_Y_MIN


def world_to_pixel(wx, wy):
    """Convert world coordinates to pixel coordinates."""
    px = int((wx - ORIGIN_X) / RESOLUTION)
    # Image row 0 is top, but world Y increases up. 
    # In PGM for ROS, row 0 = bottom of world (origin), so we flip Y.
    py = MAP_HEIGHT - 1 - int((wy - ORIGIN_Y) / RESOLUTION)
    return px, py


def draw_box(grid, cx, cy, yaw, sx, sy):
    """
    Draw a filled rotated box on the grid.
    cx, cy = center in world coords
    yaw = rotation in radians
    sx, sy = size (full width, full depth) in meters
    """
    cos_a = math.cos(yaw)
    sin_a = math.sin(yaw)
    half_x = sx / 2.0 + WALL_INFLATE
    half_y = sy / 2.0 + WALL_INFLATE

    # Corners of the box in local frame
    corners_local = [
        (-half_x, -half_y),
        ( half_x, -half_y),
        ( half_x,  half_y),
        (-half_x,  half_y),
    ]

    # Transform to world frame
    corners_world = []
    for lx, ly in corners_local:
        wx = cx + lx * cos_a - ly * sin_a
        wy = cy + lx * sin_a + ly * cos_a
        corners_world.append((wx, wy))

    # Rasterize: scan all pixels in bounding box and check if inside polygon
    wxs = [c[0] for c in corners_world]
    wys = [c[1] for c in corners_world]
    min_wx, max_wx = min(wxs), max(wxs)
    min_wy, max_wy = min(wys), max(wys)

    px_min, py_max = world_to_pixel(min_wx, min_wy)
    px_max, py_min = world_to_pixel(max_wx, max_wy)

    px_min = max(0, px_min - 1)
    px_max = min(MAP_WIDTH - 1, px_max + 1)
    py_min = max(0, py_min - 1)
    py_max = min(MAP_HEIGHT - 1, py_max + 1)

    for py in range(py_min, py_max + 1):
        for px in range(px_min, px_max + 1):
            # Convert pixel back to world
            wx = ORIGIN_X + (px + 0.5) * RESOLUTION
            wy = ORIGIN_Y + (MAP_HEIGHT - 1 - py + 0.5) * RESOLUTION

            # Check if point is inside the rotated box
            # Transform point to box-local frame
            dx = wx - cx
            dy = wy - cy
            local_x =  dx * cos_a + dy * sin_a
            local_y = -dx * sin_a + dy * cos_a

            if abs(local_x) <= half_x and abs(local_y) <= half_y:
                grid[py, px] = 0  # occupied = black


def parse_world_and_draw(world_file, grid):
    """Parse the SDF world file and draw all static models as obstacles."""
    tree = ET.parse(world_file)
    root = tree.getroot()

    world = root.find('world')
    if world is None:
        print("ERROR: No <world> element found!")
        return

    for model in world.findall('model'):
        name = model.get('name', '')

        # Skip non-obstacle models
        if name in ('Ground Plane', 'Sun', 'ground_plane', 'sun'):
            continue

        static = model.find('static')
        if static is None or static.text.strip().lower() != 'true':
            continue

        # Get model pose
        pose_elem = model.find('pose')
        if pose_elem is not None:
            pose_vals = [float(v) for v in pose_elem.text.strip().split()]
            mx, my, mz = pose_vals[0], pose_vals[1], pose_vals[2]
            yaw = pose_vals[5] if len(pose_vals) > 5 else 0.0
        else:
            mx, my, mz, yaw = 0, 0, 0, 0

        # Find geometry in any link
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                box = collision.find('.//box/size')
                if box is not None:
                    size_vals = [float(v) for v in box.text.strip().split()]
                    sx, sy, sz = size_vals[0], size_vals[1], size_vals[2]

                    print(f"  Drawing '{name}': pos=({mx:.1f}, {my:.1f}) "
                          f"size=({sx:.1f}x{sy:.1f}) yaw={math.degrees(yaw):.0f}°")
                    draw_box(grid, mx, my, yaw, sx, sy)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = os.path.dirname(script_dir)
    world_file = os.path.join(ws_dir, 'src', 'tb3_delivery', 'worlds', 'apartment.world')
    maps_dir = os.path.join(ws_dir, 'src', 'tb3_delivery', 'maps')

    os.makedirs(maps_dir, exist_ok=True)

    print(f"Map size: {MAP_WIDTH} x {MAP_HEIGHT} pixels")
    print(f"Resolution: {RESOLUTION} m/px")
    print(f"World bounds: X[{WORLD_X_MIN}, {WORLD_X_MAX}] Y[{WORLD_Y_MIN}, {WORLD_Y_MAX}]")
    print(f"Origin: ({ORIGIN_X}, {ORIGIN_Y})")
    print()

    # Initialize grid: 254 = free (white), 0 = occupied (black)
    grid = np.full((MAP_HEIGHT, MAP_WIDTH), 254, dtype=np.uint8)

    print("Parsing world file and drawing obstacles...")
    parse_world_and_draw(world_file, grid)

    # ── Save PGM ─────────────────────────────────────────────────────────────
    pgm_path = os.path.join(maps_dir, 'apartment_map.pgm')
    img = Image.fromarray(grid, mode='L')
    img.save(pgm_path)
    print(f"\n✅ Saved: {pgm_path}")

    # ── Save YAML ────────────────────────────────────────────────────────────
    yaml_path = os.path.join(maps_dir, 'apartment_map.yaml')
    yaml_content = f"""image: apartment_map.pgm
mode: trinary
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)
    print(f"✅ Saved: {yaml_path}")

    # ── Stats ────────────────────────────────────────────────────────────────
    total = grid.size
    occupied = np.sum(grid < 50)
    free = np.sum(grid > 200)
    print(f"\n📊 Map stats:")
    print(f"   Total pixels:    {total}")
    print(f"   Free (white):    {free} ({100*free/total:.1f}%)")
    print(f"   Occupied (black): {occupied} ({100*occupied/total:.1f}%)")
    print(f"\n🎉 Map generated successfully! Ready for navigation.")


if __name__ == '__main__':
    main()
