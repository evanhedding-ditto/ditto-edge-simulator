#!/usr/bin/env python3
"""Turn a 1 km OpenStreetMap extract around Park MGM into a static viewer world.

Fetch the source with:
  curl 'https://api.openstreetmap.org/api/0.6/map?bbox=-115.178727,36.099545,-115.167609,36.108528' -o park-mgm.osm
Then run: python3 worlds/build-park-mgm.py park-mgm.osm
The generated map is © OpenStreetMap contributors, ODbL 1.0.
If park-mgm-1km-aerial.png is present beside the output, it textures the ground
and roofs. Its source is USGS The National Map, USGSNAIPPlus ImageServer,
exportImage, bbox=BBOX, bboxSR=imageSR=4326, size=2048,2048 (public domain).
"""

import argparse
import json
import math
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

LAT, LON = 36.1040361, -115.1731679  # Strip roadway by Park MGM; open launch point
HALF_M = 500.0
METRES_PER_DEGREE = 111319.49079327357  # Matches the MAVLink viewer.
LON_SCALE = METRES_PER_DEGREE * math.cos(math.radians(LAT))
BBOX = (LON - HALF_M / LON_SCALE, LAT - HALF_M / METRES_PER_DEGREE,
        LON + HALF_M / LON_SCALE, LAT + HALF_M / METRES_PER_DEGREE)
COLORS = {
    "road": (0.20, 0.24, 0.27),
    "local_road": (0.29, 0.32, 0.34),
    "wall": (0.39, 0.43, 0.46),
    "roof": (0.62, 0.65, 0.66),
    "landmark_wall": (0.32, 0.39, 0.44),
    "landmark_roof": (0.50, 0.56, 0.60),
}
ROAD_WIDTH = {
    "primary": 16.0, "secondary": 12.0, "tertiary": 9.0,
    "unclassified": 7.0, "pedestrian": 8.0,
}
LANDMARK_NAMES = {
    "Park MGM": "PARK MGM",
    "Aria Resort & Casino": "ARIA",
    "New York New York Hotel and Casino": "NEW YORK-NEW YORK",
    "T-Mobile Arena": "T-MOBILE ARENA",
    "Hotel MGM Grand Las Vegas": "MGM GRAND",
}


def coords(lat, lon):
    return ((lon - LON) * LON_SCALE, (lat - LAT) * METRES_PER_DEGREE)


def height(tags):
    raw = tags.get("height", "")
    match = re.fullmatch(r"\s*(\d+(?:\.\d+)?)\s*(m|ft|')?\s*", raw)
    if match:
        metres = float(match[1]) * (0.3048 if match[2] in ("ft", "'") else 1.0)
        if 1.0 <= metres <= 500.0:
            return metres
    try:
        levels = float(tags.get("building:levels", ""))
        if 0.5 <= levels <= 150:
            return levels * 3.2
    except ValueError:
        pass
    return 14.0 if tags.get("tourism") == "hotel" else 9.0


def signed_area(points):
    return sum(a[0] * b[1] - b[0] * a[1]
               for a, b in zip(points, points[1:] + points[:1])) * 0.5


def point_in_triangle(p, a, b, c):
    def cross(u, v, w):
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    return min(cross(a, b, p), cross(b, c, p), cross(c, a, p)) >= -1e-7


def triangles(points):
    """Ear clip a simple counter-clockwise footprint; skip invalid OSM rings."""
    remaining = list(range(len(points)))
    result = []
    for _ in range(len(points) * len(points)):
        if len(remaining) <= 3:
            break
        for j in range(len(remaining)):
            a, b, c = (remaining[(j - 1) % len(remaining)], remaining[j],
                       remaining[(j + 1) % len(remaining)])
            pa, pb, pc = points[a], points[b], points[c]
            cross = (pb[0] - pa[0]) * (pc[1] - pa[1]) - (pb[1] - pa[1]) * (pc[0] - pa[0])
            if cross <= 1e-5 or any(point_in_triangle(points[k], pa, pb, pc)
                                    for k in remaining if k not in (a, b, c)):
                continue
            result.append((a, b, c))
            del remaining[j]
            break
        else:
            return []
    if len(remaining) == 3:
        result.append(tuple(remaining))
    return result


def clip(a, b):
    """Liang-Barsky clip of one road segment to the 1 km square."""
    dx, dz = b[0] - a[0], b[1] - a[1]
    lo, hi = 0.0, 1.0
    for p, q in ((-dx, a[0] + HALF_M), (dx, HALF_M - a[0]),
                 (-dz, a[1] + HALF_M), (dz, HALF_M - a[1])):
        if p == 0:
            if q < 0:
                return None
        elif p < 0:
            lo = max(lo, q / p)
        else:
            hi = min(hi, q / p)
        if lo > hi:
            return None
    return ((a[0] + lo * dx, a[1] + lo * dz),
            (a[0] + hi * dx, a[1] + hi * dz))


def clip_polygon(points):
    for axis, boundary, inside in ((0, -HALF_M, 1), (0, HALF_M, -1),
                                   (1, -HALF_M, 1), (1, HALF_M, -1)):
        result = []
        for a, b in zip(points, points[1:] + points[:1]):
            a_in = (a[axis] - boundary) * inside >= 0
            b_in = (b[axis] - boundary) * inside >= 0
            if a_in != b_in:
                fraction = (boundary - a[axis]) / (b[axis] - a[axis])
                result.append((a[0] + fraction * (b[0] - a[0]),
                               a[1] + fraction * (b[1] - a[1])))
            if b_in:
                result.append(b)
        points = result
        if not points:
            break
    return points


def build(source, output):
    root = ET.parse(source).getroot()
    nodes = {n.get("id"): coords(float(n.get("lat")), float(n.get("lon")))
             for n in root.findall("node")}
    ways = {w.get("id"): w for w in root.findall("way")}
    vertices = []
    faces = defaultdict(list)
    counts = defaultdict(int)
    landmarks = {}
    max_height = 0.0
    aerial = output.with_name(output.name + "-aerial.png")
    has_aerial = aerial.is_file()

    def vertex(x, y, z):
        vertices.append((x, y, z))
        return len(vertices)

    def road(a, b, width, material):
        clipped = clip(a, b)
        if not clipped:
            return
        a, b = clipped
        dx, dz = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dz)
        if length < 0.1:
            return
        side = (-dz * width / (2 * length), dx * width / (2 * length))
        # Small lift keeps pavement clear of the ground plane's depth buffer.
        v = [vertex(x, 0.15, z) for x, z in (
            (a[0] + side[0], a[1] + side[1]),
            (a[0] - side[0], a[1] - side[1]),
            (b[0] - side[0], b[1] - side[1]),
            (b[0] + side[0], b[1] + side[1]))]
        faces[material].extend(((v[0], v[2], v[1]), (v[0], v[3], v[2])))
        counts["road_segments"] += 1

    def building(points, tags):
        nonlocal max_height
        points = [p for i, p in enumerate(points) if i == 0 or p != points[i - 1]]
        if len(points) > 2 and points[-1] == points[0]:
            points.pop()
        points = clip_polygon(points)
        if len(points) < 3:
            return
        if signed_area(points) < 0:
            points.reverse()
        if signed_area(points) < 1.0:
            return
        roof_faces = triangles(points)
        if not roof_faces:
            counts["skipped_footprints"] += 1
            return
        top = height(tags)
        base = 0.0
        try:
            base = float(tags.get("min_height", 0))
        except ValueError:
            pass
        if not math.isfinite(base) or base < 0 or base >= top:
            base = 0.0
        name = tags.get("name", "")
        landmark = name == "Park MGM" and top > 40.0
        wall = "landmark_wall" if landmark else "wall"
        roof = "aerial" if has_aerial else ("landmark_roof" if landmark else "roof")
        bottom = [vertex(x, base, z) for x, z in points]
        upper = [vertex(x, top, z) for x, z in points]
        for i in range(len(points)):
            j = (i + 1) % len(points)
            faces[wall].extend(((bottom[i], upper[i], upper[j]),
                                (bottom[i], upper[j], bottom[j])))
        faces[roof].extend((upper[c], upper[b], upper[a]) for a, b, c in roof_faces)
        max_height = max(max_height, top)
        counts["buildings"] += 1
        if name in LANDMARK_NAMES and (name not in landmarks or landmarks[name][2] < top):
            landmarks[name] = (sum(p[0] for p in points) / len(points),
                               sum(p[1] for p in points) / len(points), top)

    for way in ways.values():
        tags = {t.get("k"): t.get("v") for t in way.findall("tag")}
        points = [nodes[n.get("ref")] for n in way.findall("nd") if n.get("ref") in nodes]
        kind = tags.get("highway")
        if not has_aerial and kind in ROAD_WIDTH and tags.get("tunnel") != "yes":
            material = "road" if kind in ("primary", "secondary") else "local_road"
            for a, b in zip(points, points[1:]):
                road(a, b, ROAD_WIDTH[kind], material)
        if "building" in tags or "building:part" in tags:
            building(points, tags)

    for relation in root.findall("relation"):
        tags = {t.get("k"): t.get("v") for t in relation.findall("tag")}
        if tags.get("type") != "multipolygon" or "building" not in tags:
            continue
        for member in relation.findall("member"):
            if member.get("role") != "outer" or member.get("ref") not in ways:
                continue
            way = ways[member.get("ref")]
            points = [nodes[n.get("ref")] for n in way.findall("nd") if n.get("ref") in nodes]
            building(points, tags)

    output.parent.mkdir(parents=True, exist_ok=True)
    if has_aerial:
        corners = ((-HALF_M, -HALF_M), (-HALF_M, HALF_M),
                   (HALF_M, HALF_M), (HALF_M, -HALF_M))
        ground = [vertex(x, -0.01, z) for x, z in corners]
        faces["aerial"].extend(((ground[0], ground[1], ground[2]),
                                (ground[0], ground[2], ground[3])))
    mtl = output.with_suffix(".mtl")
    obj = output.with_suffix(".obj")
    world = output.with_suffix(".json")
    mtl.write_text("".join(
        f"newmtl {name}\nKd {r} {g} {b}\nKa {r} {g} {b}\n\n"
        for name, (r, g, b) in COLORS.items()) +
        (f"newmtl aerial\nKd 1 1 1\nKa 1 1 1\nmap_Kd {aerial.name}\n\n"
         if has_aerial else ""))
    with obj.open("w") as file:
        file.write(f"# © OpenStreetMap contributors, ODbL 1.0\nmtllib {mtl.name}\n")
        # The viewer's z is south (see view_point in viewer/src/world.hpp), and
        # negating z mirrors the mesh, so every face below is written reversed.
        for x, y, z in vertices:
            file.write(f"v {x:.3f} {y:.3f} {-z:.3f}\n")
        if has_aerial:
            for x, _, z in vertices:
                file.write(f"vt {(x + HALF_M) / (2 * HALF_M):.6f} "
                           f"{(z + HALF_M) / (2 * HALF_M):.6f}\n")
        for material, triangles_ in faces.items():
            file.write(f"usemtl {material}\n")
            for a, b, c in triangles_:
                file.write(f"f {a}/{a} {c}/{c} {b}/{b}\n" if has_aerial
                           else f"f {a} {c} {b}\n")
    world.write_text(json.dumps({
        "name": "Park MGM / Las Vegas Strip, 1 km",
        "extent_m": 1000,
        "view_distance_m": 550,
        "view_target_height_m": 20,
        "model": obj.name,
        "bounds": [-500, 500, -500, 500, 0, max_height],
        "attribution": "Map: OpenStreetMap (ODbL) | Aerial: USGS The National Map",
        "landmarks": [
            {"name": LANDMARK_NAMES[name], "east_m": east, "north_m": north,
             "height_m": max(top + 10.0, 45.0)}
            for name, (east, north, top) in landmarks.items()
        ],
        "origin": {"lat": LAT, "lon": LON},
        "source": "https://api.openstreetmap.org/api/0.6/map?bbox=" +
                  ",".join(f"{coordinate:.6f}" for coordinate in BBOX),
        "aerial_source": (
            "https://imagery.nationalmap.gov/arcgis/rest/services/"
            "USGSNAIPPlus/ImageServer/exportImage" if has_aerial else "")
    }, indent=2) + "\n")
    print(json.dumps(counts, sort_keys=True), "max_height_m", round(max_height))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("osm", type=Path, help="OpenStreetMap XML map extract")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).with_name("park-mgm-1km"))
    args = parser.parse_args()
    build(args.osm, args.output)
