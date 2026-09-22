#!/usr/bin/env python3
"""Regenerate the drawable `objects` array of a factory world from its `building`.

The `building` section is authored truth: levels, atrium, rooms, doorways. The
`objects` array is what the viewer draws, and it is derived -- floor slabs are
the footprint minus the atrium, walls are the room edges with doorway apertures
cut out, railings are the edges that face the void. Nothing hand-maintains it.

This exists because the two sections must agree. `building` is read by
tools/factory_fleet.cpp to decide where a robot may drive; `objects` is read by
the viewer to decide what a person sees. If they drift, the picture lies about
where the robots can go -- the robot walks through a wall that was only ever
drawn in the wrong place, and the bug looks like a motion bug for as long as it
takes someone to think of checking the geometry.

Walls are classified by WHAT IS ON EACH SIDE rather than merely traced from
room bounds, because three different things now share one edge: a wall between
two rooms, an open join between two balcony segments, and a railing where a
balcony meets the void. Tracing bounds alone cannot tell them apart, and a
balcony that reads as four corridors is not a walkway.

It is a build-time authoring tool, not a runtime dependency: the simulator is
bash and C++, and nothing in a run shells out to Python. Re-run it by hand after
editing the building.

    python3 worlds/derive-factory-objects.py worlds/factory-three-level.json
"""

import json
import sys

WALL_COLOR = [96, 112, 136]
RAILING_COLOR = [128, 140, 158]
FLOOR_COLOR = [128, 132, 138]
LIFT_COLOR = [176, 138, 110]
EPS = 1e-4

# What occupies the space beside a wall segment. `VOID` is the atrium, `OUTSIDE`
# is the world beyond the footprint.
VOID = "void"
OUTSIDE = "outside"


def occupant(rooms, atrium, north, east):
    """What is at this point on this level: a room id, VOID, or OUTSIDE."""
    for room in rooms:
        if (room["north_min_m"] - EPS <= north <= room["north_max_m"] + EPS
                and room["east_min_m"] - EPS <= east <= room["east_max_m"] + EPS):
            return room["id"]
    if (atrium["north_min_m"] <= north <= atrium["north_max_m"]
            and atrium["east_min_m"] <= east <= atrium["east_max_m"]):
        return VOID
    return OUTSIDE


def classify(a, b, circulation):
    """What to build between two neighbouring occupants.

    Returns "wall", "railing", or None for nothing at all.
    """
    if a == b:
        return None                       # Same thing both sides: not an edge.
    if a == OUTSIDE and b == OUTSIDE:
        return None
    if VOID in (a, b):
        # A balcony edge facing the void gets a railing. The void never touches
        # anything else, since the balcony rings it completely -- but if the
        # geometry were ever changed so that it did, a wall is the safe answer.
        other = b if a == VOID else a
        return "railing" if other in circulation else "wall"
    if a in circulation and b in circulation:
        return None                       # The walkway is continuous.
    return "wall"


def segments(rooms, atrium, circulation, axis, at, breakpoints):
    """Classified runs along one wall line, merged where the classification holds.

    `axis` is the coordinate the line is constant in. Walking the breakpoints
    rather than sampling means a run is split exactly where its neighbour
    changes -- the north balcony edge, for instance, is railing in the middle
    where it faces the void and open at both ends where it meets the side
    balconies.
    """
    runs = []
    for low, high in zip(breakpoints, breakpoints[1:]):
        if high - low < EPS:
            continue
        middle = (low + high) / 2
        if axis == "east":
            before = occupant(rooms, atrium, middle, at - EPS * 10)
            after = occupant(rooms, atrium, middle, at + EPS * 10)
        else:
            before = occupant(rooms, atrium, at - EPS * 10, middle)
            after = occupant(rooms, atrium, at + EPS * 10, middle)
        kind = classify(before, after, circulation)
        if kind is None:
            continue
        if runs and runs[-1][0] == kind and abs(runs[-1][2] - low) < EPS:
            runs[-1][2] = high
        else:
            runs.append([kind, low, high])
    return runs


def subtract(low, high, holes, min_length):
    """Cut the doorway apertures out of a run, dropping stubs too short to read
    as a doorjamb rather than as an artefact."""
    pieces = [[low, high]]
    for hole_low, hole_high in holes:
        nxt = []
        for a, b in pieces:
            if hole_high <= a or hole_low >= b:
                nxt.append([a, b])
                continue
            if hole_low > a:
                nxt.append([a, hole_low])
            if hole_high < b:
                nxt.append([hole_high, b])
        pieces = nxt
    return [p for p in pieces if p[1] - p[0] > min_length]


def derive(world):
    b = world["building"]
    rooms = b["rooms"]
    by_id = {r["id"]: r for r in rooms}
    circulation = {r["id"] for r in rooms if r.get("circulation")}
    atrium = b["atrium"]
    fp = b["footprint"]
    thickness = b["wall_thickness_m"]
    floor_thickness = b["floor_thickness_m"]
    door = b["door_width_m"]
    wall_height = b["floor_to_floor_m"] - floor_thickness
    railing_height = b["railing_height_m"]

    objects = []

    def slab(level_index, label, base, north_low, north_high, east_low, east_high, part):
        objects.append({
            "id": f"floor-{level_index}-{part}", "label": label, "kind": "box",
            "north_m": (north_low + north_high) / 2, "east_m": (east_low + east_high) / 2,
            "size_north_m": north_high - north_low, "size_east_m": east_high - east_low,
            "base_m": base - floor_thickness, "height_m": floor_thickness,
            "style": "solid", "color_rgb": FLOOR_COLOR,
        })

    for level in b["levels"]:
        index, base = level["index"], level["base_m"]
        here = [r for r in rooms if r["level"] == index]

        # The slab is the footprint MINUS the atrium, so it is emitted as four
        # bands around the void rather than one box. A box cannot have a hole in
        # it, and the hole is the whole point.
        label = f"{level['label']} floor"
        slab(index, label, base, atrium["north_max_m"], fp["north_max_m"],
             fp["east_min_m"], fp["east_max_m"], "n")
        slab(index, label, base, fp["north_min_m"], atrium["north_min_m"],
             fp["east_min_m"], fp["east_max_m"], "s")
        slab(index, label, base, atrium["north_min_m"], atrium["north_max_m"],
             fp["east_min_m"], atrium["east_min_m"], "w")
        slab(index, label, base, atrium["north_min_m"], atrium["north_max_m"],
             atrium["east_max_m"], fp["east_max_m"], "e")

        for axis in ("east", "north"):
            span = "north" if axis == "east" else "east"

            # Every coordinate at which an edge could start or stop.
            lines = sorted({r[f"{axis}_{side}_m"] for r in here for side in ("min", "max")}
                           | {atrium[f"{axis}_min_m"], atrium[f"{axis}_max_m"]})
            breakpoints = sorted({r[f"{span}_{side}_m"] for r in here for side in ("min", "max")}
                                 | {atrium[f"{span}_min_m"], atrium[f"{span}_max_m"]})

            holes = {}
            for d in b["doorways"]:
                first, second = (by_id[i] for i in d["between"])
                if first["level"] != index:
                    continue
                shares_north = (abs(first["north_max_m"] - second["north_min_m"]) < EPS
                                or abs(second["north_max_m"] - first["north_min_m"]) < EPS)
                if ((axis == "north") if shares_north else (axis == "east")):
                    # A doorway may declare its own width. The balcony corner
                    # joins use the full shared edge, which matters to the fleet
                    # even though no wall is drawn there to cut.
                    width = d.get("width_m", door)
                    holes.setdefault(d[f"{axis}_m"], []).append(
                        [d[f"{span}_m"] - width / 2, d[f"{span}_m"] + width / 2])

            for at in lines:
                for kind, low, high in segments(
                        here, atrium, circulation, axis, at, breakpoints):
                    height = railing_height if kind == "railing" else wall_height
                    colour = RAILING_COLOR if kind == "railing" else WALL_COLOR
                    for piece_low, piece_high in subtract(
                            low, high, holes.get(at, []), thickness):
                        centre = (piece_low + piece_high) / 2
                        objects.append({
                            "id": f"{kind}-{index}-{axis}{at:g}-{piece_low:g}".replace(".", "p"),
                            "label": "", "kind": "box",
                            "north_m": centre if axis == "east" else at,
                            "east_m": at if axis == "east" else centre,
                            "size_north_m": (piece_high - piece_low) if axis == "east" else thickness,
                            "size_east_m": thickness if axis == "east" else (piece_high - piece_low),
                            "base_m": base, "height_m": height,
                            "style": "wire", "color_rgb": colour,
                        })

    lift = b["lift"]
    objects.append({
        "id": lift["id"], "label": "Lift", "kind": "box",
        "north_m": lift["north_m"], "east_m": lift["east_m"],
        "size_north_m": lift["size_north_m"], "size_east_m": lift["size_east_m"],
        "base_m": 0.0, "height_m": b["levels"][-1]["base_m"] + wall_height,
        "style": "wire", "color_rgb": LIFT_COLOR,
    })
    return objects


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    with open(path) as handle:
        text = handle.read()
    objects = derive(json.loads(text))

    # Splice the array in as text rather than re-serialising the document. A
    # round-trip through json.dump would reflow the `building` section, and that
    # section is read by people: the room bounds are laid out in aligned columns
    # so a mistyped coordinate is visible on the page.
    marker = '\n  "objects":'
    head = text[:text.index(marker)] if marker in text else text.rstrip()[:-1]
    head = head.rstrip().rstrip(",")
    rows = [json.dumps(entry, separators=(", ", ": ")) for entry in objects]
    body = '  "objects": [\n' + ",\n".join("    " + row for row in rows) + "\n  ]"
    with open(path, "w") as handle:
        handle.write(head + ",\n\n" + body + "\n}\n")

    counts = {}
    for entry in objects:
        counts[entry["id"].split("-")[0]] = counts.get(entry["id"].split("-")[0], 0) + 1
    print(f"{path}: {len(objects)} objects " +
          ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))


if __name__ == "__main__":
    main()
