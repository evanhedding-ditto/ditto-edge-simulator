// Static world objects, drawn and nothing more.
//
// This is display truth, not simulation truth. Nothing here is published,
// replicated, or handed to a vehicle: no autopilot integrates against it, no
// adapter writes it, and no Ditto store holds it. A drone flies through one of
// these buildings quite happily. That is deliberate -- the simulator's ground
// rule is that simulator truth never reaches an autonomy decision, and the
// cheapest way to honour it is for the world to exist only in the viewer.
//
// Giving an object physical consequence is a separate piece of work, and it
// belongs on the autopilot side of the MAVLink boundary (the vehicle senses an
// obstacle through its own interface), never as a fact the viewer shares out.
//
// The file is read once at startup. A missing or malformed world is not an
// error: the viewer draws bare ground, exactly as it did before worlds existed.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <raylib.h>

namespace sim::world {

/// The viewer's default camera angles, which `Bounds::framing_span_m` assumes.
///
/// These must match the initial `yaw` and `pitch` in main.cpp. Framing depends
/// on both: the yaw decides that the footprint is seen corner-to-corner, and
/// the pitch sets how much a metre of height is worth against a metre of
/// ground. Neither is adjustable today beyond the user dragging the view, which
/// does not re-frame. If either default moves, or the reset view stops using
/// them, this is what has to move with it.
constexpr float kDefaultYawRad = 0.78F;
constexpr float kDefaultPitchRad = 0.55F;

enum class Shape { box, cylinder };

/// How an object is filled.
///
/// `solid` is the default so that a world file written before styles existed
/// renders exactly as it did. `wire` draws edges only, which is how you see
/// something on the far side of a wall. `translucent` fills with `opacity`.
enum class Style { solid, wire, translucent };

/// One static object, already converted into the viewer's frame.
///
/// Positions arrive as PX4 local NED offsets from the configured home position,
/// which is the same frame the `goto` and `orbit` commands are spelled in, so a
/// world file can be written straight off a demo script's coordinates.
struct Object
{
  std::string id;
  std::string label;
  Shape shape{Shape::box};
  float north_m{};
  float east_m{};
  float height_m{};
  /// Elevation of the underside. Zero stands the object on the ground plane;
  /// a raised value is how a storey above the first one is placed.
  float base_m{};
  Style style{Style::solid};
  float opacity{1.0F};
  float size_north_m{};   //< box only
  float size_east_m{};    //< box only
  float radius_m{};       //< cylinder only
  Color color{150, 158, 170, 255};
};

struct World
{
  std::string name;
  std::string path;
  float extent_m{};
  std::vector<Object> objects;

  bool empty() const { return objects.empty(); }
};

inline Color color_from(const nlohmann::json & object, const Color fallback)
{
  const auto entry = object.find("color_rgb");
  if (entry == object.end() || !entry->is_array() || entry->size() < 3) return fallback;
  const auto channel = [&](const std::size_t index) {
    const double value = entry->at(index).get<double>();
    return static_cast<std::uint8_t>(value < 0.0 ? 0.0 : (value > 255.0 ? 255.0 : value));
  };
  return {channel(0), channel(1), channel(2), 255};
}

inline bool positive_finite(const float value)
{
  return std::isfinite(value) && value > 0.0F;
}

/// Read a world file.
///
/// Tolerant on purpose, at two levels. A world that will not parse at all
/// yields no objects and the viewer draws bare ground, because refusing to
/// start over a misspelled decoration is worse than drawing no decoration. An
/// object that will not parse is skipped on its own and the rest of the world
/// still stands -- one wrong field in a large hand-written world file should
/// cost you that wall, not every wall.
inline World read(const std::string & path)
{
  World world;
  if (path.empty()) return world;
  try {
    std::ifstream input(path);
    nlohmann::json document;
    input >> document;
    world.path = path;
    world.name = document.value("name", "world");
    world.extent_m = document.value("extent_m", 0.0F);
    const auto objects = document.find("objects");
    if (objects == document.end() || !objects->is_array()) return world;
    for (const auto & entry : *objects) {
      // `continue` inside the try skips this object; so does a throw from a
      // field of the wrong JSON type. Both mean the same thing here.
      try {

        Object object;
        object.id = entry.value("id", "");
        object.label = entry.value("label", object.id);
        const std::string kind = entry.value("kind", "box");
        if (kind == "cylinder") {
          object.shape = Shape::cylinder;
          object.radius_m = entry.value("radius_m", 0.0F);
          if (!positive_finite(object.radius_m)) continue;
        } else if (kind == "box") {
          object.shape = Shape::box;
          object.size_north_m = entry.value("size_north_m", 0.0F);
          object.size_east_m = entry.value("size_east_m", 0.0F);
          if (!positive_finite(object.size_north_m) || !positive_finite(object.size_east_m)) continue;
        } else {
          continue;   // An object kind this build does not know how to draw.
        }
        object.north_m = entry.value("north_m", 0.0F);
        object.east_m = entry.value("east_m", 0.0F);
        object.height_m = entry.value("height_m", 0.0F);
        object.base_m = entry.value("base_m", 0.0F);
        if (!std::isfinite(object.north_m) || !std::isfinite(object.east_m)) continue;
        if (!std::isfinite(object.base_m)) continue;
        if (!positive_finite(object.height_m)) continue;
        const std::string style = entry.value("style", "solid");
        if (style == "wire") {
          object.style = Style::wire;
        } else if (style == "translucent") {
          object.style = Style::translucent;
        } else if (style != "solid") {
          continue;   // A style this build does not know how to draw.
        }
        object.opacity = entry.value("opacity", 1.0F);
        if (!std::isfinite(object.opacity)) object.opacity = 1.0F;
        object.opacity = object.opacity < 0.0F ? 0.0F : (object.opacity > 1.0F ? 1.0F : object.opacity);
        object.color = color_from(entry, object.color);
        world.objects.push_back(std::move(object));
    
      } catch (const std::exception &) {
        continue;
      }
    }
  } catch (const std::exception &) {
    world.objects.clear();
  }
  return world;
}

/// What the camera has to frame, and where it should point.
///
/// Not `extent_m`. The extent is the ground plane, deliberately drawn wider
/// than what stands on it, so framing on it opens too far out: the twenty-node
/// world puts 127 m of buildings inside a 240 m plane.
struct Bounds
{
  float north_span{};
  float east_span{};
  float centre_north_m{};
  float centre_east_m{};
  float floor_m{};
  float top_m{};
  bool valid{};

  /// Where the camera should point: the centre of the content, in all three
  /// axes. The target used to be a fixed {0, 3, 0}, which assumes the content
  /// is centred on the origin and lies near the ground. Neither holds once
  /// there is scenery: the twenty-node world's buildings are centred 9.75 m
  /// west of the origin, which was enough to push one off the bottom of the
  /// frame, and a three-storey building is off-centre vertically at 3 m.
  Vector3 target() const
  {
    // Viewer axes are (x, y, z) = (east, altitude, north).
    return {centre_east_m, (floor_m + top_m) * 0.5F, centre_north_m};
  }

  /// The span to frame on.
  ///
  /// Horizontally this is the diagonal, not the longest side: the default view
  /// is yawed 45 degrees, so the footprint presents corner-to-corner across the
  /// screen. Framing on the longest side looks sufficient and is not -- it put
  /// the near cylinder of the twenty-node world off the bottom of the frame,
  /// with the camera apparently far enough away.
  ///
  /// Height is kept out of that diagonal and compared against it instead,
  /// because it does not foreshorten the same way: the view is pitched down by
  /// `kDefaultPitchRad`, so a metre of ground shrinks to sin(pitch) while a
  /// metre of height projects at cos(pitch). The ratio -- cot(pitch), about
  /// 1.63 at the current 0.55 rad -- is what a metre of height costs in
  /// footprint, so it is computed from the pitch rather than written down.
  /// Folding height into a 3D diagonal double-counts it against a wide, flat
  /// world; comparing it lets a tower on a small plot win, which is the only
  /// case where height should decide the distance.
  float framing_span_m() const
  {
    // Corner-to-corner, because the view is yawed (see kDefaultYawRad).
    const float footprint_diagonal = std::hypot(north_span, east_span);
    const float height_weight =
      std::cos(kDefaultPitchRad) / std::sin(kDefaultPitchRad);
    return std::max(footprint_diagonal, (top_m - floor_m) * height_weight);
  }
};

inline Bounds content_bounds(const World & world)
{
  Bounds bounds;
  float north_low = 0.0F, north_high = 0.0F, east_low = 0.0F, east_high = 0.0F;
  for (const auto & object : world.objects) {
    const float half_north = object.shape == Shape::box
      ? object.size_north_m * 0.5F : object.radius_m;
    const float half_east = object.shape == Shape::box
      ? object.size_east_m * 0.5F : object.radius_m;
    if (!bounds.valid) {
      north_low = object.north_m - half_north;
      north_high = object.north_m + half_north;
      east_low = object.east_m - half_east;
      east_high = object.east_m + half_east;
      bounds.floor_m = object.base_m;
      bounds.top_m = object.base_m + object.height_m;
      bounds.valid = true;
      continue;
    }
    north_low = std::min(north_low, object.north_m - half_north);
    north_high = std::max(north_high, object.north_m + half_north);
    east_low = std::min(east_low, object.east_m - half_east);
    east_high = std::max(east_high, object.east_m + half_east);
    bounds.floor_m = std::min(bounds.floor_m, object.base_m);
    bounds.top_m = std::max(bounds.top_m, object.base_m + object.height_m);
  }
  bounds.north_span = north_high - north_low;
  bounds.east_span = east_high - east_low;
  bounds.centre_north_m = (north_low + north_high) * 0.5F;
  bounds.centre_east_m = (east_low + east_high) * 0.5F;
  return bounds;
}

/// Draw the world. Call inside BeginMode3D, before the fleet, so a drone
/// passing behind a building still reads correctly against it.
///
/// `camera_position` exists only to order translucent objects. raylib does no
/// depth sorting, so an alpha-filled surface drawn before the geometry behind
/// it writes depth and hides it -- a robot vanishes through a floor slab from
/// half the camera angles. Opaque objects go down first and translucent ones
/// follow, farthest first, which is correct for any arrangement that does not
/// interpenetrate.
inline void draw(const World & world, const Vector3 camera_position)
{
  const auto shape_of = [](const Object & object, const Color fill, const bool wires_only) {
    // Viewer axes are (x, y, z) = (east, altitude, north). An object rises from
    // its underside, so the centre sits half a height above `base_m`.
    const Vector3 base{object.east_m, object.base_m, object.north_m};
    const Vector3 centre{object.east_m, object.base_m + object.height_m * 0.5F, object.north_m};
    if (object.shape == Shape::box) {
      const Vector3 size{object.size_east_m, object.height_m, object.size_north_m};
      if (!wires_only) DrawCubeV(centre, size, fill);
      DrawCubeWiresV(centre, size, wires_only ? fill : Fade(BLACK, 0.35F));
    } else {
      // Wires over a filled cylinder are an outline, so they stay sparse --
      // one vertical per slice at the fill's 20 reads as hatching. A cylinder
      // that is only wires gets more, because there the wires are the object.
      const int wire_slices = wires_only ? 16 : 8;
      if (!wires_only) {
        DrawCylinder(base, object.radius_m, object.radius_m, object.height_m, 20, fill);
      }
      DrawCylinderWires(base, object.radius_m, object.radius_m, object.height_m, wire_slices,
        wires_only ? fill : Fade(BLACK, 0.35F));
    }
  };

  std::vector<const Object *> translucent;
  for (const auto & object : world.objects) {
    switch (object.style) {
      case Style::solid: shape_of(object, object.color, false); break;
      case Style::wire: shape_of(object, object.color, true); break;
      case Style::translucent: translucent.push_back(&object); break;
    }
  }
  if (translucent.empty()) return;

  const auto distance_squared = [&](const Object * object) {
    const float dx = object->east_m - camera_position.x;
    const float dy = object->base_m + object->height_m * 0.5F - camera_position.y;
    const float dz = object->north_m - camera_position.z;
    return dx * dx + dy * dy + dz * dz;
  };
  std::sort(translucent.begin(), translucent.end(),
    [&](const Object * left, const Object * right) {
      return distance_squared(left) > distance_squared(right);
    });
  for (const Object * object : translucent) {
    shape_of(*object, Fade(object->color, object->opacity), false);
  }
}

}   // namespace sim::world
