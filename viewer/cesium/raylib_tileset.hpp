#pragma once

#include <raylib.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim::cesium {

class RaylibTileset final {
public:
  RaylibTileset(double latitude_degrees, double longitude_degrees, double altitude_m,
      double radius_m, std::string token);
  ~RaylibTileset();
  RaylibTileset(const RaylibTileset&) = delete;
  RaylibTileset& operator=(const RaylibTileset&) = delete;

  /// Selects tiles for one view. View 0 is the window; each other view -- a
  /// vehicle camera -- keeps its own selection, drawn with draw(view).
  void update(const Camera3D& camera, int width, int height, float delta_seconds, std::size_t view = 0);
  void draw(std::size_t view = 0) const;
  /// True once the tiles the window's view wants have all loaded.
  bool idle() const noexcept;
  const std::string& status() const noexcept;
  const std::string& attribution() const noexcept;
  /// Sample the rendered 3D Tiles surface under fixed local (east, north) sites.
  void sample_surfaces(const std::vector<Vector2>& sites);
  std::optional<float> surface_height(std::size_t site) const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};

} // namespace sim::cesium
