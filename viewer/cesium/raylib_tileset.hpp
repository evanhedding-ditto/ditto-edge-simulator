#pragma once

#include <raylib.h>

#include <memory>
#include <string>

namespace sim::cesium {

class RaylibTileset final {
public:
  RaylibTileset(double latitude_degrees, double longitude_degrees, double altitude_m,
      double radius_m, std::string token);
  ~RaylibTileset();
  RaylibTileset(const RaylibTileset&) = delete;
  RaylibTileset& operator=(const RaylibTileset&) = delete;

  void update(const Camera3D& camera, int width, int height, float delta_seconds);
  void draw() const;
  /// True once the tiles this view wants have all loaded.
  bool idle() const noexcept;
  const std::string& status() const noexcept;
  const std::string& attribution() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};

} // namespace sim::cesium
