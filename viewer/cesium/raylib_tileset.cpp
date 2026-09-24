#include "raylib_tileset.hpp"

#include <raymath.h>

#include <Cesium3DTilesSelection/IPrepareRendererResources.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/ITileExcluder.h>
#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetExternals.h>
#include <Cesium3DTilesSelection/TilesetOptions.h>
#include <Cesium3DTilesSelection/ViewState.h>
#include <CesiumAsync/AsyncSystem.h>
#include <CesiumAsync/ITaskProcessor.h>
#include <CesiumCurl/CurlAssetAccessor.h>
#include <Cesium3DTilesContent/registerAllTileContentTypes.h>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGltf/AccessorUtility.h>
#include <CesiumGltf/Model.h>
#include <CesiumGltfContent/GltfUtilities.h>
#include <CesiumUtility/CreditSystem.h>

#include <glm/geometric.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <any>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

namespace sim::cesium {
namespace {

class WorkerPool final : public CesiumAsync::ITaskProcessor {
public:
  explicit WorkerPool(unsigned count) {
    _threads.reserve(count);
    for (unsigned i = 0; i < count; ++i) {
      _threads.emplace_back([this] {
        for (;;) {
          std::function<void()> task;
          {
            std::unique_lock lock(_mutex);
            _condition.wait(lock, [this] { return _stopping || !_tasks.empty(); });
            if (_stopping && _tasks.empty()) return;
            task = std::move(_tasks.front());
            _tasks.pop_front();
          }
          task();
        }
      });
    }
  }

  ~WorkerPool() override {
    {
      std::lock_guard lock(_mutex);
      _stopping = true;
    }
    _condition.notify_all();
    for (auto& thread : _threads) thread.join();
  }

  void startTask(std::function<void()> task) override {
    {
      std::lock_guard lock(_mutex);
      if (_stopping) return;
      _tasks.push_back(std::move(task));
    }
    _condition.notify_one();
  }

private:
  std::mutex _mutex;
  std::condition_variable _condition;
  std::deque<std::function<void()>> _tasks;
  std::vector<std::thread> _threads;
  bool _stopping = false;
};

struct Part {
  Mesh mesh{};
  Material material{};
};

struct RenderTile {
  std::vector<Part> parts;
  ~RenderTile() {
    for (Part& part : parts) {
      UnloadMesh(part.mesh);
      UnloadMaterial(part.material);
    }
  }
};

static Color factor_color(const CesiumGltf::Model& model,
                          const CesiumGltf::MeshPrimitive& primitive) {
  if (primitive.material < 0 ||
      static_cast<size_t>(primitive.material) >= model.materials.size()) return WHITE;
  const auto& pbr = model.materials[static_cast<size_t>(primitive.material)].pbrMetallicRoughness;
  if (!pbr || pbr->baseColorFactor.size() < 3) return WHITE;
  const auto channel = [&pbr](size_t i) {
    return static_cast<unsigned char>(std::clamp(pbr->baseColorFactor[i], 0.0, 1.0) * 255.0);
  };
  const unsigned char alpha = pbr->baseColorFactor.size() > 3 ? channel(3) : static_cast<unsigned char>(255);
  return {channel(0), channel(1), channel(2), alpha};
}

static Texture2D load_base_color_texture(const CesiumGltf::Model& model,
                                         const CesiumGltf::MeshPrimitive& primitive,
                                         int64_t& texcoord_set) {
  if (primitive.material < 0 ||
      static_cast<size_t>(primitive.material) >= model.materials.size()) return {};
  const auto& pbr = model.materials[static_cast<size_t>(primitive.material)].pbrMetallicRoughness;
  if (!pbr || !pbr->baseColorTexture) return {};
  const auto& info = *pbr->baseColorTexture;
  texcoord_set = info.texCoord;
  if (info.index < 0 || static_cast<size_t>(info.index) >= model.textures.size()) return {};
  const int image_index = model.textures[static_cast<size_t>(info.index)].source;
  if (image_index < 0 || static_cast<size_t>(image_index) >= model.images.size()) return {};
  const auto& asset = model.images[static_cast<size_t>(image_index)].pAsset;
  if (!asset || asset->width <= 0 || asset->height <= 0 ||
      asset->compressedPixelFormat != CesiumImage::GpuCompressedPixelFormat::NONE ||
      asset->bytesPerChannel != 1 || asset->channels < 1 || asset->channels > 4) return {};

  const size_t pixels = static_cast<size_t>(asset->width) * static_cast<size_t>(asset->height);
  if (pixels > asset->pixelData.size() || asset->pixelData.size() < pixels * asset->channels) return {};
  std::vector<unsigned char> rgba(pixels * 4);
  for (size_t i = 0; i < pixels; ++i) {
    const auto at = [&asset, i](size_t c) {
      return std::to_integer<unsigned char>(asset->pixelData[i * asset->channels + c]);
    };
    unsigned char r = 255, g = 255, b = 255, a = 255;
    if (asset->channels == 1) r = g = b = at(0);
    else if (asset->channels == 2) { r = g = b = at(0); a = at(1); }
    else { r = at(0); g = at(1); b = at(2); if (asset->channels == 4) a = at(3); }
    rgba[i * 4] = r;
    rgba[i * 4 + 1] = g;
    rgba[i * 4 + 2] = b;
    rgba[i * 4 + 3] = a;
  }
  Image image{rgba.data(), asset->width, asset->height, 1, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8};
  Texture2D texture = LoadTextureFromImage(image);
  if (texture.id != 0) SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
  return texture;
}

static std::unique_ptr<RenderTile> make_render_tile(
    const Cesium3DTilesSelection::Tile& tile,
    const glm::dmat4& ecef_to_local) {
  const auto* content = tile.getContent().getRenderContent();
  if (!content) return {};
  const auto& model = content->getModel();
  auto render = std::make_unique<RenderTile>();
  glm::dmat4 root = CesiumGltfContent::GltfUtilities::applyRtcCenter(model, tile.getTransform());
  root = CesiumGltfContent::GltfUtilities::applyGltfUpAxisTransform(model, root);

  model.forEachPrimitiveInScene(-1,
      [&render, &root, &ecef_to_local](
          const CesiumGltf::Model& gltf, const CesiumGltf::Node&, const CesiumGltf::Mesh&,
          const CesiumGltf::MeshPrimitive& primitive, const glm::dmat4& node_transform) {
        if (primitive.mode != CesiumGltf::MeshPrimitive::Mode::TRIANGLES) return;
        const auto positions = CesiumGltf::getPositionAccessorView(gltf, primitive);
        const int64_t vertex_count = std::visit(CesiumGltf::CountFromAccessor{}, positions);
        if (vertex_count > 65535) {
          // raylib indices are 16-bit. Skipping leaves a hole, so say so once.
          static bool warned = false;
          if (!warned) TraceLog(LOG_WARNING, "Cesium: skipping primitives over 65535 vertices");
          warned = true;
          return;
        }
        if (vertex_count < 3) return;

        const glm::dmat4 transform = ecef_to_local * root * node_transform;
        const glm::dmat3 normal_transform = glm::transpose(glm::inverse(glm::dmat3(transform)));
        Mesh mesh{};
        mesh.vertexCount = static_cast<int>(vertex_count);
        mesh.vertices = static_cast<float*>(MemAlloc(static_cast<unsigned int>(vertex_count * 3 * sizeof(float))));
        mesh.normals = static_cast<float*>(MemAlloc(static_cast<unsigned int>(vertex_count * 3 * sizeof(float))));
        int64_t texcoord_set = 0;
        const Texture2D texture = load_base_color_texture(gltf, primitive, texcoord_set);
        CesiumGltf::TexCoordAccessorType texcoords;
        const bool have_uvs = texture.id != 0 && primitive.material >= 0 &&
          static_cast<size_t>(primitive.material) < gltf.materials.size() &&
          gltf.materials[static_cast<size_t>(primitive.material)].pbrMetallicRoughness &&
          gltf.materials[static_cast<size_t>(primitive.material)].pbrMetallicRoughness->baseColorTexture &&
          primitive.attributes.find("TEXCOORD_" + std::to_string(texcoord_set)) != primitive.attributes.end();
        if (have_uvs) {
          texcoords = CesiumGltf::getTexCoordAccessorView(gltf, primitive, static_cast<int32_t>(texcoord_set));
          mesh.texcoords = static_cast<float*>(MemAlloc(static_cast<unsigned int>(vertex_count * 2 * sizeof(float))));
        }
        const auto normals_it = primitive.attributes.find("NORMAL");
        const bool have_normals = normals_it != primitive.attributes.end();
        CesiumGltf::NormalAccessorType normals;
        if (have_normals) normals = CesiumGltf::getNormalAccessorView(gltf, primitive);
        for (int64_t i = 0; i < vertex_count; ++i) {
          const auto position = std::visit(CesiumGltf::PositionFromAccessor{i}, positions);
          if (!position) { UnloadMesh(mesh); if (texture.id) UnloadTexture(texture); return; }
          const glm::dvec4 world = transform * glm::dvec4(*position, 1.0);
          const size_t offset = static_cast<size_t>(i) * 3;
          mesh.vertices[offset] = static_cast<float>(world.x);
          mesh.vertices[offset + 1] = static_cast<float>(world.y);
          mesh.vertices[offset + 2] = static_cast<float>(world.z);
          glm::dvec3 normal{0.0, 1.0, 0.0};
          if (have_normals) {
            const auto n = std::visit(CesiumGltf::NormalFromAccessor{i}, normals);
            if (n) normal = glm::normalize(normal_transform * *n);
          }
          mesh.normals[offset] = static_cast<float>(normal.x);
          mesh.normals[offset + 1] = static_cast<float>(normal.y);
          mesh.normals[offset + 2] = static_cast<float>(normal.z);
          if (have_uvs) {
            const auto uv = std::visit(CesiumGltf::TexCoordFromAccessor{i}, texcoords);
            const size_t uv_offset = static_cast<size_t>(i) * 2;
            if (uv) { mesh.texcoords[uv_offset] = static_cast<float>(uv->x); mesh.texcoords[uv_offset + 1] = static_cast<float>(uv->y); }
            else { mesh.texcoords[uv_offset] = 0.0F; mesh.texcoords[uv_offset + 1] = 0.0F; }
          }
        }
        const auto indices = CesiumGltf::getIndexAccessorView(gltf, primitive);
        const int64_t index_count = std::visit(CesiumGltf::NumIndicesFromAccessor{}, indices);
        if (index_count > 0) {
          if (index_count > INT32_MAX) { UnloadMesh(mesh); if (texture.id) UnloadTexture(texture); return; }
          mesh.indices = static_cast<unsigned short*>(MemAlloc(static_cast<unsigned int>(index_count * sizeof(unsigned short))));
          for (int64_t i = 0; i < index_count; ++i) {
            const int64_t index = std::visit(CesiumGltf::IndexFromAccessor{i}, indices);
            if (index < 0 || index >= vertex_count) { UnloadMesh(mesh); if (texture.id) UnloadTexture(texture); return; }
            mesh.indices[i] = static_cast<unsigned short>(index);
          }
          mesh.triangleCount = static_cast<int>(index_count / 3);
        } else {
          mesh.triangleCount = mesh.vertexCount / 3;
        }
        UploadMesh(&mesh, false);
        Material material = LoadMaterialDefault();
        material.maps[MATERIAL_MAP_DIFFUSE].color = factor_color(gltf, primitive);
        if (texture.id != 0 && have_uvs) material.maps[MATERIAL_MAP_DIFFUSE].texture = texture;
        else if (texture.id != 0) UnloadTexture(texture);
        render->parts.push_back({mesh, material});
      });
  if (render->parts.empty()) return {};
  return render;
}

static std::string visible_credit(std::string html) {
  std::string text;
  bool tag = false;
  for (char c : html) {
    if (c == '<') tag = true;
    else if (c == '>') tag = false;
    else if (!tag) text.push_back(c);
  }
  for (const auto& [entity, replacement] : std::vector<std::pair<std::string, std::string>>{
         {"&copy;", "©"}, {"&amp;", "&"}, {"&nbsp;", " "}}) {
    size_t at = 0;
    while ((at = text.find(entity, at)) != std::string::npos) {
      text.replace(at, entity.size(), replacement);
      at += replacement.size();
    }
  }
  return text;
}

class RadiusTileExcluder final : public Cesium3DTilesSelection::ITileExcluder {
public:
  RadiusTileExcluder(glm::dmat4 ecef_to_local, double radius_m)
      : _ecefToLocal(ecef_to_local), _radius(radius_m) {}

  bool shouldExclude(const Cesium3DTilesSelection::Tile& tile) const noexcept override {
    const auto box = Cesium3DTilesSelection::getOrientedBoundingBoxFromBoundingVolume(
        tile.getBoundingVolume());
    const glm::dvec3 center = glm::dvec3(_ecefToLocal * glm::dvec4(box.getCenter(), 1.0));
    double bound_radius = 0.0;
    for (int i = 0; i < 3; ++i) {
      const glm::dvec3 axis = glm::dvec3(_ecefToLocal * glm::dvec4(box.getHalfAxes()[i], 0.0));
      bound_radius += std::hypot(axis.x, axis.z);
    }
    return std::hypot(center.x, center.z) > _radius + bound_radius;
  }

private:
  glm::dmat4 _ecefToLocal;
  double _radius;
};

} // namespace

struct RaylibTileset::Impl {
  explicit Impl(double latitude, double longitude, double altitude, double radius,
      const std::string& token)
      : processor(std::make_shared<WorkerPool>(4)), async(processor),
        accessor(std::make_shared<CesiumCurl::CurlAssetAccessor>()),
        credits(std::make_shared<CesiumUtility::CreditSystem>()) {
    using CesiumGeospatial::Cartographic;
    const auto& ellipsoid = CesiumGeospatial::Ellipsoid::WGS84;
    const Cartographic home = Cartographic::fromDegrees(longitude, latitude, altitude);
    origin = ellipsoid.cartographicToCartesian(home);
    const double lon = home.longitude, lat = home.latitude;
    east = {-std::sin(lon), std::cos(lon), 0.0};
    const glm::dvec3 north{-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)};
    up = glm::normalize(glm::cross(east, north));
    // Viewer axes are (x, y, z) = (east, up, south): right-handed like raylib,
    // so glTF winding survives backface culling and the map is not mirrored.
    south = -north;
    ecefToLocal = glm::dmat4(
        glm::dvec4(east.x, up.x, south.x, 0.0),
        glm::dvec4(east.y, up.y, south.y, 0.0),
        glm::dvec4(east.z, up.z, south.z, 0.0),
        glm::dvec4(-glm::dot(origin, east), -glm::dot(origin, up), -glm::dot(origin, south), 1.0));

    Cesium3DTilesContent::registerAllTileContentTypes();
    renderer = std::make_shared<Renderer>(ecefToLocal);
    Cesium3DTilesSelection::TilesetExternals externals{accessor, renderer, async, credits};
    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = 16.0;
    options.maximumCachedBytes = 512LL * 1024 * 1024;
    options.preloadAncestors = false;
    options.preloadSiblings = false;
    options.forbidHoles = true;
    options.showCreditsOnScreen = true;
    options.credit = "Google Photorealistic 3D Tiles";
    options.excluders.push_back(std::make_shared<RadiusTileExcluder>(ecefToLocal, radius));
    tileset = std::make_unique<Cesium3DTilesSelection::Tileset>(externals, int64_t{2275207}, token, options);
    // Vehicles stand on y = 0, PX4's home. The configured altitude is AMSL
    // and the tiles are ellipsoidal, so sample the real ground under home and
    // draw the tiles shifted to put it there.
    tileset->sampleHeightMostDetailed({Cartographic::fromDegrees(longitude, latitude)})
        .thenInMainThread([this, altitude](Cesium3DTilesSelection::SampleHeightResult&& result) {
          if (!result.sampleSuccess.empty() && result.sampleSuccess[0])
            ground_offset = result.positions[0].height - altitude;
          else
            TraceLog(LOG_WARNING, "Cesium: no ground height under home; tiles stay at SIM_VIEWER_ORIGIN_ALT");
        });
  }

  struct Renderer final : Cesium3DTilesSelection::IPrepareRendererResources {
    explicit Renderer(glm::dmat4 transform) : ecef_to_local(transform) {}

    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
    prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
        Cesium3DTilesSelection::TileLoadResult&& result, const glm::dmat4&, const std::any&) override {
      return asyncSystem.createResolvedFuture(
          Cesium3DTilesSelection::TileLoadResultAndRenderResources{std::move(result), nullptr});
    }
    void* prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void*) override {
      auto render = make_render_tile(tile, ecef_to_local);
      if (render) prepared_tiles.fetch_add(1, std::memory_order_relaxed);
      return render.release();
    }
    void free(Cesium3DTilesSelection::Tile&, void*, void* main_result) noexcept override {
      delete static_cast<RenderTile*>(main_result);
    }
    void attachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
        const CesiumRasterOverlays::RasterOverlayTile&, void*, const glm::dvec2&, const glm::dvec2&) override {}
    void detachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
        const CesiumRasterOverlays::RasterOverlayTile&, void*) noexcept override {}
    void* prepareRasterInLoadThread(CesiumImage::ImageAsset&, const std::any&) override { return nullptr; }
    void* prepareRasterInMainThread(CesiumRasterOverlays::RasterOverlayTile&, void*) override { return nullptr; }
    void freeRaster(const CesiumRasterOverlays::RasterOverlayTile&, void*, void*) noexcept override {}

    glm::dmat4 ecef_to_local;
    std::atomic<int> prepared_tiles{0};
  };

  std::shared_ptr<WorkerPool> processor;
  CesiumAsync::AsyncSystem async;
  std::shared_ptr<CesiumCurl::CurlAssetAccessor> accessor;
  std::shared_ptr<CesiumUtility::CreditSystem> credits;
  std::shared_ptr<Renderer> renderer;
  double ground_offset = 0.0;  // before `tileset`, whose teardown may still resolve the sample
  std::unique_ptr<Cesium3DTilesSelection::Tileset> tileset;
  glm::dvec3 origin{}, east{}, south{}, up{};
  glm::dmat4 ecefToLocal{1.0};
  std::vector<Cesium3DTilesSelection::Tile::ConstPointer> visible;
  std::string load_status{"Loading Cesium tiles…"};
  float load_progress = 0.0F;
  std::string credit_text;

  void update(const Camera3D& camera, int width, int height, float delta) {
    const glm::dvec3 local_position(camera.position.x, camera.position.y + ground_offset, camera.position.z);
    const glm::dvec3 local_direction(camera.target.x - camera.position.x,
        camera.target.y - camera.position.y, camera.target.z - camera.position.z);
    const glm::dvec3 position = origin + east * local_position.x + up * local_position.y + south * local_position.z;
    const glm::dvec3 direction = glm::normalize(east * local_direction.x + up * local_direction.y + south * local_direction.z);
    const glm::dvec3 view_up = up;
    const double vfov = static_cast<double>(camera.fovy) * 3.14159265358979323846 / 180.0;
    const double hfov = 2.0 * std::atan(std::tan(vfov * 0.5) * static_cast<double>(width) / std::max(1, height));
    const Cesium3DTilesSelection::ViewState view(position, direction, view_up,
        glm::dvec2(width, height), hfov, vfov, CesiumGeospatial::Ellipsoid::WGS84);
    async.dispatchMainThreadTasks();
    const auto& result = tileset->updateViewGroup(tileset->getDefaultViewGroup(), {view}, delta);
    async.dispatchMainThreadTasks();
    tileset->loadTiles();
    visible = result.tilesToRenderThisFrame;
    const auto& snapshot = credits->getSnapshot();
    credit_text.clear();
    for (const auto& credit : snapshot.currentCredits) {
      if (!credits->shouldBeShownOnScreen(credit)) continue;
      const std::string text = visible_credit(credits->getHtml(credit));
      if (!text.empty() && credit_text.find(text) == std::string::npos) {
        if (!credit_text.empty()) credit_text += "  |  ";
        credit_text += text;
      }
    }
    load_progress = tileset->computeLoadProgress();
    std::ostringstream status;
    status << "Cesium 3D Tiles  " << static_cast<int>(load_progress) << "% loaded / "
           << renderer->prepared_tiles.load(std::memory_order_relaxed) << " mesh tiles";
    load_status = status.str();
  }

  void draw() const {
    const Matrix to_ground = MatrixTranslate(0.0F, static_cast<float>(-ground_offset), 0.0F);
    for (const auto& tile : visible) {
      const auto* content = tile->getContent().getRenderContent();
      if (!content) continue;
      const auto* render = static_cast<const RenderTile*>(content->getRenderResources());
      if (!render) continue;
      for (const Part& part : render->parts) DrawMesh(part.mesh, part.material, to_ground);
    }
  }
};

RaylibTileset::RaylibTileset(double latitude, double longitude, double altitude,
    double radius, std::string token)
    : _impl(std::make_unique<Impl>(latitude, longitude, altitude, radius, token)) {}
RaylibTileset::~RaylibTileset() = default;
void RaylibTileset::update(const Camera3D& camera, int width, int height, float delta) {
  _impl->update(camera, width, height, delta);
}
void RaylibTileset::draw() const { _impl->draw(); }
bool RaylibTileset::idle() const noexcept {
  return _impl->load_progress >= 100.0F && !_impl->visible.empty();
}
const std::string& RaylibTileset::status() const noexcept { return _impl->load_status; }
const std::string& RaylibTileset::attribution() const noexcept { return _impl->credit_text; }

} // namespace sim::cesium
