#include <algorithm>
#include <limits>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <deque>
#include <exception>
#include <fstream>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <utility>
#include <vector>

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>

#include <ditto/observer/client.hpp>
#include <ditto/edge/client.hpp>

#include <mavlink/common/mavlink.h>

#include <nlohmann/json.hpp>

#include "video.hpp"
#include "world.hpp"
#ifdef DITTO_CESIUM_VIEWER
#include "raylib_tileset.hpp"
#endif

extern char **environ;

namespace
{

using Clock = std::chrono::steady_clock;

volatile std::sig_atomic_t shutdown_requested = 0;

void request_shutdown(int)
{
  shutdown_requested = 1;
}

struct Vehicle {
  std::string id;
  float north_m{};
  float east_m{};
  float down_m{};
  std::array<float, 4> attitude_quaternion{1.0F, 0.0F, 0.0F, 0.0F};
  bool armed{};
  bool failsafe{};
  /// Drawn as a ground robot rather than a drone. Taken from the vehicle's own
  /// HEARTBEAT rather than a viewer-side setting: a fleet that is part rover and
  /// part multirotor is a thing the simulator can already run, and the vehicle
  /// is the only party that knows which it is.
  bool ground{};
  /// Horizontal ground speed, used to drive a walking gait. Taken from the
  /// velocity the telemetry already carries rather than differenced from
  /// position, which at a 10 Hz publish rate would be mostly quantisation.
  float speed_mps{};
  std::int64_t published_unix_ms{};
};

struct Snapshot {
  std::unordered_map<std::string, Vehicle> vehicles;
  std::string error;
  Clock::time_point received_at{};
};

struct Source {
  Source(std::string vehicle_id, const std::uint16_t port) : vehicle{std::move(vehicle_id)}, telemetry_port(port) {
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) throw std::runtime_error("could not create PX4 telemetry socket");
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
      close(fd);
      throw std::runtime_error("could not configure PX4 telemetry socket");
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
      close(fd);
      throw std::runtime_error("could not bind PX4 SIH telemetry socket");
    }
  }

  Source(const Source &) = delete;
  Source & operator=(const Source &) = delete;
  Source(Source && other) noexcept
  : vehicle(std::move(other.vehicle)), telemetry_port(other.telemetry_port), gcs_port(other.gcs_port),
    parser_buffer(other.parser_buffer), parser_status(other.parser_status),
    fd(std::exchange(other.fd, -1)) {}
  ~Source() { if (fd >= 0) close(fd); }

  Vehicle vehicle;
  std::uint16_t telemetry_port{};
  std::uint16_t gcs_port{};
  mavlink_message_t parser_buffer{};
  mavlink_status_t parser_status{};
  /// Set once this vehicle has sent LOCAL_POSITION_NED, after which its
  /// GLOBAL_POSITION_INT is ignored for position. See decode_mavlink.
  bool local_frame{};
  int fd{-1};
};

struct NetworkStatus {
  std::int64_t observed_unix_ms{};
  bool connected{};
  std::uint32_t links{};
  std::uint32_t connected_links{};
  double tx_bps{};
  double rx_bps{};
  double tx_utilization{};
  double rx_utilization{};
};

std::unordered_map<std::string, NetworkStatus> read_network_metrics(const std::string & path)
{
  std::unordered_map<std::string, NetworkStatus> result;
  if (path.empty()) return result;
  try {
    std::ifstream input(path);
    nlohmann::json document;
    input >> document;
    const auto observed = document.value("observed_unix_ms", 0LL);
    const auto add = [&](const std::string & id, const bool connected, const double tx_bps, const double rx_bps,
      const double tx_utilization, const double rx_utilization) {
      if (id.empty()) return;
      auto & status = result[id];
      status.observed_unix_ms = observed;
      ++status.links;
      status.connected_links += connected;
      status.connected = status.connected || connected;
      status.tx_bps += tx_bps;
      status.rx_bps += rx_bps;
      status.tx_utilization = std::max(status.tx_utilization, tx_utilization);
      status.rx_utilization = std::max(status.rx_utilization, rx_utilization);
    };
    for (const auto & link : document.at("links")) {
      const auto connected = link.value("connected", false);
      const auto tx_bps = link.value("tx_bps", 0.0);
      const auto rx_bps = link.value("rx_bps", 0.0);
      const auto tx_utilization = link.value("tx_utilization_percent", 0.0);
      const auto rx_utilization = link.value("rx_utilization_percent", 0.0);
      if (link.contains("source_node_id")) {
        add(link.value("source_node_id", ""), connected, tx_bps, rx_bps, tx_utilization, rx_utilization);
        add(link.value("destination_node_id", ""), connected, rx_bps, tx_bps, rx_utilization, tx_utilization);
      } else {
        add(link.value("node_id", ""), connected, tx_bps, rx_bps, tx_utilization, rx_utilization);
      }
    }
  } catch (const std::exception &) {}
  return result;
}

/// How stale the relay's link metrics may be before a node reads as NO LINK.
///
/// Sized from measurement, not intent. The relay means to publish every second,
/// but on a live twenty-node fleet the published interval ran from 631 ms to
/// 1588 ms (median 1068), and this viewer re-reads on its own 250 ms cycle, so
/// the worst age on screen reached ~1838 ms. Against the old 1500 ms threshold
/// that tripped on roughly one gap in seven -- and because every node inherits
/// the file's single `observed_unix_ms`, the whole fleet blinked to NO LINK at
/// once, which read as a fleet-wide network event rather than a stale file.
///
/// Four seconds is a missed publish absorbed with margin, while a relay that
/// has actually stopped still shows within a few seconds.
constexpr std::int64_t kMetricsStaleMs = 4000;

/// A publish that lands between this frame's clock read and its file read is
/// newer than `now`, not invalid, and rejecting it blanked the fleet for a
/// frame. Only a wildly future stamp means a clock worth distrusting.
constexpr std::int64_t kMetricsSkewMs = 1000;

bool fresh(const NetworkStatus & status, const std::int64_t now)
{
  if (status.observed_unix_ms <= 0) return false;
  const std::int64_t age = now - status.observed_unix_ms;
  return age <= kMetricsStaleMs && age >= -kMetricsSkewMs;
}

Color vehicle_color(const std::string & id)
{
  static constexpr Color palette[] = {
    {0, 130, 200, 255}, {245, 130, 48, 255}, {60, 180, 75, 255}, {145, 30, 180, 255},
  };
  std::size_t hash = 0;
  for (const auto character : id) hash = hash * 33U + static_cast<unsigned char>(character);
  return palette[hash % (sizeof(palette) / sizeof(*palette))];
}

bool vehicle_less(const std::string & left, const std::string & right)
{
  const auto left_separator = left.rfind('_');
  const auto right_separator = right.rfind('_');
  if (left_separator == std::string::npos || right_separator == std::string::npos ||
    left.substr(0, left_separator) != right.substr(0, right_separator)) return left < right;
  char * left_end = nullptr;
  char * right_end = nullptr;
  const auto left_number = std::strtoul(left.c_str() + left_separator + 1, &left_end, 10);
  const auto right_number = std::strtoul(right.c_str() + right_separator + 1, &right_end, 10);
  if (left_end == left.c_str() + left_separator + 1 || *left_end != '\0' ||
    right_end == right.c_str() + right_separator + 1 || *right_end != '\0') return left < right;
  return left_number == right_number ? left < right : left_number < right_number;
}

Vehicle blend(const Vehicle & from, const Vehicle & to, const float fraction)
{
  Vehicle result = to;
  result.north_m = from.north_m + (to.north_m - from.north_m) * fraction;
  result.east_m = from.east_m + (to.east_m - from.east_m) * fraction;
  result.down_m = from.down_m + (to.down_m - from.down_m) * fraction;
  const Quaternion a{from.attitude_quaternion[1], from.attitude_quaternion[2],
    from.attitude_quaternion[3], from.attitude_quaternion[0]};
  const Quaternion b{to.attitude_quaternion[1], to.attitude_quaternion[2],
    to.attitude_quaternion[3], to.attitude_quaternion[0]};
  const Quaternion q = QuaternionSlerp(a, b, fraction);
  result.attitude_quaternion = {q.w, q.x, q.y, q.z};
  return result;
}

struct Presentation {
  Vehicle from;
  Vehicle to;
  Clock::time_point started_at;
  Clock::time_point finishes_at;
};

Vehicle presented(const Presentation & presentation, const Clock::time_point now)
{
  const auto duration = presentation.finishes_at - presentation.started_at;
  const float fraction = duration.count() <= 0 ? 1.0F : std::clamp(
    std::chrono::duration<float>(now - presentation.started_at).count() /
    std::chrono::duration<float>(duration).count(), 0.0F, 1.0F);
  return blend(presentation.from, presentation.to, fraction);
}

Vector3 rotate_by_px4_quaternion(const Vehicle & vehicle, const Vector3 body_vector)
{
  const auto & q = vehicle.attitude_quaternion;
  const Quaternion attitude{q[1], q[2], q[3], q[0]};
  const Vector3 ned = Vector3RotateByQuaternion(body_vector, attitude);
  return sim::world::view_point(ned.y, -ned.z, ned.x);
}

/// The body centre's height above PX4's position, which is where the feet
/// touch down.
constexpr float kBodyHeight = 0.26F;
/// Each motor's offset from the body centre along both body axes: 0.72 m arms.
constexpr float kMotorReach = 0.72F * 0.70710678F;
/// Where the props turn, above the arms.
constexpr float kPropHeight = 0.07F;
/// The camera ball, under the nose on the centreline, about the body centre.
constexpr Vector3 kBall{0.21F, -0.135F, 0.0F};
constexpr float kBallRadius = 0.055F;

/// The camera gimbal under the nose: where its ball hangs, and which way the
/// ball looks -- level, turning with the heading, 30 degrees down, as a real
/// drone's camera does, so the horizon holds still while the airframe tilts.
struct Gimbal {
  Vector3 ball;
  Vector3 look;
  Vector3 up;
};

Gimbal gimbal(const Vehicle & vehicle)
{
  const Vector3 forward = rotate_by_px4_quaternion(vehicle, {1.0F, 0.0F, 0.0F});
  const Vector3 up = Vector3Negate(rotate_by_px4_quaternion(vehicle, {0.0F, 0.0F, 1.0F}));
  const float heading = std::atan2(forward.x, forward.z);
  constexpr float tilt = 30.0F * DEG2RAD;
  const Vector3 level{std::sin(heading), 0.0F, std::cos(heading)};
  return {
    Vector3Add(sim::world::view_point(vehicle.east_m, -vehicle.down_m, vehicle.north_m),
      Vector3Add(Vector3Scale(forward, kBall.x), Vector3Scale(up, kBodyHeight + kBall.y))),
    Vector3Add(Vector3Scale(level, std::cos(tilt)), {0.0F, -std::sin(tilt), 0.0F}),
    Vector3Add(Vector3Scale(level, std::sin(tilt)), {0.0F, std::cos(tilt), 0.0F}),
  };
}

/// What a vehicle's video feed sees: out through the window on its gimbal ball.
Camera3D nose_camera(const Vehicle & vehicle)
{
  const Gimbal mount = gimbal(vehicle);
  Camera3D camera{};
  camera.position = Vector3Add(mount.ball, Vector3Scale(mount.look, kBallRadius + 0.005F));
  camera.target = Vector3Add(camera.position, mount.look);
  camera.up = {0.0F, 1.0F, 0.0F};
  camera.fovy = 48.0F;  // vertical; 77 degrees across at 16:9, a typical drone camera
  camera.projection = CAMERA_PERSPECTIVE;
  return camera;
}

struct IsrTarget {
  std::string id;
  std::string site;
  float north_m{}, east_m{}, surface_m{};
  bool found{};
  bool surface_ready{true};
  Vector3 center() const { return sim::world::view_point(east_m, surface_m + 1.1F, north_m); }
};

std::vector<IsrTarget> read_isr_targets(const std::string & path)
{
  if (path.empty()) return {};
  std::ifstream input(path);
  if (!input) throw std::runtime_error("ISR targets file not readable: " + path);
  const auto document = nlohmann::json::parse(input);
  std::vector<IsrTarget> targets;
  for (const auto & entry : document.at("targets")) {
    IsrTarget target;
    target.id = entry.at("id").get<std::string>();
    target.site = entry.at("site").get<std::string>();
    target.north_m = entry.at("north_m").get<float>();
    target.east_m = entry.at("east_m").get<float>();
    target.surface_m = entry.at("surface_hint_m").get<float>();
    if (target.id.empty() || !std::isfinite(target.north_m) || !std::isfinite(target.east_m) ||
      !std::isfinite(target.surface_m)) throw std::runtime_error("invalid ISR target in " + path);
    targets.push_back(std::move(target));
  }
  return targets;
}

struct TargetBox { Rectangle pixels; float distance_m; };
struct OcclusionSample { Clock::time_point checked{}; bool clear{}; };

std::optional<TargetBox> projected_target(const IsrTarget & target, const Camera3D & camera,
  const int width, const int height)
{
  const Vector3 center = target.center();
  const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
  const Vector3 to_target = Vector3Subtract(center, camera.position);
  const float distance = Vector3Length(to_target);
  if (distance > 100.0F || distance < 2.0F || Vector3DotProduct(to_target, forward) <= 0.0F)
    return std::nullopt;
  Vector2 low{static_cast<float>(width), static_cast<float>(height)};
  Vector2 high{0.0F, 0.0F};
  for (const float x : {-1.1F, 1.1F}) for (const float y : {-1.1F, 1.1F})
    for (const float z : {-1.1F, 1.1F}) {
      const Vector3 corner = Vector3Add(center, {x, y, z});
      if (Vector3DotProduct(Vector3Subtract(corner, camera.position), forward) <= 0.0F)
        return std::nullopt;
      const Vector2 pixel = GetWorldToScreenEx(corner, camera, width, height);
      low.x = std::min(low.x, pixel.x); low.y = std::min(low.y, pixel.y);
      high.x = std::max(high.x, pixel.x); high.y = std::max(high.y, pixel.y);
    }
  if (low.x < 0 || low.y < 0 || high.x >= width || high.y >= height ||
    high.x - low.x < 18.0F || high.y - low.y < 18.0F) return std::nullopt;
  return TargetBox{{low.x, low.y, high.x - low.x, high.y - low.y}, distance};
}

bool target_unoccluded(const IsrTarget & target, const Camera3D & camera, const int width, const int height)
{
  const Vector2 pixel = GetWorldToScreenEx(target.center(), camera, width, height);
  const int x = static_cast<int>(pixel.x), y = height - 1 - static_cast<int>(pixel.y);
  if (x < 0 || x >= width || y < 0 || y >= height) return false;
  rlDrawRenderBatchActive();
  float depth = 1.0F;
  glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
  if (depth >= 1.0F) return false;
  const double near = rlGetCullDistanceNear(), far = rlGetCullDistanceFar();
  const double scene_m = near * far / (far - depth * (far - near));
  const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
  const float target_m = Vector3DotProduct(Vector3Subtract(target.center(), camera.position), forward);
  return scene_m + 0.75 >= target_m;
}

class IsrPublisher {
public:
  IsrPublisher(std::string socket, std::string scenario, std::string run_id, const double home_latitude,
    const double home_longitude, const std::vector<IsrTarget> & targets)
    : socket_(std::move(socket)), scenario_(std::move(scenario)), run_id_(std::move(run_id)),
      home_latitude_(home_latitude),
      home_longitude_(home_longitude), worker_([this] { run(); })
  {
    for (const auto & target : targets) publish(target, false, "");
  }

  ~IsrPublisher()
  {
    { std::lock_guard<std::mutex> lock(mutex_); stopping_ = true; }
    ready_.notify_one();
    worker_.join();
  }

  void publish(const IsrTarget & target, const bool found, std::string observer)
  {
    constexpr double metres_per_degree = 111319.49079327357;
    nlohmann::json position = nullptr;
    if (found) position = {
      {"north_m", target.north_m}, {"east_m", target.east_m}, {"up_m", target.surface_m},
      {"latitude_deg", home_latitude_ + target.north_m / metres_per_degree},
      {"longitude_deg", home_longitude_ + target.east_m /
        (metres_per_degree * std::cos(home_latitude_ * DEG2RAD))}};
    const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    nlohmann::json document{
      {"_id", scenario_ + "/" + target.id}, {"schema", "ditto.isr_target.v1"},
      {"run_id", run_id_}, {"target_id", target.id}, {"site", target.site},
      {"state", found ? "found" : "unknown"}, {"position", position},
      {"found_by", found ? observer : ""}, {"updated_unix_ms", now}};
    { std::lock_guard<std::mutex> lock(mutex_); pending_.push_back(std::move(document)); }
    ready_.notify_one();
  }

  bool idle()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.empty();
  }

private:
  void run()
  {
    constexpr char query[] = "INSERT INTO isr_targets DOCUMENTS (:doc) ON ID CONFLICT DO UPDATE_LOCAL_DIFF";
    for (;;) {
      nlohmann::json document;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
        if (stopping_) return;
        document = pending_.front();
      }
      bool success = false;
      try {
        ditto::edge::Client client(socket_, std::string{}, std::chrono::seconds(5));
        ditto::edge::StoreResult result;
        const auto status = client.execute(query, nlohmann::json{{"doc", document}}.dump(), result);
        success = status.ok();
        if (!success) TraceLog(LOG_WARNING, "ISR target publish failed: %s", status.error_message().c_str());
        if (success) {
          const auto positioned = client.execute(
            "UPDATE isr_targets SET position = :position WHERE _id = :id",
            nlohmann::json{{"id", document["_id"]}, {"position", document["position"]}}.dump(), result);
          success = positioned.ok();
          if (!success) TraceLog(LOG_WARNING, "ISR target position failed: %s", positioned.error_message().c_str());
        }
      } catch (const std::exception & error) {
        TraceLog(LOG_WARNING, "ISR target publish failed: %s", error.what());
      }
      std::unique_lock<std::mutex> lock(mutex_);
      if (success) pending_.pop_front();
      else ready_.wait_for(lock, std::chrono::seconds(1), [this] { return stopping_; });
    }
  }

  std::string socket_, scenario_, run_id_;
  double home_latitude_{}, home_longitude_{};
  std::mutex mutex_;
  std::condition_variable ready_;
  std::deque<nlohmann::json> pending_;
  bool stopping_{};
  std::thread worker_;
};

/// One vertex of a drone part, built once in the part's own frame.
struct LitVertex {
  Vector3 position;
  Vector3 normal;
  Color color;
};

using Surface = std::vector<LitVertex>;

/// A slice through a loft: an ellipse `width` by `height` (half-extents) at
/// `at` along the axis, raised by `lift` and turned about the axis by `twist`.
struct Section {
  float at;
  float width;
  float height;
  float lift = 0.0F;
  float twist = 0.0F;
};

/// Appends a loft through `sections` along `axis`, each ellipse drawn as a
/// `sides`-gon whose flats, not its corners, sit at the stated size, with both
/// ends capped. Flat normals give a hull its facets; smooth ones suit turned
/// parts.
void loft(Surface & surface, const Vector3 origin, const Vector3 axis, const std::vector<Section> & sections,
  const int sides, const bool smooth, const Color color)
{
  const Vector3 u = Vector3Normalize(Vector3CrossProduct(
    axis, std::fabs(axis.y) > 0.9F ? Vector3{1.0F, 0.0F, 0.0F} : Vector3{0.0F, 1.0F, 0.0F}));
  const Vector3 v = Vector3CrossProduct(u, axis);
  const float widen = 1.0F / std::cos(PI / static_cast<float>(sides));
  const auto middle = [&](const Section & section) {
    return Vector3Add(origin, Vector3Add(Vector3Scale(axis, section.at), Vector3Scale(v, section.lift)));
  };
  std::vector<Vector3> points;
  points.reserve(sections.size() * static_cast<std::size_t>(sides));
  for (const auto & section : sections) {
    const float c = std::cos(section.twist), s = std::sin(section.twist);
    for (int k = 0; k < sides; ++k) {
      const float angle = PI * static_cast<float>(2 * k + 1) / static_cast<float>(sides);
      const float a = section.width * widen * std::cos(angle), b = section.height * widen * std::sin(angle);
      points.push_back(Vector3Add(middle(section),
        Vector3Add(Vector3Scale(u, a * c - b * s), Vector3Scale(v, a * s + b * c))));
    }
  }
  const int rings = static_cast<int>(sections.size());
  const auto at = [&](const int ring, const int k) {
    return points[static_cast<std::size_t>(ring * sides + (k + sides) % sides)];
  };
  // Along the axis crossed with around it points outward.
  const auto normal = [&](const int ring, const int k) {
    return Vector3Normalize(Vector3CrossProduct(
      Vector3Subtract(at(std::min(ring + 1, rings - 1), k), at(std::max(ring - 1, 0), k)),
      Vector3Subtract(at(ring, k + 1), at(ring, k - 1))));
  };
  for (int ring = 0; ring + 1 < rings; ++ring) {
    for (int k = 0; k < sides; ++k) {
      const std::array<Vector3, 4> p{at(ring, k), at(ring + 1, k), at(ring + 1, k + 1), at(ring, k + 1)};
      const Vector3 flat = Vector3Normalize(
        Vector3CrossProduct(Vector3Subtract(p[2], p[0]), Vector3Subtract(p[3], p[1])));
      const std::array<Vector3, 4> n = smooth ?
        std::array<Vector3, 4>{normal(ring, k), normal(ring + 1, k), normal(ring + 1, k + 1), normal(ring, k + 1)} :
        std::array<Vector3, 4>{flat, flat, flat, flat};
      for (const int corner : {0, 1, 2, 0, 2, 3}) surface.push_back({p[corner], n[corner], color});
    }
  }
  for (const int ring : {0, rings - 1}) {
    // Wound to face back along the axis at the first ring, forward at the last.
    const Vector3 n = Vector3Scale(axis, ring == 0 ? -1.0F : 1.0F);
    const Vector3 centre = middle(sections[static_cast<std::size_t>(ring)]);
    for (int k = 0; k < sides; ++k) {
      surface.push_back({centre, n, color});
      surface.push_back({at(ring, ring == 0 ? k : k + 1), n, color});
      surface.push_back({at(ring, ring == 0 ? k + 1 : k), n, color});
    }
  }
}

/// Draws `surface` under the current transform, lit for the way the part
/// faces: `frame`'s rotation turns its normals into view space. An opaque
/// `paint` replaces the colours it was built with.
void draw_lit(const Surface & surface, const Matrix & frame, const Color paint = {}, const unsigned char alpha = 255)
{
  // An afternoon sun in the south-west, 55 degrees up, in view space: x east,
  // y up, z south.
  constexpr Vector3 sun{-0.406F, 0.819F, 0.406F};
  rlBegin(RL_TRIANGLES);
  for (const auto & [position, normal, color] : surface) {
    const Vector3 n{frame.m0 * normal.x + frame.m4 * normal.y + frame.m8 * normal.z,
      frame.m1 * normal.x + frame.m5 * normal.y + frame.m9 * normal.z,
      frame.m2 * normal.x + frame.m6 * normal.y + frame.m10 * normal.z};
    // Sky from above, bounce from below, and the sun; matte paint has no highlight.
    const float light = 0.34F + 0.12F * n.y + 0.72F * std::max(0.0F, Vector3DotProduct(n, sun));
    const Color base = paint.a != 0 ? paint : color;
    const auto lit = [light](const unsigned char channel) {
      return static_cast<unsigned char>(std::min(255.0F, static_cast<float>(channel) * light));
    };
    rlColor4ub(lit(base.r), lit(base.g), lit(base.b), alpha);
    rlVertex3f(position.x, position.y, position.z);
  }
  rlEnd();
}

/// The parts of a drone, each drawn whole.
struct DroneSurfaces {
  Surface hull;   // hull and battery: red in failsafe, and all a distant drone is
  Surface frame;  // arms, motors, legs, the GNSS mast and the gimbal's mount
  Surface marks;  // ID bands on the front arms, painted in the vehicle colour
  Surface ball;   // the gimbal's camera ball, in its own level frame
  Surface prop;   // one two-blade prop about its hub
};

/// A military quadrotor in the X configuration, the size of a Group 2 tactical
/// airframe: a faceted hull with the battery on its spine, carbon arms on
/// folding hinges, motor pods standing on their own legs, a GNSS mast, and an
/// EO/IR ball under the nose. Built once, in the body frame -- x forward, y up,
/// z right -- about the body centre.
const DroneSurfaces & drone_surfaces()
{
  static const DroneSurfaces surfaces = [] {
    // Matte olive-grey paint over carbon and dark metal.
    constexpr Color paint{94, 100, 86, 255}, panel{66, 71, 64, 255}, carbon{36, 38, 40, 255};
    constexpr Color metal{62, 65, 70, 255}, black{26, 27, 29, 255}, glass{14, 20, 30, 255};
    constexpr Vector3 x{1.0F, 0.0F, 0.0F}, y{0.0F, 1.0F, 0.0F}, z{0.0F, 0.0F, 1.0F}, down{0.0F, -1.0F, 0.0F};
    const auto tube = [](Surface & surface, const Vector3 from, const Vector3 to, const float radius,
      const Color color, const int sides = 6, const bool smooth = true) {
      loft(surface, from, Vector3Normalize(Vector3Subtract(to, from)),
        {{0.0F, radius, radius}, {Vector3Distance(from, to), radius, radius}}, sides, smooth, color);
    };
    DroneSurfaces s;
    // Widest behind the middle, sloping down to the nose.
    loft(s.hull, {}, x, {{-0.29F, 0.055F, 0.034F, 0.004F}, {-0.23F, 0.1F, 0.058F}, {0.08F, 0.125F, 0.068F},
      {0.22F, 0.1F, 0.056F, -0.008F}, {0.3F, 0.045F, 0.03F, -0.022F}}, 8, false, paint);
    loft(s.hull, {}, x, {{-0.21F, 0.07F, 0.018F, 0.078F}, {-0.02F, 0.07F, 0.018F, 0.078F}}, 4, false, panel);
    for (const float along : {1.0F, -1.0F}) {
      for (const float side : {1.0F, -1.0F}) {
        const Vector3 root{along * 0.12F, 0.0F, side * 0.1F};
        const Vector3 motor{along * kMotorReach, 0.0F, side * kMotorReach};
        tube(s.frame, root, motor, 0.018F, carbon, 8);
        tube(s.frame, root, Vector3Lerp(root, motor, 0.16F), 0.03F, paint, 6, false);  // folding hinge
        if (along > 0.0F) {
          tube(s.marks, Vector3Lerp(root, motor, 0.7F), Vector3Lerp(root, motor, 0.8F), 0.021F, WHITE, 8);
        }
        loft(s.frame, motor, y, {{-0.03F, 0.03F, 0.03F}, {0.012F, 0.03F, 0.03F}}, 10, true, panel);
        loft(s.frame, motor, y, {{0.012F, 0.044F, 0.044F}, {0.062F, 0.044F, 0.044F}, {kPropHeight, 0.036F, 0.036F}},
          12, true, metal);
        // A faired leg under each motor, splayed out onto a rubber foot. The
        // rear pair carry the radio antennas in their lower half.
        const Vector3 hip{motor.x, -0.02F, motor.z};
        const Vector3 foot{motor.x * 1.08F, -kBodyHeight, motor.z * 1.08F};
        const Vector3 leg = Vector3Subtract(Vector3Add(foot, {0.0F, 0.012F, 0.0F}), hip);
        const float length = Vector3Length(leg);
        loft(s.frame, hip, Vector3Normalize(leg), {{0.0F, 0.012F, 0.02F}, {length, 0.008F, 0.012F}}, 8, true, carbon);
        if (along < 0.0F) {
          loft(s.frame, hip, Vector3Normalize(leg), {{length * 0.45F, 0.0135F, 0.02F}, {length * 0.9F, 0.011F, 0.016F}},
            8, true, panel);
        }
        loft(s.frame, foot, y, {{0.0F, 0.014F, 0.014F}, {0.007F, 0.02F, 0.02F}, {0.02F, 0.011F, 0.011F}}, 8, true,
          black);
      }
    }
    // A GNSS puck on its mast.
    tube(s.frame, {0.03F, 0.05F, 0.0F}, {0.03F, 0.14F, 0.0F}, 0.006F, carbon);
    loft(s.frame, {0.03F, 0.0F, 0.0F}, y, {{0.14F, 0.03F, 0.03F}, {0.15F, 0.03F, 0.03F}, {0.157F, 0.022F, 0.022F}},
      12, true, paint);
    // The gimbal's yaw housing and fork. The ball between the tines holds
    // level, so it is its own surface.
    loft(s.frame, {kBall.x, 0.0F, 0.0F}, y, {{-0.072F, 0.028F, 0.028F}, {-0.05F, 0.028F, 0.028F}}, 10, true, panel);
    loft(s.frame, {kBall.x, -0.072F, 0.0F}, z, {{-0.068F, 0.012F, 0.005F}, {0.068F, 0.012F, 0.005F}}, 4, false,
      panel);
    for (const float side : {1.0F, -1.0F}) {
      loft(s.frame, {kBall.x, -0.072F, side * 0.064F}, down,
        {{0.0F, 0.004F, 0.014F}, {-0.072F - kBall.y, 0.004F, 0.012F}}, 4, false, panel);
    }
    std::vector<Section> sphere;
    for (int ring = 0; ring <= 7; ++ring) {
      const float angle = PI * (0.04F + 0.92F * static_cast<float>(ring) / 7.0F);
      const float radius = kBallRadius * std::sin(angle);
      sphere.push_back({-kBallRadius * std::cos(angle), radius, radius});
    }
    loft(s.ball, {}, x, sphere, 12, true, paint);
    // EO and IR windows, looking along x.
    loft(s.ball, {0.0F, 0.0F, -0.013F}, x, {{0.038F, 0.02F, 0.02F}, {0.058F, 0.02F, 0.02F}}, 10, true, glass);
    loft(s.ball, {0.0F, 0.0F, 0.023F}, x, {{0.038F, 0.012F, 0.012F}, {0.057F, 0.012F, 0.012F}}, 10, true, glass);
    // A spinner, and two blades that taper and flatten toward the tips.
    loft(s.prop, {}, y, {{0.0F, 0.018F, 0.018F}, {0.01F, 0.018F, 0.018F}, {0.022F, 0.006F, 0.006F}}, 8, true, black);
    for (const Vector3 blade : {x, Vector3Negate(x)}) {
      loft(s.prop, {0.0F, 0.005F, 0.0F}, blade, {{0.012F, 0.011F, 0.005F}, {0.045F, 0.021F, 0.0035F, 0.0F, 0.42F},
        {0.09F, 0.023F, 0.003F, 0.0F, 0.34F}, {0.16F, 0.019F, 0.0025F, 0.0F, 0.25F},
        {0.23F, 0.013F, 0.002F, 0.0F, 0.18F}, {0.28F, 0.005F, 0.0015F, 0.0F, 0.14F}}, 4, false, black);
    }
    return s;
  }();
  return surfaces;
}

/// A drone where PX4 puts it, at the attitude PX4 reports. Close up, the
/// vehicle colour is only the ID bands on the front arms, which also mark the
/// nose. Past 120 m the detail is a few pixels, so a distant drone is its hull
/// under discs in the vehicle colour, which is what tells a fleet apart.
/// Failsafe turns the hull red at any distance. Props turn while armed, faint
/// under a dark blur disc; `prop_degrees` comes from wall time so they turn at
/// the same rate at any frame rate.
void draw_drone(
  const Vehicle & vehicle, const Vector3 center, const float prop_degrees, const Vector3 camera_position)
{
  const DroneSurfaces & surfaces = drone_surfaces();
  const Color color = vehicle_color(vehicle.id);
  const Vector3 forward = Vector3Normalize(rotate_by_px4_quaternion(vehicle, {1.0F, 0.0F, 0.0F}));
  const Vector3 right = Vector3Normalize(rotate_by_px4_quaternion(vehicle, {0.0F, 1.0F, 0.0F}));
  const Vector3 up = Vector3Negate(
    Vector3Normalize(rotate_by_px4_quaternion(vehicle, {0.0F, 0.0F, 1.0F})));
  // Columns are the body axes; forward x up = right, so no mirror.
  const Matrix body{forward.x, up.x, right.x, center.x, forward.y, up.y, right.y, center.y,
    forward.z, up.z, right.z, center.z, 0.0F, 0.0F, 0.0F, 1.0F};
  const bool detailed = Vector3Distance(center, camera_position) < 120.0F;
  if (detailed) {
    const Gimbal mount = gimbal(vehicle);
    const Vector3 side = Vector3CrossProduct(mount.look, mount.up);
    const Matrix frame{mount.look.x, mount.up.x, side.x, mount.ball.x, mount.look.y, mount.up.y, side.y,
      mount.ball.y, mount.look.z, mount.up.z, side.z, mount.ball.z, 0.0F, 0.0F, 0.0F, 1.0F};
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloat(frame));
    draw_lit(surfaces.ball, frame);
    rlPopMatrix();
  }
  rlPushMatrix();
  rlMultMatrixf(MatrixToFloat(body));
  rlTranslatef(0.0F, kBodyHeight, 0.0F);
  draw_lit(surfaces.hull, body, vehicle.failsafe ? RED : Color{});
  if (detailed) {
    draw_lit(surfaces.frame, body);
    draw_lit(surfaces.marks, body, color);
    // Navigation lights, red to port and green to starboard, and a
    // double-flash anti-collision strobe while armed.
    constexpr float outboard = kMotorReach + 0.022F;  // on the front motor pods' outer faces
    DrawSphereEx({outboard, -0.01F, -outboard}, 0.011F, 2, 5, {255, 48, 40, 255});
    DrawSphereEx({outboard, -0.01F, outboard}, 0.011F, 2, 5, {40, 255, 110, 255});
    const double beat = std::fmod(GetTime(), 1.2);
    DrawSphereEx({-0.19F, 0.098F, 0.0F}, 0.012F, 2, 5,
      vehicle.armed && (beat < 0.05 || (beat > 0.15 && beat < 0.2)) ? WHITE : Color{92, 94, 98, 255});
  }
  int motor = 0;
  for (const float along : {kMotorReach, -kMotorReach}) {
    for (const float side : {kMotorReach, -kMotorReach}) {
      // Diagonal pairs turn the same way, as on a real X frame.
      const float turn = (along > 0.0F) == (side > 0.0F) ? 1.0F : -1.0F;
      if (detailed) {
        rlPushMatrix();
        rlTranslatef(along, kPropHeight, side);
        rlRotatef(turn * prop_degrees + 47.0F * static_cast<float>(motor), 0.0F, 1.0F, 0.0F);
        draw_lit(surfaces.prop, body, {}, vehicle.armed ? 110 : 255);
        rlPopMatrix();
      }
      if (vehicle.armed || !detailed) {
        DrawCylinder({along, kPropHeight + 0.02F, side}, 0.28F, 0.28F, 0.003F, 16,
          detailed ? Color{26, 27, 29, 56} : Fade(color, 0.6F));
      }
      ++motor;
    }
  }
  rlPopMatrix();
}

/// A bipedal factory robot, in the manner of an Agility Digit: digitigrade legs
/// that bend backwards at the middle joint, a boxy torso, slim arms and a
/// sensor head. `center` sits on the floor the robot is standing on, so
/// everything here is built upward from it.
///
/// `gait` is a walk phase in radians, advanced by the caller from the robot's
/// own ground speed. A biped whose legs hold still while its body slides across
/// the floor looks far worse than a wheeled puck ever did -- the legs are the
/// reason to accept the extra geometry, so they have to move.
void draw_robot(const Vehicle & vehicle, const Vector3 center, const float gait)
{
  const Color color = vehicle_color(vehicle.id);
  const Vector3 forward = Vector3Normalize(rotate_by_px4_quaternion(vehicle, {1.0F, 0.0F, 0.0F}));
  const Vector3 right = Vector3Normalize(rotate_by_px4_quaternion(vehicle, {0.0F, 1.0F, 0.0F}));
  const Vector3 up = Vector3Negate(
    Vector3Normalize(rotate_by_px4_quaternion(vehicle, {0.0F, 0.0F, 1.0F})));

  // Stride scales with speed so a standing robot stands still rather than
  // marching on the spot.
  const float stride = std::clamp(vehicle.speed_mps / 1.4F, 0.0F, 1.0F);

  /// A point in the robot's own frame: height, forward offset, lateral offset.
  const auto at = [&](const float u, const float f, const float r) {
      return Vector3Add(center, Vector3Add(Vector3Scale(up, u),
        Vector3Add(Vector3Scale(forward, f), Vector3Scale(right, r))));
    };
  const auto limb = [](const Vector3 from, const Vector3 to, const float radius,
      const Color limb_color) {
      DrawCylinderEx(from, to, radius, radius, 8, limb_color);
    };

  // A slight vertical bob at twice the step rate, as weight transfers.
  const float bob = 0.012F * std::cos(gait * 2.0F) * stride;
  const float hip_height = 0.86F + bob;

  for (const float side : {-1.0F, 1.0F}) {
    const float swing = std::sin(gait + (side > 0.0F ? 0.0F : PI)) * stride;
    // Lift the foot only on the half of the cycle that carries it forward.
    const float lift = std::max(0.0F, std::sin(gait + (side > 0.0F ? 0.0F : PI))) * 0.06F * stride;
    const float lateral = side * 0.12F;

    const Vector3 hip = at(hip_height, 0.0F, lateral);
    // The middle joint sits BEHIND the line from hip to ankle. That backward
    // bend is what makes the leg read as a bird's rather than a person's, and
    // it is the single most recognisable thing about this class of robot.
    const Vector3 knee = at(0.44F + bob, -0.15F + swing * 0.09F, lateral);
    const Vector3 ankle = at(0.10F + lift, swing * 0.19F, lateral);
    const Vector3 toe = Vector3Add(ankle, Vector3Scale(forward, 0.11F));

    limb(hip, knee, 0.048F, DARKGRAY);
    limb(knee, ankle, 0.038F, GRAY);
    limb(Vector3Add(ankle, Vector3Scale(forward, -0.05F)), toe, 0.032F, DARKGRAY);
    DrawSphere(knee, 0.052F, DARKGRAY);
  }

  // Pelvis, then the torso above it.
  limb(at(hip_height, 0.0F, -0.12F), at(hip_height, 0.0F, 0.12F), 0.10F, DARKGRAY);
  DrawCylinderEx(at(hip_height + 0.04F, 0.0F, 0.0F), at(1.34F + bob, 0.0F, 0.0F), 0.15F, 0.17F,
    12, color);

  // Arms swing opposite the leg on the same side while walking, and work at
  // something once the robot stops. A robot standing perfectly rigid at a
  // destination reads as frozen -- as a stalled simulation rather than as a
  // machine doing its job -- and standing still is most of what these robots do
  // once they have been commanded somewhere.
  const float working = 1.0F - std::min(1.0F, stride * 4.0F);
  // Offset the work cycle per robot, or a room full of them moves in lockstep.
  std::size_t hash = 0;
  for (const auto character : vehicle.id) hash = hash * 31U + static_cast<unsigned char>(character);
  const float cycle = static_cast<float>(GetTime()) * 2.6F +
    static_cast<float>(hash % 100U) * 0.063F;
  for (const float side : {-1.0F, 1.0F}) {
    const float swing = -std::sin(gait + (side > 0.0F ? 0.0F : PI)) * stride;
    const float lateral = side * 0.19F;
    const Vector3 shoulder = at(1.30F + bob, 0.0F, lateral);
    Vector3 elbow = at(1.05F + bob, swing * 0.10F, lateral * 1.05F);
    Vector3 hand = at(0.83F + bob, swing * 0.17F, lateral * 1.02F);
    if (working > 0.0F) {
      // Elbows in, hands out in front, rising and falling out of phase with
      // each other: working at something waist-high.
      const float lift = std::sin(cycle + (side > 0.0F ? 0.0F : 1.7F)) * 0.5F + 0.5F;
      const Vector3 busy_elbow = at(1.06F + bob, 0.14F, lateral * 0.92F);
      const Vector3 busy_hand = at(0.94F + lift * 0.11F + bob, 0.34F, lateral * 0.52F);
      elbow = Vector3Lerp(elbow, busy_elbow, working);
      hand = Vector3Lerp(hand, busy_hand, working);
    }
    limb(shoulder, elbow, 0.035F, GRAY);
    limb(elbow, hand, 0.030F, DARKGRAY);
    DrawSphere(shoulder, 0.048F, DARKGRAY);
  }

  // Head, with a visor on the front so the facing is readable at a distance.
  DrawCylinderEx(at(1.36F + bob, 0.0F, 0.0F), at(1.53F + bob, 0.0F, 0.0F), 0.08F, 0.08F, 10,
    DARKGRAY);
  const Vector3 visor = at(1.47F + bob, 0.06F, 0.0F);
  DrawCylinderEx(visor, Vector3Add(visor, Vector3Scale(forward, 0.04F)), 0.06F, 0.05F, 10,
    Fade(YELLOW, 0.9F));

  // Status, on the chest where it stays visible from every angle.
  DrawSphere(at(1.22F + bob, 0.15F, 0.0F), 0.055F,
    vehicle.failsafe ? RED : (vehicle.armed ? LIME : LIGHTGRAY));
}

std::int64_t unix_time_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

struct WorldOrigin {
  double latitude{};
  double longitude{};
  double altitude{};
  bool horizontal_set{};
  bool altitude_set{};
};

std::optional<double> environment_number(const char * name)
{
  const char * text = std::getenv(name);
  if (text == nullptr || *text == '\0') return std::nullopt;
  char * end = nullptr;
  errno = 0;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(value)) return std::nullopt;
  return value;
}

void decode_mavlink(
  Source & source, WorldOrigin & origin, const std::uint8_t * bytes, const std::size_t size)
{
  for (std::size_t offset = 0; offset < size; ++offset) {
    mavlink_message_t message{};
    if (mavlink_frame_char_buffer(&source.parser_buffer, &source.parser_status, bytes[offset],
      &message, nullptr) != MAVLINK_FRAMING_OK)
    {
      continue;
    }
    auto & vehicle = source.vehicle;
    // LOCAL_POSITION_NED when the vehicle offers it, and it wins outright.
    //
    // The global path below rebuilds NED by flat-earth from the FIRST fix it
    // receives, and that datum is whichever vehicle's packet happened to arrive
    // first -- including its altitude. Out in the open that only shifts the
    // whole fleet together and nobody notices. Inside a building it is fatal: a
    // robot on the top floor reporting first would put its own floor at zero and
    // every other level above or below the ground the building is drawn on. The
    // local frame is already what the world file is authored in, so taking it
    // directly removes the reconstruction and the trap with it.
    if (message.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
      mavlink_local_position_ned_t state{};
      mavlink_msg_local_position_ned_decode(&message, &state);
      source.local_frame = true;
      vehicle.north_m = state.x;
      vehicle.east_m = state.y;
      vehicle.down_m = state.z;
      vehicle.speed_mps = std::hypot(state.vx, state.vy);
      vehicle.published_unix_ms = unix_time_ms();
    } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT && !source.local_frame) {
      mavlink_global_position_int_t state{};
      mavlink_msg_global_position_int_decode(&message, &state);
      const double latitude = state.lat / 1e7;
      const double longitude = state.lon / 1e7;
      const double altitude = state.alt / 1000.0;
      if (!origin.horizontal_set) {
        origin.latitude = latitude;
        origin.longitude = longitude;
        origin.horizontal_set = true;
      }
      if (!origin.altitude_set) {
        origin.altitude = altitude;
        origin.altitude_set = true;
      }
      constexpr double metres_per_degree = 111319.49079327357;
      vehicle.north_m = static_cast<float>((latitude - origin.latitude) * metres_per_degree);
      vehicle.east_m = static_cast<float>((longitude - origin.longitude) * metres_per_degree *
        std::cos(origin.latitude * 0.017453292519943295));
      vehicle.down_m = static_cast<float>(origin.altitude - altitude);
      // Velocity here is centimetres per second.
      vehicle.speed_mps = std::hypot(state.vx, state.vy) * 0.01F;
      vehicle.published_unix_ms = unix_time_ms();
    } else if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t state{};
      mavlink_msg_heartbeat_decode(&message, &state);
      vehicle.armed = (state.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
      vehicle.ground = state.type == MAV_TYPE_GROUND_ROVER;
    } else if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) {
      mavlink_attitude_t state{};
      mavlink_msg_attitude_decode(&message, &state);
      const float half_roll = state.roll * 0.5F;
      const float half_pitch = state.pitch * 0.5F;
      const float half_yaw = state.yaw * 0.5F;
      const float cr = std::cos(half_roll), sr = std::sin(half_roll);
      const float cp = std::cos(half_pitch), sp = std::sin(half_pitch);
      const float cy = std::cos(half_yaw), sy = std::sin(half_yaw);
      vehicle.attitude_quaternion = {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy};
    }
  }
}

void append_vehicle(const Source & source, Snapshot & snapshot)
{
  const auto & vehicle = source.vehicle;
  if (std::isfinite(vehicle.north_m) && std::isfinite(vehicle.east_m) && std::isfinite(vehicle.down_m) &&
    vehicle.published_unix_ms != 0)
  {
    snapshot.vehicles[vehicle.id] = vehicle;
  }
}

void poll_px4(Source & source, WorldOrigin & origin, Snapshot & snapshot)
{
  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> packet{};
  for (;;) {
    const auto received = recv(source.fd, packet.data(), packet.size(), 0);
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    if (received < 0) {
      snapshot.error = source.vehicle.id + ": PX4 telemetry receive failed";
      break;
    }
    decode_mavlink(source, origin, packet.data(), static_cast<std::size_t>(received));
  }
  append_vehicle(source, snapshot);
}

/// The observer's latest word on the network.
///
/// `connected` tracks whether the watch stream is open, NOT how old the
/// snapshot is. The observer publishes only when the network changes, so a
/// settled fleet legitimately sends one snapshot and then nothing; ageing it
/// out would report a healthy mesh as stale.
struct NetworkView {
  ditto::observer::Snapshot snapshot;
  bool connected{};
  std::string error;
};

Color path_color(const ditto::observer::PathType type) {
  switch (type) {
    case ditto::observer::PathType::access_point: return {96, 165, 250, 255};
    case ditto::observer::PathType::p2p_wifi: return {251, 146, 60, 255};
    case ditto::observer::PathType::bluetooth: return {167, 139, 250, 255};
    case ditto::observer::PathType::web_socket: return {236, 72, 153, 255};
    case ditto::observer::PathType::cloud: return {250, 204, 21, 255};
    case ditto::observer::PathType::unspecified:
    default: return {148, 163, 184, 255};
  }
}

const char * transport_label(const std::string & kind) {
  if (kind == "bluetooth") return "BT";
  if (kind == "lan") return "LAN";
  if (kind == "lan_mdns") return "LAN MDNS";
  if (kind == "lan_multicast") return "LAN MCAST";
  if (kind == "awdl") return "AWDL";
  if (kind == "wifi_aware") return "WIFI-AW";
  if (kind == "multicast_beta") return "MCAST BETA";
  if (kind == "tcp_connect") return "TCP OUT";
  if (kind == "tcp_listen") return "TCP IN";
  if (kind == "websocket_connect") return "CLOUD";
  if (kind == "http_listen") return "HTTP";
  return kind.c_str();
}

/// Whether a transport can be switched on at all.
///
/// The two dialing kinds are live exactly when they have somewhere to dial, and
/// their addresses come from the configuration the node started with. A node
/// that was never given a cloud URL cannot be told to go and connect to one, so
/// offering the control would only earn a FailedPrecondition from Edge Server.
/// The observer reports the configured endpoints even while a transport is off,
/// which is what makes the distinction visible here.
/// Whether `node` still holds a transport that could carry `type`.
///
/// Two sources disagree for a while after a cut, and this reconciles them.
/// Transport state is read from each node directly and changes the instant it
/// is set; the presence graph that produces links is eventually consistent and
/// coalesced, and on a settled fleet nothing prompts it to reconverge. So a
/// link whose transport is already gone kept its line on screen until the fleet
/// next had something to say -- which made a cut look like it had not taken.
/// Drawing the intersection makes a cut leave the screen when it is cut.
///
/// This is one-directional on purpose. A transport being off proves the link
/// cannot exist, so the line goes immediately. A transport being back on proves
/// nothing about whether the link has re-formed, so restoring waits for
/// presence to actually report it -- which is the honest answer, since there is
/// no link to draw until there is.
bool can_carry(const ditto::observer::Node & node, const ditto::observer::PathType type)
{
  // For a node no observer could read, transports are unknown rather than
  // absent, so nothing is suppressed on its behalf.
  if (!node.reachable) return true;
  const auto enabled = [&node](const std::initializer_list<const char *> kinds) {
    return std::any_of(
      node.transports.begin(), node.transports.end(),
      [&kinds](const ditto::observer::TransportStatus & transport) {
        if (!transport.enabled) return false;
        return std::any_of(kinds.begin(), kinds.end(), [&transport](const char * kind) {
          return transport.kind == kind;
        });
      });
  };
  switch (type) {
    // A LAN link and a relay-proxied TCP one both present as `access_point`
    // and cannot be told apart in presence, so either transport keeps the path
    // alive. Cutting TCP with LAN still up correctly leaves the line drawn.
    case ditto::observer::PathType::access_point:
      return enabled({"lan", "tcp_connect", "tcp_listen"});
    case ditto::observer::PathType::p2p_wifi:
      return enabled({"awdl", "wifi_aware"});
    case ditto::observer::PathType::bluetooth:
      return enabled({"bluetooth"});
    case ditto::observer::PathType::web_socket:
      return enabled({"websocket_connect", "http_listen"});
    case ditto::observer::PathType::cloud:
      return enabled({"websocket_connect"});
    // No known carrier to check against.
    case ditto::observer::PathType::unspecified:
      break;
  }
  return true;
}

bool transport_actionable(
  const ditto::observer::TransportStatus & transport, const ditto::observer::Node & node) {
  if (transport.kind == "tcp_connect" || transport.kind == "websocket_connect") {
    return !transport.endpoints.empty();
  }
  // LAN discovery only does anything while the LAN transport itself is up.
  // Offering it otherwise would let someone enable mDNS and watch nothing
  // happen, which is the confusion these two kinds exist to remove.
  if (transport.kind == "lan_mdns" || transport.kind == "lan_multicast") {
    return std::any_of(
      node.transports.begin(), node.transports.end(),
      [](const ditto::observer::TransportStatus & sibling) {
        return sibling.kind == "lan" && sibling.enabled;
      });
  }
  return true;
}

/// A stable key for the unordered pair a link joins, so the several transports
/// between one pair can be spread apart instead of drawn on top of each other.
std::string pair_key(const ditto::observer::Link & link) {
  return link.peer_key_1 < link.peer_key_2 ?
    link.peer_key_1 + "|" + link.peer_key_2 : link.peer_key_2 + "|" + link.peer_key_1;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, request_shutdown);
  std::signal(SIGTERM, request_shutdown);
  std::vector<Source> sources;
  std::string network_metrics;
  std::string observer_endpoint;
  std::string world_path;
  std::string targets_path;
  std::string isr_socket, isr_scenario, isr_run_id;
  std::string isr_reset_script;
  std::uint16_t rtsp_port = 0;
  try {
    for (int index = 1; index < argc; ++index) {
      if (std::string(argv[index]) == "--gcs-port") {
        if (sources.empty() || ++index == argc) throw std::invalid_argument("GCS port needs a vehicle and port");
        const auto port = std::stoul(argv[index]);
        if (port == 0 || port > 65535) throw std::invalid_argument("GCS port out of range");
        sources.back().gcs_port = static_cast<std::uint16_t>(port);
        continue;
      }
      if (std::string(argv[index]) == "--rtsp") {
        if (++index == argc) throw std::invalid_argument("RTSP port is required");
        const auto port = std::stoul(argv[index]);
        if (port == 0 || port > 65534) throw std::invalid_argument("RTSP port out of range");
        rtsp_port = static_cast<std::uint16_t>(port);
        continue;
      }
      if (std::string(argv[index]) == "--world") {
        if (++index == argc) throw std::invalid_argument("world file path is required");
        world_path = argv[index];
        continue;
      }
      if (std::string(argv[index]) == "--targets") {
        if (++index == argc) throw std::invalid_argument("ISR targets file path is required");
        targets_path = argv[index];
        continue;
      }
      if (std::string(argv[index]) == "--isr-socket") {
        if (++index == argc) throw std::invalid_argument("ISR Edge Server socket is required");
        isr_socket = argv[index];
        continue;
      }
      if (std::string(argv[index]) == "--isr-scenario") {
        if (++index == argc) throw std::invalid_argument("ISR scenario ID is required");
        isr_scenario = argv[index];
        continue;
      }
      if (std::string(argv[index]) == "--isr-run-id") {
        if (++index == argc) throw std::invalid_argument("ISR run ID is required");
        isr_run_id = argv[index];
        continue;
      }
      if (std::string(argv[index]) == "--isr-reset-script") {
        if (++index == argc) throw std::invalid_argument("ISR reset script path is required");
        isr_reset_script = argv[index];
        continue;
      }
      if (std::string(argv[index]) == "--observer") {
        if (++index == argc) throw std::invalid_argument("observer endpoint is required");
        observer_endpoint = argv[index];
        continue;
      }
      if (std::string(argv[index]) == "--network-metrics") {
        if (++index == argc) throw std::invalid_argument("network metrics path is required");
        network_metrics = argv[index];
        continue;
      }
      if (std::string(argv[index]) != "--vehicle" || ++index == argc) {
        TraceLog(LOG_ERROR,
          "usage: %s [--network-metrics PATH] [--observer ADDR] [--world PATH] [--rtsp PORT]"
          " --vehicle ID --port TELEMETRY_PORT [--gcs-port GCS_PORT] [...]",
          argv[0]);
        return 2;
      }
      std::string vehicle_id(argv[index]);
      if (++index == argc || std::string(argv[index]) != "--port" || ++index == argc) {
        TraceLog(LOG_ERROR,
          "usage: %s [--network-metrics PATH] [--observer ADDR] [--world PATH] [--rtsp PORT]"
          " --vehicle ID --port TELEMETRY_PORT [--gcs-port GCS_PORT] [...]",
          argv[0]);
        return 2;
      }
      const auto port = std::stoul(argv[index]);
      if (port == 0 || port > 65535) throw std::invalid_argument("PX4 GCS port out of range");
      sources.emplace_back(std::move(vehicle_id), static_cast<std::uint16_t>(port));
    }
  } catch (const std::exception & error) {
    TraceLog(LOG_ERROR, "%s", error.what());
    return 2;
  }
  if (sources.empty()) {
    TraceLog(LOG_ERROR, "at least one PX4 SIH telemetry source is required");
    return 2;
  }
  std::vector<IsrTarget> isr_targets;
  try { isr_targets = read_isr_targets(targets_path); }
  catch (const std::exception & error) {
    TraceLog(LOG_ERROR, "%s", error.what());
    return 2;
  }
#ifdef DITTO_CESIUM_VIEWER
  const char * cesium_token = std::getenv("CESIUM_ACCESS_TOKEN");
  const char * use_cesium_env = std::getenv("SIM_VIEWER_USE_CESIUM");
  const bool use_cesium = use_cesium_env != nullptr && std::string(use_cesium_env) == "1";
  if (use_cesium && (cesium_token == nullptr || *cesium_token == '\0')) {
    TraceLog(LOG_ERROR, "Cesium scenario requires CESIUM_ACCESS_TOKEN in .env");
    return 2;
  }
#endif

  // Taken before the poller starts writing to `sources`.
  std::vector<std::string> vehicle_ids;
  for (const auto & source : sources) vehicle_ids.push_back(source.vehicle.id);

  std::mutex snapshot_mutex;
  Snapshot latest;
  std::atomic<bool> stopping{false};
  WorldOrigin origin;
  const auto home_latitude = environment_number("SIM_VIEWER_ORIGIN_LAT");
  const auto home_longitude = environment_number("SIM_VIEWER_ORIGIN_LON");
  if (home_latitude && home_longitude) {
    origin.latitude = *home_latitude;
    origin.longitude = *home_longitude;
    origin.horizontal_set = true;
  } else if (home_latitude || home_longitude) {
    TraceLog(LOG_WARNING, "ignoring incomplete SIM_VIEWER_ORIGIN coordinates");
  }
  std::thread poller([&]() {
    std::vector<pollfd> poll_fds;
    poll_fds.reserve(sources.size());
    for (const auto & source : sources) poll_fds.push_back({source.fd, POLLIN, 0});
    while (!stopping.load()) {
      Snapshot next;
      const int ready = poll(poll_fds.data(), static_cast<nfds_t>(poll_fds.size()), 100);
      if (ready < 0 && errno != EINTR) next.error = "PX4 telemetry poll failed";
      for (std::size_t index = 0; index < sources.size(); ++index) {
        if (ready > 0 && poll_fds[index].revents != 0) {
          poll_px4(sources[index], origin, next);
        } else {
          append_vehicle(sources[index], next);
        }
        poll_fds[index].revents = 0;
      }
      next.received_at = Clock::now();
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex);
        latest = std::move(next);
      }
    }
  });

  // The network observer. Optional: without --observer the viewer behaves as
  // before and simply draws no overlay.
  std::mutex network_mutex;
  NetworkView network;
  std::mutex stream_mutex;
  std::shared_ptr<ditto::observer::SnapshotStream> active_stream;
  std::thread observer_watcher;
  if (!observer_endpoint.empty()) {
    observer_watcher = std::thread([&]() {
      // One client for the process; its channel reconnects on its own, so only
      // the stream is rebuilt after a failure.
      ditto::observer::Client client(observer_endpoint);
      while (!stopping.load()) {
        auto stream = std::shared_ptr<ditto::observer::SnapshotStream>(client.watch(250));
        {
          std::lock_guard<std::mutex> lock(stream_mutex);
          active_stream = stream;
        }
        ditto::observer::Snapshot snapshot;
        bool delivered = false;
        while (!stopping.load() && stream->read(snapshot)) {
          delivered = true;
          std::lock_guard<std::mutex> lock(network_mutex);
          network.snapshot = snapshot;
          network.connected = true;
          network.error.clear();
        }
        const auto status = stream->finish();
        {
          std::lock_guard<std::mutex> lock(stream_mutex);
          active_stream.reset();
        }
        {
          std::lock_guard<std::mutex> lock(network_mutex);
          network.connected = false;
          if (!stopping.load()) {
            network.error = status.ok() ?
              (delivered ? "observer stream ended" : "observer sent nothing") :
              status.error_message();
          }
        }
        for (int attempt = 0; attempt < 10 && !stopping.load(); ++attempt) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
    });
  }

  // Where a toggle drops the snapshot it pulls once the change has applied.
  //
  // Held by `shared_ptr` because the toggle thread is detached: it may outlive
  // the click, and main, so it must not hold a reference to anything on main's
  // stack. The render thread is the only writer of `network`.
  struct PendingNetwork
  {
    std::mutex mutex;
    bool has{false};
    ditto::observer::Snapshot snapshot;
  };
  const auto pending_network = std::make_shared<PendingNetwork>();

  // Applying a change to twenty-one nodes took about a third of a second, so
  // it runs off the render thread. Everything is copied in: the thread may
  // outlive the click that started it.
  // Several kinds in one call, because some changes are only meaningful as a
  // set: cutting `tcp_connect` alone does not isolate a node, since peers keep
  // dialing in, so TCP has to go down in both directions at once.
  const auto send_toggle = [&observer_endpoint, pending_network](
    std::vector<std::string> node_ids, std::vector<std::pair<std::string, bool>> toggles) {
      if (observer_endpoint.empty()) return;
      std::thread(
        [endpoint = observer_endpoint, node_ids = std::move(node_ids),
          toggles = std::move(toggles), pending = pending_network]() {
          ditto::observer::Client client(endpoint, std::chrono::seconds(30));
          std::vector<ditto::observer::TransportResult> results;
          const auto status = client.set_transports(node_ids, toggles, results);
          if (!status.ok()) {
            TraceLog(LOG_WARNING, "transport change refused: %s", status.error_message().c_str());
            return;
          }
          for (const auto & result : results) {
            if (!result.applied) {
              TraceLog(LOG_WARNING, "%s: %s", result.node_id.c_str(), result.error.c_str());
            }
          }
          // Pull the network instead of waiting to be told about it. Cutting a
          // transport does not reliably move presence: on a settled fleet the
          // observer has nothing to report, so a link that is already gone kept
          // its line on screen until unrelated traffic woke the stream, which
          // made a cut look like it had not taken.
          ditto::observer::Snapshot refreshed;
          if (client.network(refreshed).ok()) {
            std::lock_guard<std::mutex> lock(pending->mutex);
            pending->snapshot = std::move(refreshed);
            pending->has = true;
          }
        })
        .detach();
    };

  SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
  InitWindow(1200, 800, "Ditto Edge Simulator — MVP Fleet");
  if (!IsWindowReady()) {
    TraceLog(LOG_ERROR, "could not create viewer window");
    stopping.store(true);
    {
      std::lock_guard<std::mutex> lock(stream_mutex);
      if (active_stream) active_stream->cancel();
    }
    if (observer_watcher.joinable()) observer_watcher.join();
    poller.join();
    return 1;
  }
  SetWindowMinSize(800, 560);
  SetTargetFPS(60);
#ifdef DITTO_CESIUM_VIEWER
  std::unique_ptr<sim::cesium::RaylibTileset> cesium_tiles;
  if (use_cesium) {
    try {
      cesium_tiles = std::make_unique<sim::cesium::RaylibTileset>(
        origin.latitude, origin.longitude, environment_number("SIM_VIEWER_ORIGIN_ALT").value_or(0.0),
        environment_number("SIM_VIEWER_RADIUS_M").value_or(500.0), cesium_token);
      if (!isr_targets.empty()) {
        std::vector<Vector2> sites;
        for (auto & target : isr_targets) {
          target.surface_ready = false;
          sites.push_back({target.east_m, target.north_m});
        }
        cesium_tiles->sample_surfaces(sites);
      }
    } catch (const std::exception& error) {
      TraceLog(LOG_ERROR, "could not start Cesium tiles: %s", error.what());
      stopping.store(true);
      {
        std::lock_guard<std::mutex> lock(stream_mutex);
        if (active_stream) active_stream->cancel();
      }
      if (observer_watcher.joinable()) observer_watcher.join();
      poller.join();
      CloseWindow();
      return 1;
    }
  }
#endif
  // Each vehicle's camera, as RTSP for a ground station. Optional, and a feed
  // that cannot start leaves the viewer running without one.
  std::unique_ptr<sim::video::Feeds> feeds;
  if (rtsp_port != 0) {
    try {
      feeds = std::make_unique<sim::video::Feeds>(rtsp_port, vehicle_ids);
    } catch (const std::exception & error) {
      TraceLog(LOG_ERROR, "no camera feeds: %s", error.what());
    }
  }
  std::vector<OcclusionSample> occlusion(feeds ? feeds->size() * isr_targets.size() : 0);
  std::unique_ptr<IsrPublisher> isr_publisher;
  if (!isr_targets.empty() && !isr_socket.empty())
    isr_publisher = std::make_unique<IsrPublisher>(isr_socket, isr_scenario, isr_run_id,
      origin.latitude, origin.longitude, isr_targets);
  Font ui_font = LoadFontEx("/System/Library/Fonts/SFNS.ttf", 48, nullptr, 0);
  SetTextureFilter(ui_font.texture, TEXTURE_FILTER_BILINEAR);
  const auto draw_text = [&ui_font](
    const char * text, const float x, const float y, const float size, const Color color) {
      DrawTextEx(ui_font, text, {x, y}, size, 0.5F, color);
    };

  // Static scenery, if the scenario names a world file. Display only; see
  // world.hpp. An absent or unreadable world leaves the ground bare.
#ifdef DITTO_CESIUM_VIEWER
  const sim::world::World world = sim::world::read(world_path, !use_cesium);
#else
  const sim::world::World world = sim::world::read(world_path);
#endif
  // A real city: kilometre sight lines, and vehicles that are specks from the
  // default view, so they get screen markers as well as their models.
#ifdef DITTO_CESIUM_VIEWER
  const bool city_map = world.model_loaded || cesium_tiles != nullptr;
#else
  const bool city_map = world.model_loaded;
#endif
  if (city_map) rlSetClipPlanes(1.0, 10000.0);
  if (!world_path.empty()) {
    if (world.empty()) {
#ifdef DITTO_CESIUM_VIEWER
      if (!cesium_tiles)
#endif
      TraceLog(LOG_WARNING, "no world objects drawn from %s", world_path.c_str());
    } else {
      TraceLog(LOG_INFO, "world '%s': %zu objects, %d model meshes from %s",
        world.name.c_str(), world.objects.size(), world.model_loaded ? world.model.meshCount : 0,
        world_path.c_str());
    }
  }

  // A world declaring its own extent overrides the fleet-size guess, so scenery
  // near the edge of the map is not left standing off the ground plane.
  const float map_size = world.extent_m > 0.0F
    ? world.extent_m
    : (sources.size() > 4 ? 240.0F : 100.0F);
  // Frame the world when there is one. The fleet-size guess is tuned for the
  // open-air demos, where twenty vehicles spread over a couple of hundred
  // metres; pointed at a fifty-metre building it opens on a speck in the middle
  // of an empty plane.
  // Frame the camera on what is drawn, not on the ground plane. The extent is
  // deliberately wider than the scenery -- 240 m of plane under 127 m of
  // buildings in the twenty-node world -- so opening at the extent puts the
  // fleet in the distance. A world with no objects falls back to the fleet-size
  // guess, which is all there is to go on.
  const sim::world::Bounds bounds = sim::world::content_bounds(world);
  const float initial_distance = world.view_distance_m > 0.0F ? world.view_distance_m : bounds.valid
    ? bounds.framing_span_m() * 1.15F
    : (sources.size() > 4 ? 140.0F : 48.0F);
  // The world framing above assumes these two; they live in world.hpp so the
  // assumption and the value cannot drift apart.
  float yaw = sim::world::kDefaultYawRad;
  float pitch = sim::world::kDefaultPitchRad;
  float distance = initial_distance;
  Camera3D camera{};
  Vector3 home_target = bounds.valid ? bounds.target() : Vector3{0.0F, 3.0F, 0.0F};
  if (world.view_distance_m > 0.0F) home_target.y = world.view_target_height_m;
  camera.target = home_target;
  camera.up = {0.0F, 1.0F, 0.0F};
  camera.fovy = 48.0F;
  camera.projection = CAMERA_PERSPECTIVE;
  std::unordered_map<std::string, Presentation> presentations;
  /// Walk phase per ground robot, advanced from its own speed so that stride
  /// rate follows the robot rather than the frame rate.
  std::unordered_map<std::string, float> gait_phase;
  std::unordered_map<std::string, NetworkStatus> link_status;
  auto next_metrics_read = Clock::now();
  std::string selected_vehicle;
  // Selecting a vehicle, on the map or in the sidebar, also follows it: the
  // camera snaps in behind it and rides along. Right-drag then orbits relative
  // to its heading; a pan, F, or a click on bare ground lets go.
  bool following = false;
  bool snap_follow = false;
  float follow_yaw_offset = 0.0F;
  const auto select_vehicle = [&](const std::string & id) {
    selected_vehicle = id;
    following = snap_follow = !id.empty();
    follow_yaw_offset = 0.0F;
  };
  // Opens already following one: for a headless capture of that view, or a demo.
  if (const char * preselect = std::getenv("SIM_VIEWER_SELECT")) select_vehicle(preselect);
  else if (feeds) selected_vehicle = sources.front().vehicle.id;
  // A left-press in the world both pans the camera and, if it turns out not to
  // have moved, selects whatever is under it. Only a release close to where the
  // press landed counts as a click.
  Vector2 press_at{};
  bool press_in_world = false;
  // A frame-local copy of the observer's view, refreshed only when it changes.
  NetworkView net;
  bool net_ever_seen = false;
  // Writes one PNG once the overlay has data, for checking the render without
  // a person at the screen. Unset in normal use. raylib joins this onto its
  // working directory, so it must be RELATIVE -- an absolute path silently
  // fails to save.
  const char * capture_path = std::getenv("SIM_VIEWER_CAPTURE");
  bool captured = false;
  int settle_frames = 0;
  pid_t reset_pid = -1;
  bool resetting = false, reset_process_done = false, reset_failed = false, reset_completed = false;
  std::int64_t reset_started_ms = 0;

  const Color sky = city_map ? Color{174, 214, 235, 255} : Color{238, 243, 247, 255};
  // Scenery as seen from `eye`, for the window (view 0) or a vehicle camera.
  const auto draw_scenery = [&](const std::size_t view, const Vector3 eye) {
#ifdef DITTO_CESIUM_VIEWER
    if (cesium_tiles) {
      cesium_tiles->draw(view);
      return;
    }
#endif
    (void)view;
    DrawPlane({0.0F, world.model_loaded ? -1.0F : -0.03F, 0.0F}, {map_size, map_size},
      world.model_loaded ? Color{157, 161, 162, 255} : Color{229, 235, 240, 255});
    if (!world.model_loaded) DrawGrid(static_cast<int>(map_size), 1.0F);
    sim::world::draw(world, eye);
  };

  while (!shutdown_requested && !WindowShouldClose()) {
    // Built once per frame, after `net` settles: the link lines and the legend
    // both filter on the same rule, so the panel cannot claim links that are
    // no longer drawn.
    std::unordered_map<std::string, const ditto::observer::Node *> peer_to_node_info;
    std::map<std::string, std::uint32_t> live_links_by_type;

    bool network_dirty = false;
    {
      std::lock_guard<std::mutex> lock(pending_network->mutex);
      if (pending_network->has) {
        std::lock_guard<std::mutex> network_lock(network_mutex);
        network.snapshot = std::move(pending_network->snapshot);
        pending_network->has = false;
        network_dirty = true;
      }
    }
    {
      std::lock_guard<std::mutex> lock(network_mutex);
      if (network_dirty ||
        network.snapshot.observed_unix_ms != net.snapshot.observed_unix_ms ||
        network.connected != net.connected || network.error != net.error)
      {
        net = network;
        net_ever_seen = net_ever_seen || net.snapshot.observed_unix_ms != 0;
      }
    }
    for (const auto & node : net.snapshot.nodes) {
      if (!node.peer_key.empty()) peer_to_node_info[node.peer_key] = &node;
    }
    for (const auto & link : net.snapshot.links) {
      const auto first = peer_to_node_info.find(link.peer_key_1);
      const auto second = peer_to_node_info.find(link.peer_key_2);
      if (first != peer_to_node_info.end() && !can_carry(*first->second, link.type)) continue;
      if (second != peer_to_node_info.end() && !can_carry(*second->second, link.type)) continue;
      ++live_links_by_type[ditto::observer::path_type_name(link.type)];
    }

    // The node inspector accepts clicks, so its rectangle has to be known
    // before the camera decides a left-drag is a pan.
    const ditto::observer::Node * inspected = nullptr;
    if (!selected_vehicle.empty()) {
      for (const auto & node : net.snapshot.nodes) {
        if (node.node_id == selected_vehicle) {
          inspected = &node;
          break;
        }
      }
    }
    const std::size_t chip_rows = inspected != nullptr && !inspected->transports.empty() ?
      (inspected->transports.size() + 2) / 3 : 0;
    const float inspector_height = chip_rows == 0 ? 126.0F :
      148.0F + 28.0F * static_cast<float>(chip_rows);
    const Rectangle inspector{24.0F, 106.0F, 330.0F, inspector_height};
    const bool pointer_on_inspector = !selected_vehicle.empty() &&
      CheckCollisionPointRec(GetMousePosition(), inspector);

    const float sidebar_width = std::clamp(GetScreenWidth() * 0.30F, 280.0F, 340.0F);
    const float sidebar_left = static_cast<float>(GetScreenWidth()) - sidebar_width;
    if (!pointer_on_inspector && GetMousePosition().x < sidebar_left &&
      IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
      const auto delta = GetMouseDelta();
      (following ? follow_yaw_offset : yaw) -= delta.x * 0.006F;
      pitch = std::clamp(pitch + delta.y * 0.006F, 0.12F, 1.35F);
    }
    if (!pointer_on_inspector && GetMousePosition().x < sidebar_left &&
      IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
      press_at = GetMousePosition();
      press_in_world = true;
    }
    if (!pointer_on_inspector && GetMousePosition().x < sidebar_left &&
      IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
      if (following && Vector2Distance(press_at, GetMousePosition()) > 4.0F) following = false;
      const auto delta = GetMouseDelta();
      const Vector3 position{
        camera.target.x + std::sin(yaw) * std::cos(pitch) * distance,
        camera.target.y + std::sin(pitch) * distance,
        camera.target.z + std::cos(yaw) * std::cos(pitch) * distance,
      };
      const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, position));
      const Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera.up));
      const Vector3 ground_forward = Vector3Normalize({forward.x, 0.0F, forward.z});
      const float scale = distance * 0.0015F;
      camera.target = Vector3Add(camera.target, Vector3Scale(right, -delta.x * scale));
      camera.target = Vector3Add(camera.target, Vector3Scale(ground_forward, delta.y * scale));
    }
    // Proportional, so a notch means as much at a 500 m city view as at a
    // chase view a few metres behind one drone.
    distance = std::clamp(distance * std::pow(0.88F, GetMouseWheelMove()), 3.0F, map_size * 2.0F);
    if (IsKeyPressed(KEY_F)) {
      following = false;
      yaw = sim::world::kDefaultYawRad;
      pitch = sim::world::kDefaultPitchRad;
      distance = initial_distance;
      camera.target = home_target;
    }

    Snapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex);
      snapshot = latest;
    }
    const auto render_time = Clock::now();
    if (render_time >= next_metrics_read) {
      link_status = read_network_metrics(network_metrics);
      next_metrics_read = render_time + std::chrono::milliseconds(250);
    }
    // After the read, so a file published mid-frame is never stamped ahead of
    // the clock it is compared against.
    const auto now = unix_time_ms();
    const bool live = !snapshot.vehicles.empty() && render_time - snapshot.received_at < std::chrono::seconds(2);
    std::unordered_map<std::string, Vehicle> visible;
    for (const auto & [id, vehicle] : snapshot.vehicles) {
      auto found = presentations.find(id);
      if (found == presentations.end()) {
        presentations.emplace(id, Presentation{vehicle, vehicle, render_time, render_time});
      } else if (vehicle.published_unix_ms != found->second.to.published_unix_ms) {
        const auto previous = presented(found->second, render_time);
        const auto interval_ms = std::clamp(
          vehicle.published_unix_ms - found->second.to.published_unix_ms, std::int64_t{80},
          std::int64_t{220});
        found->second = {previous, vehicle, render_time, render_time + std::chrono::milliseconds(interval_ms)};
      }
      visible.emplace(id, presented(presentations.at(id), render_time));
    }
    if (reset_pid > 0) {
      int status = 0;
      const pid_t done = waitpid(reset_pid, &status, WNOHANG);
      if (done == reset_pid) {
        reset_pid = -1;
        reset_process_done = WIFEXITED(status) && WEXITSTATUS(status) == 0;
        if (!reset_process_done) { resetting = false; reset_failed = true; }
      }
    }
    if (resetting && reset_process_done && (!isr_publisher || isr_publisher->idle())) {
      const bool at_origin = visible.size() == sources.size() && std::all_of(
        visible.begin(), visible.end(), [reset_started_ms, now](const auto & entry) {
          const auto & vehicle = entry.second;
          return vehicle.published_unix_ms > reset_started_ms &&
            now - vehicle.published_unix_ms < 3000 &&
            std::hypot(vehicle.north_m, vehicle.east_m) < 2.0F &&
            std::abs(vehicle.down_m + 5.0F) < 2.0F;
        });
      if (at_origin) { resetting = false; reset_completed = true; }
    }
    if (resetting && now - reset_started_ms > 180000) { resetting = false; reset_failed = true; }
    if (following) {
      if (const auto found = visible.find(selected_vehicle); found != visible.end()) {
        const Vehicle & vehicle = found->second;
        camera.target = sim::world::view_point(vehicle.east_m, -vehicle.down_m, vehicle.north_m);
        // camera.position sits at (sin yaw, cos yaw) from the target, so this
        // yaw puts it opposite the nose. Eased, so attitude jitter does not
        // shake the view, except on the snap.
        const Vector3 nose = rotate_by_px4_quaternion(vehicle, {1.0F, 0.0F, 0.0F});
        if (std::hypot(nose.x, nose.z) > 0.1F) {
          const float behind = std::atan2(-nose.x, -nose.z) + follow_yaw_offset;
          yaw += std::remainder(behind - yaw, 2.0F * PI) *
            (snap_follow ? 1.0F : 1.0F - std::exp(-4.0F * GetFrameTime()));
        }
        if (snap_follow) {
          pitch = 0.3F;
          distance = 6.0F;
          snap_follow = false;
        }
      }
    }
    camera.position = {
      camera.target.x + std::sin(yaw) * std::cos(pitch) * distance,
      camera.target.y + std::sin(pitch) * distance,
      camera.target.z + std::cos(yaw) * std::cos(pitch) * distance,
    };
#ifdef DITTO_CESIUM_VIEWER
    if (cesium_tiles) {
      // The projection spans the whole window (the sidebar only scissors it),
      // so tile selection must see the same frustum or it culls the left edge.
      cesium_tiles->update(camera, GetScreenWidth(), GetScreenHeight(), GetFrameTime());
      for (std::size_t i = 0; i < isr_targets.size(); ++i) {
        if (const auto height = cesium_tiles->surface_height(i)) {
          isr_targets[i].surface_m = *height;
          isr_targets[i].surface_ready = true;
        }
      }
    }
#endif
    const float prop_degrees = static_cast<float>(std::fmod(GetTime() * 3.5, 1.0) * 360.0);

    // Vehicle cameras, each drawn only while a ground station is watching it.
    for (std::size_t index = 0; feeds && index < feeds->size(); ++index) {
      const auto own = visible.find(feeds->name(index));
      if (own == visible.end() || !feeds->due(index, render_time)) continue;
      const Camera3D eye = nose_camera(own->second);
#ifdef DITTO_CESIUM_VIEWER
      if (cesium_tiles) cesium_tiles->update(eye, sim::video::Feeds::kWidth, sim::video::Feeds::kHeight, GetFrameTime(), index + 1);
#endif
      feeds->begin(index);
      ClearBackground(sky);
      BeginMode3D(eye);
      draw_scenery(index + 1, eye.position);
      for (const auto & [id, vehicle] : visible) {
        if (id == own->first) continue;
        const Vector3 position = sim::world::view_point(vehicle.east_m, -vehicle.down_m, vehicle.north_m);
        if (!vehicle.ground) draw_drone(vehicle, position, prop_degrees, eye.position);
        else if (const auto phase = gait_phase.find(id); phase != gait_phase.end()) draw_robot(vehicle, position, phase->second);
      }
      std::vector<std::pair<std::size_t, TargetBox>> recognized;
      for (std::size_t i = 0; i < isr_targets.size(); ++i) {
        const auto & target = isr_targets[i];
        if (!target.surface_ready) continue;
        const auto box = projected_target(target, eye, sim::video::Feeds::kWidth, sim::video::Feeds::kHeight);
        if (!resetting && box) {
          auto & sample = occlusion[index * isr_targets.size() + i];
          if (sample.checked == Clock::time_point{} || render_time - sample.checked >= std::chrono::milliseconds(200)) {
            sample.clear = target_unoccluded(target, eye, sim::video::Feeds::kWidth, sim::video::Feeds::kHeight);
            sample.checked = render_time;
          }
          if (sample.clear) recognized.emplace_back(i, *box);
        }
      }
      for (const auto & target : isr_targets) if (target.surface_ready) {
        DrawCube(target.center(), 2.2F, 2.2F, 2.2F, {196, 36, 48, 255});
        DrawCubeWires(target.center(), 2.25F, 2.25F, 2.25F, {255, 78, 78, 255});
      }
      EndMode3D();
      for (const auto & [target_index, box] : recognized) {
        auto & target = isr_targets[target_index];
        DrawRectangleLinesEx(box.pixels, 3.0F, {255, 48, 48, 255});
        const std::string caption = target.id + "  TARGET ACQUIRED  " +
          std::to_string(static_cast<int>(box.distance_m)) + "m";
        const float label_y = std::max(0.0F, box.pixels.y - 28.0F);
        const float label_w = MeasureTextEx(ui_font, caption.c_str(), 17, 0.5F).x + 18.0F;
        DrawRectangle(static_cast<int>(box.pixels.x), static_cast<int>(label_y),
          static_cast<int>(label_w), 25, {152, 24, 33, 235});
        draw_text(caption.c_str(), box.pixels.x + 8.0F, label_y + 3.0F, 17, RAYWHITE);
        if (!target.found) {
          target.found = true;
          if (isr_publisher) isr_publisher->publish(target, true, own->first);
        }
      }
#ifdef DITTO_CESIUM_VIEWER
      // The tiles' licence wants their credit wherever they are shown, and on
      // a shaded strip, since the ground behind it is often white concrete.
      if (cesium_tiles && !cesium_tiles->attribution().empty()) {
        const char * credit = cesium_tiles->attribution().c_str();
        constexpr int strip = 26;
        DrawRectangle(0, sim::video::Feeds::kHeight - strip,
          static_cast<int>(MeasureTextEx(ui_font, credit, 14, 0.5F).x) + 24, strip, Fade(BLACK, 0.45F));
        draw_text(credit, 12, static_cast<float>(sim::video::Feeds::kHeight - strip + 6), 14, RAYWHITE);
      }
#endif
      feeds->end(index, render_time);
    }

    BeginDrawing();
    ClearBackground(sky);
    BeginScissorMode(0, 0, static_cast<int>(sidebar_left), GetScreenHeight());
    BeginMode3D(camera);
    draw_scenery(0, camera.position);
    for (const auto & target : isr_targets) if (target.surface_ready) {
      DrawCube(target.center(), 2.2F, 2.2F, 2.2F, {196, 36, 48, 255});
      DrawCubeWires(target.center(), 2.25F, 2.25F, 2.25F, {255, 78, 78, 255});
    }
    for (const auto & [id, vehicle] : visible) {
      const Vector3 position = sim::world::view_point(vehicle.east_m, -vehicle.down_m, vehicle.north_m);
      if (vehicle.ground) {
        // One full cycle is two steps, so the phase advances by the distance
        // covered divided by a stride pair. A stopped robot holds its phase
        // rather than resetting, which would snap the legs together.
        constexpr float stride_pair_m = 1.1F;
        float & phase = gait_phase[id];
        phase = std::fmod(
          phase + vehicle.speed_mps / stride_pair_m * 2.0F * PI * GetFrameTime(), 2.0F * PI);
        draw_robot(vehicle, position, phase);
      } else {
        draw_drone(vehicle, position, prop_degrees, camera.position);
      }
    }

    // The network the observer reports, drawn over the fleet.
    std::unordered_map<std::string, Vector3> node_positions;
    if (net_ever_seen) {
      for (const auto & [id, vehicle] : visible) {
        node_positions[id] = sim::world::view_point(vehicle.east_m, -vehicle.down_m, vehicle.north_m);
      }
      // Nodes with no vehicle of their own -- the operator, normally. Spread
      // them on a small ring at the origin so several never sit on one spot.
      std::size_t placed = 0;
      for (const auto & node : net.snapshot.nodes) {
        if (node_positions.count(node.node_id) != 0) continue;
        const float angle = static_cast<float>(placed) * 1.2F;
        node_positions[node.node_id] = {std::sin(angle) * 4.0F, 1.0F, std::cos(angle) * 4.0F};
        DrawCube(node_positions[node.node_id], 1.6F, 1.6F, 1.6F, {148, 163, 184, 255});
        DrawCubeWires(node_positions[node.node_id], 1.6F, 1.6F, 1.6F, RAYWHITE);
        ++placed;
      }

      std::unordered_map<std::string, std::string> peer_to_node;
      for (const auto & node : net.snapshot.nodes) {
        if (!node.peer_key.empty()) peer_to_node[node.peer_key] = node.node_id;
      }

      // Several transports between one pair must not be drawn on top of each
      // other, so each gets its own lane, offset across the pair's axis. Link
      // order is stable, so a lane does not change between frames.
      std::unordered_map<std::string, int> lanes;
      for (const auto & link : net.snapshot.links) {
        if (link.type == ditto::observer::PathType::cloud) continue;
        const auto first = peer_to_node.find(link.peer_key_1);
        const auto second = peer_to_node.find(link.peer_key_2);
        if (first == peer_to_node.end() || second == peer_to_node.end()) continue;
        // Either endpoint having lost the transport means this link is gone,
        // whatever presence still says. See can_carry.
        const auto first_info = peer_to_node_info.find(link.peer_key_1);
        const auto second_info = peer_to_node_info.find(link.peer_key_2);
        if (first_info != peer_to_node_info.end() &&
          !can_carry(*first_info->second, link.type)) continue;
        if (second_info != peer_to_node_info.end() &&
          !can_carry(*second_info->second, link.type)) continue;
        const auto from = node_positions.find(first->second);
        const auto to = node_positions.find(second->second);
        if (from == node_positions.end() || to == node_positions.end()) continue;

        const int lane = lanes[pair_key(link)]++;
        const Vector3 axis = Vector3Subtract(to->second, from->second);
        Vector3 across = Vector3Normalize({-axis.z, 0.0F, axis.x});
        if (!std::isfinite(across.x) || !std::isfinite(across.z)) across = {1.0F, 0.0F, 0.0F};
        const float step = lane == 0 ? 0.0F :
          (lane % 2 == 1 ? 1.0F : -1.0F) * 0.8F * static_cast<float>((lane + 1) / 2);
        const Vector3 shift = Vector3Scale(across, step);
        DrawLine3D(Vector3Add(from->second, shift), Vector3Add(to->second, shift),
          path_color(link.type));
      }

      // The cloud link has no second endpoint, so it is a property of a node
      // rather than an edge: a stalk rising out of whoever holds one.
      for (const auto & node : net.snapshot.nodes) {
        if (!node.connected_to_cloud) continue;
        // The cloud flag comes from the peer and lingers past a cut exactly as
        // a link does, so the stalk is suppressed the same way.
        if (!can_carry(node, ditto::observer::PathType::cloud)) continue;
        const auto position = node_positions.find(node.node_id);
        if (position == node_positions.end()) continue;
        // Sized to the scene rather than fixed. A 6 m stalk reads well over a
        // 170 m airfield and goes straight up through two floors of a building
        // with 4 m storeys, which is worse than not drawing it: it puts a cloud
        // marker in a room that does not hold that node. Scaling it by the same
        // content span the camera is framed on keeps it legible in both.
        const float stalk = bounds.valid ? bounds.framing_span_m() * 0.045F : 6.0F;
        const Vector3 top = Vector3Add(position->second, {0.0F, stalk, 0.0F});
        DrawLine3D(position->second, top, path_color(ditto::observer::PathType::cloud));
        DrawSphere(top, stalk * 0.07F, path_color(ditto::observer::PathType::cloud));
      }
    }
    EndMode3D();
    EndScissorMode();

    for (const auto & target : isr_targets) if (target.surface_ready) {
      const Vector3 toward = Vector3Subtract(target.center(), camera.position);
      if (Vector3DotProduct(toward, Vector3Subtract(camera.target, camera.position)) <= 0.0F) continue;
      const Vector2 at = GetWorldToScreen(target.center(), camera);
      if (at.x < 0 || at.x >= sidebar_left || at.y < 110 || at.y >= GetScreenHeight() - 32) continue;
      const std::string label = target.id + (target.found ? "  FOUND" : "  TARGET");
      const float width = MeasureTextEx(ui_font, label.c_str(), 13, 0.5F).x + 14.0F;
      const float label_x = std::clamp(at.x - width * 0.5F, 4.0F, sidebar_left - width - 4.0F);
      DrawRectangleRounded({label_x, at.y - 36.0F, width, 20.0F}, 0.25F, 4,
        target.found ? Color{123, 28, 39, 235} : Color{72, 33, 39, 235});
      draw_text(label.c_str(), label_x + 7.0F, at.y - 33.0F, 13, RAYWHITE);
    }

#ifndef DITTO_CESIUM_VIEWER
    for (const auto & landmark : world.landmarks) {
      const Vector2 at = GetWorldToScreen(landmark.position, camera);
      if (at.x < 0 || at.x >= sidebar_left || at.y < 100 || at.y >= GetScreenHeight() - 32) continue;
      const float width = MeasureTextEx(ui_font, landmark.name.c_str(), 13, 0.5F).x + 16.0F;
      const Rectangle tag{at.x - width * 0.5F, at.y - 22.0F, width, 21.0F};
      DrawRectangleRounded(tag, 0.25F, 4, {30, 41, 59, 225});
      draw_text(landmark.name.c_str(), tag.x + 8.0F, tag.y + 4.0F, 13, RAYWHITE);
    }
#endif
    if (city_map) {
      std::vector<std::string> ids;
      ids.reserve(visible.size());
      for (const auto & [id, vehicle] : visible) ids.push_back(id);
      std::sort(ids.begin(), ids.end(), vehicle_less);
      std::vector<Vector2> placed;
      for (const auto & id : ids) {
        const auto & vehicle = visible.at(id);
        const Vector3 over = sim::world::view_point(vehicle.east_m, -vehicle.down_m + 2.0F, vehicle.north_m);
        // Close enough to see the model, or behind the camera where the
        // projection folds onto the screen: no marker.
        const Vector3 from_camera = Vector3Subtract(over, camera.position);
        if (Vector3Length(from_camera) < 40.0F ||
          Vector3DotProduct(from_camera, Vector3Subtract(camera.target, camera.position)) <= 0.0F) continue;
        const Vector2 at = GetWorldToScreen(over, camera);
        if (at.x < 10 || at.x >= sidebar_left - 10 || at.y < 100 || at.y >= GetScreenHeight() - 30) continue;
        const Color color = now - vehicle.published_unix_ms > 2000 ? ORANGE : vehicle_color(id);
        DrawCircleV(at, 7.0F, color);
        DrawCircleLines(static_cast<int>(at.x), static_cast<int>(at.y), 10.0F, RAYWHITE);
        const float width = MeasureTextEx(ui_font, id.c_str(), 13, 0.5F).x + 14.0F;
        Vector2 label{std::min(at.x + 12.0F, sidebar_left - width - 4.0F), at.y - 11.0F};
        for (const auto & prior : placed) {
          if (std::abs(label.x - prior.x) < 80.0F && std::abs(label.y - prior.y) < 20.0F)
            label.y += 24.0F;
        }
        placed.push_back(label);
        DrawRectangleRounded({label.x, label.y, width, 20.0F}, 0.25F, 4, {30, 41, 59, 225});
        draw_text(id.c_str(), label.x + 7.0F, label.y + 4.0F, 13, RAYWHITE);
      }
    }

    if (!selected_vehicle.empty()) {
      const auto vehicle = snapshot.vehicles.find(selected_vehicle);
      if (vehicle != snapshot.vehicles.end()) {
        const auto status = link_status.find(selected_vehicle);
        const auto age = std::max<std::int64_t>(0, now - vehicle->second.published_unix_ms);
        DrawRectangleRounded(inspector, 0.08F, 6, {30, 41, 59, 238});
        draw_text("NODE INSPECTOR", 38, 118, 16, RAYWHITE);
        draw_text(selected_vehicle.c_str(), 38, 141, 18, {101, 214, 159, 255});
        draw_text(TextFormat("PX4  %s   telemetry age  %lld ms",
          vehicle->second.armed ? "ARMED" : "DISARMED", static_cast<long long>(age)), 38, 166, 13, RAYWHITE);
        const char * edge = "SIMULATED LINK: awaiting relay";
        if (status != link_status.end() && fresh(status->second, now)) {
          edge = TextFormat("SIMULATED MESH: %u / %u links connected", status->second.connected_links,
            status->second.links);
        }
        draw_text(edge, 38, 187, 13, RAYWHITE);
        if (status != link_status.end() && fresh(status->second, now)) {
          draw_text(TextFormat("TX %.1f kbps (%.2f%%)  RX %.1f kbps (%.2f%%)",
            status->second.tx_bps / 1000.0, status->second.tx_utilization,
            status->second.rx_bps / 1000.0, status->second.rx_utilization),
            38, 208, 13, {174, 193, 212, 255});
        }

        // Transports, as the observer reports them for this node. Clicking one
        // asks the observer to enable or disable it on this node alone.
        if (inspected != nullptr && !inspected->transports.empty()) {
          const bool any_unconfigured = std::any_of(
            inspected->transports.begin(), inspected->transports.end(),
            [&inspected](const ditto::observer::TransportStatus & transport) {
              return !transport_actionable(transport, *inspected);
            });
          const char * chip_hint = !inspected->reachable ? "TRANSPORTS  (node unreachable)" :
            any_unconfigured ? "TRANSPORTS  (dimmed: cannot be enabled as configured)" :
            "TRANSPORTS  (click to toggle)";
          draw_text(chip_hint, 38, 230, 12,
            inspected->reachable ? Color{174, 193, 212, 255} : Color{255, 183, 77, 255});
          float chip_x = 38.0F;
          float chip_y = 248.0F;
          for (const auto & transport : inspected->transports) {
            const Rectangle chip{chip_x, chip_y, 98.0F, 24.0F};
            const bool actionable = transport_actionable(transport, *inspected);
            const Color fill = transport.enabled ? Color{22, 101, 52, 255} :
              (actionable ? Color{55, 65, 81, 255} : Color{39, 44, 54, 255});
            DrawRectangleRounded(chip, 0.25F, 4, fill);
            if (transport.enabled) {
              DrawRectangleRoundedLines(chip, 0.25F, 4, {101, 214, 159, 255});
            }
            const Color label_color = transport.enabled ? RAYWHITE :
              (actionable ? Color{148, 163, 184, 255} : Color{88, 97, 112, 255});
            draw_text(transport_label(transport.kind), chip.x + 8.0F, chip.y + 5.0F, 12,
              label_color);
            if (inspected->reachable && actionable && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
              CheckCollisionPointRec(GetMousePosition(), chip))
            {
              send_toggle({inspected->node_id}, {{transport.kind, !transport.enabled}});
            }
            chip_x += 102.0F;
            if (chip_x > 38.0F + 2.0F * 102.0F) {
              chip_x = 38.0F;
              chip_y += 28.0F;
            }
          }
        }
      }
    }

    draw_text("DITTO EDGE SIMULATOR — LIVE PX4 FLEET", 24, 20, 24, {30, 41, 59, 255});
    draw_text("Left-drag: pan   Right-drag: orbit   Wheel: zoom   F: reset   "
      "Click a node, then a transport chip", 24, 50, 16, GRAY);
    draw_text(live ? "LIVE" : "WAITING FOR VEHICLE STATE", 24, 78, 18,
      live ? DARKGREEN : MAROON);

    DrawRectangle(static_cast<int>(sidebar_left), 0, static_cast<int>(sidebar_width),
      GetScreenHeight(), {30, 41, 59, 255});
    DrawRectangle(static_cast<int>(sidebar_left), 0, 1, GetScreenHeight(), {121, 146, 173, 255});
    draw_text("FLEET STATUS", sidebar_left + 16.0F, 18, 20, RAYWHITE);
    const Color fleet_color = live ? Color{101, 214, 159, 255} : Color{255, 183, 77, 255};
    draw_text(TextFormat("%d / %d live", static_cast<int>(snapshot.vehicles.size()),
      static_cast<int>(sources.size())), sidebar_left + 16.0F, 46, 14, fleet_color);
    draw_text("Vehicle position                         Network", sidebar_left + 16.0F, 72, 13,
      {174, 193, 212, 255});

    std::vector<std::string> vehicle_ids;
    vehicle_ids.reserve(snapshot.vehicles.size());
    for (const auto & [id, vehicle] : snapshot.vehicles) vehicle_ids.push_back(id);
    std::sort(vehicle_ids.begin(), vehicle_ids.end(), vehicle_less);
    const float row_height = std::clamp(
      (GetScreenHeight() - 94.0F - (feeds ? 120.0F : 0.0F)) /
        std::max(1.0F, static_cast<float>(vehicle_ids.size())),
      feeds ? 18.0F : 21.0F, 27.0F);
    float row = 94.0F;
    for (const auto & id : vehicle_ids) {
      const auto & vehicle = snapshot.vehicles.at(id);
      const auto age = now - vehicle.published_unix_ms;
      const auto color = age > 2000 ? ORANGE : vehicle_color(id);
      if (id == selected_vehicle) {
        DrawRectangle(static_cast<int>(sidebar_left + 4.0F), static_cast<int>(row + 1.0F),
          static_cast<int>(sidebar_width - 8.0F), static_cast<int>(row_height - 2.0F), {55, 84, 112, 255});
      }
      DrawLine(static_cast<int>(sidebar_left + 12.0F), static_cast<int>(row + row_height - 2.0F),
        GetScreenWidth() - 12, static_cast<int>(row + row_height - 2.0F), {55, 70, 89, 255});
      draw_text(TextFormat("%s  N%.0f E%.0f A%.0f", id.c_str(), vehicle.north_m, vehicle.east_m,
        -vehicle.down_m), sidebar_left + 16.0F, row + 4.0F, 13, color);
      const auto found = link_status.find(id);
      const bool link_live = found != link_status.end() && fresh(found->second, now);
      const auto utilization = link_live ?
        std::max(found->second.tx_utilization, found->second.rx_utilization) : 0.0;
      const bool connected = link_live && found->second.connected;
      const char * label = connected ? TextFormat("%u/%u %.0f%%", found->second.connected_links,
        found->second.links, utilization) : "NO LINK";
      const Color button_color = connected ?
          (utilization >= 90.0 ? MAROON : utilization >= 70.0 ? ORANGE : DARKGREEN) : GRAY;
      const Rectangle button{static_cast<float>(GetScreenWidth()) - 96.0F, row + 2.0F,
        80.0F, row_height - 5.0F};
      DrawRectangleRounded(button, 0.2F, 4, button_color);
      draw_text(label, button.x + 8.0F, button.y + 3.0F, 12, RAYWHITE);
      const Rectangle row_button{sidebar_left, row, sidebar_width - 104.0F, row_height};
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(GetMousePosition(), row_button)) {
        select_vehicle(id);
      }
      row += row_height;
    }
    if (!isr_reset_script.empty()) {
      const int found_count = static_cast<int>(std::count_if(isr_targets.begin(), isr_targets.end(),
        [](const IsrTarget & target) { return target.found; }));
      draw_text(TextFormat("ISR TARGETS  %d / %d found", found_count,
        static_cast<int>(isr_targets.size())), sidebar_left + 16.0F,
        static_cast<float>(GetScreenHeight()) - 284.0F, 14, RAYWHITE);
      for (std::size_t i = 0; i < isr_targets.size(); ++i) {
        const auto & target = isr_targets[i];
        draw_text(TextFormat("%s  N%.0f E%.0f  %s", target.id.c_str(), target.north_m,
          target.east_m, target.found ? "FOUND" : "UNSEEN"), sidebar_left + 16.0F,
          static_cast<float>(GetScreenHeight()) - 259.0F + static_cast<float>(i) * 20.0F,
          12, target.found ? Color{255, 124, 124, 255} : Color{174, 193, 212, 255});
      }
      const Rectangle reset_button{sidebar_left + 16.0F, static_cast<float>(GetScreenHeight()) - 169.0F,
        sidebar_width - 32.0F, 36.0F};
      DrawRectangleRounded(reset_button, 0.18F, 5,
        resetting ? Color{78, 91, 107, 255} : Color{153, 37, 45, 255});
      draw_text(resetting ? "RESETTING MISSION..." : "RESET MISSION",
        reset_button.x + 12.0F, reset_button.y + 8.0F, 16, RAYWHITE);
      if (reset_failed) draw_text("Reset failed — see runtime/isr-reset.log",
        reset_button.x, reset_button.y - 22.0F, 12, {255, 183, 77, 255});
      else if (reset_completed) draw_text("Fleet home; target state cleared",
        reset_button.x, reset_button.y - 22.0F, 12, {101, 214, 159, 255});
      if (!resetting && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
        CheckCollisionPointRec(GetMousePosition(), reset_button))
      {
        resetting = true; reset_failed = reset_completed = reset_process_done = false;
        reset_started_ms = unix_time_ms();
        for (auto & target : isr_targets) {
          target.found = false;
          if (isr_publisher) isr_publisher->publish(target, false, "");
        }
        char * args[] = {const_cast<char *>("/bin/bash"),
          const_cast<char *>(isr_reset_script.c_str()), nullptr};
        const int error = posix_spawn(&reset_pid, "/bin/bash", nullptr, nullptr, args, environ);
        if (error != 0) { reset_pid = -1; resetting = false; reset_failed = true; }
      }
    }
    if (feeds && !selected_vehicle.empty()) {
      const auto source = std::find_if(sources.begin(), sources.end(), [&](const Source & item) {
        return item.vehicle.id == selected_vehicle;
      });
      if (source != sources.end()) {
        const float top = static_cast<float>(GetScreenHeight()) - 112.0F;
        DrawLine(static_cast<int>(sidebar_left + 12.0F), static_cast<int>(top - 8.0F),
          GetScreenWidth() - 12, static_cast<int>(top - 8.0F), {121, 146, 173, 255});
        draw_text("CAMERA & MAVLINK (LOCAL)", sidebar_left + 16.0F, top, 14, RAYWHITE);
        draw_text(TextFormat("rtsp://localhost:%u/%s", rtsp_port, selected_vehicle.c_str()),
          sidebar_left + 16.0F, top + 25.0F, 12, {101, 214, 159, 255});
        draw_text(TextFormat("Telemetry UDP: %u", source->telemetry_port),
          sidebar_left + 16.0F, top + 50.0F, 12, {174, 193, 212, 255});
        draw_text(source->gcs_port ? TextFormat("GCS MAVLink TCP: %u", source->gcs_port) :
          "GCS MAVLink TCP: unavailable", sidebar_left + 16.0F, top + 72.0F, 12,
          {174, 193, 212, 255});
      }
    }
    // What the observer says the network is, and the one control that makes
    // the point: the cloud is a path nobody configured a mesh for.
    Rectangle legend_panel{0.0F, 0.0F, 0.0F, 0.0F};
    if (!observer_endpoint.empty()) {
      // Only nodes that were started with a cloud URL can be told to use one,
      // so they are the only ones the fleet button addresses. Sending to the
      // whole fleet would earn a FailedPrecondition from every node with none.
      std::vector<std::string> cloud_nodes;
      for (const auto & node : net.snapshot.nodes) {
        if (!node.reachable) continue;
        for (const auto & transport : node.transports) {
          if (transport.kind == "websocket_connect" && transport_actionable(transport, node)) {
            cloud_nodes.push_back(node.node_id);
            break;
          }
        }
      }

      // Every Ditto link in this simulator is TCP through the relay, so this
      // is the whole mesh. Both directions are collected together: `tcp_listen`
      // is always actionable, `tcp_connect` only once a node has peers, and a
      // node needs to be addressed if either applies.
      std::vector<std::string> tcp_nodes;
      bool any_tcp = false;
      for (const auto & node : net.snapshot.nodes) {
        if (!node.reachable) continue;
        bool addressable = false;
        for (const auto & transport : node.transports) {
          if (transport.kind != "tcp_connect" && transport.kind != "tcp_listen") continue;
          if (!transport_actionable(transport, node)) continue;
          addressable = true;
          any_tcp = any_tcp || transport.enabled;
        }
        if (addressable) tcp_nodes.push_back(node.node_id);
      }

      // Sized before anything is drawn, because the unconfigured case needs a
      // second line and would otherwise spill past the panel.
      const bool cloud_available = !cloud_nodes.empty();
      const float legend_height = (cloud_available ? 92.0F : 110.0F) + 30.0F +
        16.0F * static_cast<float>(std::max<std::size_t>(1, live_links_by_type.size()));
      const float legend_top = static_cast<float>(GetScreenHeight()) - 60.0F - legend_height;
      legend_panel = {24.0F, legend_top, 300.0F, legend_height};
      DrawRectangleRounded(legend_panel, 0.06F, 6, {30, 41, 59, 238});
      draw_text("NETWORK OBSERVER", 38, legend_top + 10.0F, 15, RAYWHITE);
      const char * state = net.connected ? "WATCHING" :
        (net.error.empty() ? "CONNECTING" : net.error.c_str());
      draw_text(state, 38, legend_top + 30.0F, 12,
        net.connected ? Color{101, 214, 159, 255} : Color{255, 183, 77, 255});

      float legend_row = legend_top + 48.0F;
      if (live_links_by_type.empty()) {
        draw_text("no links reported", 38, legend_row, 12, {148, 163, 184, 255});
        legend_row += 16.0F;
      }
      for (const auto & [kind, count] : live_links_by_type) {
        auto type = ditto::observer::PathType::unspecified;
        if (kind == "bluetooth") type = ditto::observer::PathType::bluetooth;
        else if (kind == "access_point") type = ditto::observer::PathType::access_point;
        else if (kind == "p2p_wifi") type = ditto::observer::PathType::p2p_wifi;
        else if (kind == "web_socket") type = ditto::observer::PathType::web_socket;
        else if (kind == "cloud") type = ditto::observer::PathType::cloud;
        DrawRectangle(38, static_cast<int>(legend_row) + 3, 10, 10, path_color(type));
        draw_text(TextFormat("%s  %u", kind.c_str(), count), 54, legend_row, 12,
          {223, 232, 242, 255});
        legend_row += 16.0F;
      }
      if (!net.snapshot.warnings.empty()) {
        draw_text(TextFormat("%d warning(s)", static_cast<int>(net.snapshot.warnings.size())),
          170, legend_top + 30.0F, 12, ORANGE);
      }

      // Fleet-wide cloud control. The cloud carried an entire twenty-node run
      // once while the mesh looked healthy, so being able to cut it from here
      // is the demonstration, not a convenience.
      const bool any_cloud = live_links_by_type.count("cloud") != 0;
      const float button_top = cloud_available ? legend_height - 62.0F : legend_height - 80.0F;
      const Rectangle fleet_button{38.0F, legend_top + button_top, 240.0F, 24.0F};
      if (!cloud_available) {
        // Nothing to offer: say why rather than presenting a control that
        // cannot work.
        DrawRectangleRounded(fleet_button, 0.25F, 4, {39, 44, 54, 255});
        draw_text("NO CLOUD CONFIGURED", fleet_button.x + 10.0F, fleet_button.y + 5.0F, 12,
          {88, 97, 112, 255});
        draw_text("start with SIM_ENABLE_CLOUD_SYNC=1", fleet_button.x + 10.0F,
          fleet_button.y + 26.0F, 11, {88, 97, 112, 255});
      } else {
        DrawRectangleRounded(fleet_button, 0.25F, 4,
          any_cloud ? Color{127, 29, 29, 255} : Color{22, 101, 52, 255});
        draw_text(
          any_cloud ? TextFormat("CUT CLOUD ON %d NODES", static_cast<int>(cloud_nodes.size())) :
            TextFormat("RESTORE CLOUD ON %d NODES", static_cast<int>(cloud_nodes.size())),
          fleet_button.x + 10.0F, fleet_button.y + 5.0F, 12, RAYWHITE);
        if (net.connected && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
          CheckCollisionPointRec(GetMousePosition(), fleet_button))
        {
          send_toggle(cloud_nodes, {{"websocket_connect", !any_cloud}});
        }
      }

      // Fleet-wide TCP control, beneath the cloud one. This is the harder cut
      // of the two: the cloud is one path that nobody configured a mesh for,
      // while TCP is the mesh, so cutting it should leave twenty nodes holding
      // nothing but their own stores.
      //
      // Both directions go in one call. `tcp_connect` alone is not isolation --
      // peers keep dialing in, which is what made a "cut" node carry on
      // replicating the first time this was measured.
      const Rectangle tcp_button{38.0F, legend_top + legend_height - 32.0F, 240.0F, 24.0F};
      if (tcp_nodes.empty()) {
        DrawRectangleRounded(tcp_button, 0.25F, 4, {39, 44, 54, 255});
        draw_text("NO TCP TRANSPORTS REPORTED", tcp_button.x + 10.0F, tcp_button.y + 5.0F, 12,
          {88, 97, 112, 255});
      } else {
        DrawRectangleRounded(tcp_button, 0.25F, 4,
          any_tcp ? Color{127, 29, 29, 255} : Color{22, 101, 52, 255});
        draw_text(
          any_tcp ? TextFormat("CUT TCP ON %d NODES", static_cast<int>(tcp_nodes.size())) :
            TextFormat("RESTORE TCP ON %d NODES", static_cast<int>(tcp_nodes.size())),
          tcp_button.x + 10.0F, tcp_button.y + 5.0F, 12, RAYWHITE);
        if (net.connected && IsMouseButtonPressed(MOUSE_BUTTON_LEFT) &&
          CheckCollisionPointRec(GetMousePosition(), tcp_button))
        {
          send_toggle(tcp_nodes, {{"tcp_connect", !any_tcp}, {"tcp_listen", !any_tcp}});
        }
      }
    }

    // Click a vehicle in the world to inspect it -- the same selection the
    // sidebar rows make, reached by pointing at the thing itself.
    //
    // Resolved here, at the end of the frame, because it has to know where the
    // legend panel ended up. That panel is drawn over the world, so a click on
    // CUT TCP would otherwise also select whatever drone happens to be behind
    // it.
    if (press_in_world && IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
      const Vector2 released_at = GetMousePosition();
      const bool dragged = Vector2Distance(press_at, released_at) > 4.0F;
      press_in_world = false;
      if (!dragged && !CheckCollisionPointRec(released_at, legend_panel)) {
        // The pick radius grows with camera distance. A drone a hundred metres
        // out is a few pixels across, and selecting it should not demand
        // precision the view cannot offer.
        const float pick_radius = std::max(0.9F, distance * 0.012F);
        const Ray ray = GetScreenToWorldRay(released_at, camera);
        const std::string * hit = nullptr;
        float nearest = std::numeric_limits<float>::max();
        for (const auto & [id, vehicle] : visible) {
          const Vector3 centre = sim::world::view_point(vehicle.east_m, -vehicle.down_m, vehicle.north_m);
          const RayCollision collision = GetRayCollisionSphere(ray, centre, pick_radius);
          if (collision.hit && collision.distance < nearest) {
            nearest = collision.distance;
            hit = &id;
          }
        }
        // Clicking bare ground clears the selection, which is how the inspector
        // gets closed without going back to the row that opened it.
        select_vehicle(hit != nullptr ? *hit : std::string());
      }
    }

    if (!snapshot.error.empty()) {
      draw_text(snapshot.error.c_str(), 24, static_cast<float>(GetScreenHeight() - 48), 14, MAROON);
    }
#ifndef DITTO_CESIUM_VIEWER
    if (!world.attribution.empty()) {
      draw_text(world.attribution.c_str(), 24, static_cast<float>(GetScreenHeight() - 24), 12,
        {30, 41, 59, 255});
    }
#endif
#ifdef DITTO_CESIUM_VIEWER
    if (cesium_tiles) {
      draw_text(cesium_tiles->status().c_str(), 24, 104, 13, {30, 41, 59, 255});
      if (!cesium_tiles->attribution().empty()) {
        draw_text(cesium_tiles->attribution().c_str(), 24,
          static_cast<float>(GetScreenHeight() - 24), 11, {30, 41, 59, 255});
      }
    }
#endif
    EndDrawing();

    // Capture once there is something to look at. This used to wait on the
    // observer specifically, which made it useless for the two things it is
    // most wanted for: checking a world file, and checking a fleet, neither of
    // which needs a mesh to be up. A network run still settles on the same
    // condition it always did, because net_ever_seen trips first.
#ifdef DITTO_CESIUM_VIEWER
    // Tiles stream in over seconds, so a Cesium capture waits for this view's.
    const bool scenery_settled = !cesium_tiles || cesium_tiles->idle();
#else
    const bool scenery_settled = true;
#endif
    if (capture_path != nullptr && !captured && (net_ever_seen || !visible.empty() || !world.empty()
#ifdef DITTO_CESIUM_VIEWER
      || cesium_tiles
#endif
      ) && scenery_settled &&
      ++settle_frames > 90)
    {
      TakeScreenshot(capture_path);
      captured = true;
      TraceLog(LOG_INFO, "wrote %s", capture_path);
    }
  }

  stopping.store(true);
  // A settled network is silent for as long as it stays settled, so the
  // watcher is parked in a blocking read and has to be cancelled rather than
  // waited out.
  {
    std::lock_guard<std::mutex> lock(stream_mutex);
    if (active_stream) active_stream->cancel();
  }
  if (observer_watcher.joinable()) observer_watcher.join();
  poller.join();
  if (reset_pid > 0) { kill(reset_pid, SIGTERM); waitpid(reset_pid, nullptr, 0); }
  isr_publisher.reset();
  feeds.reset();  // its GL resources need the window
  sim::world::unload(world);
  UnloadFont(ui_font);
  CloseWindow();
  return 0;
}
