#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <exception>
#include <fstream>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <utility>
#include <vector>

#include <raylib.h>
#include <raymath.h>

#include <ditto/observer/client.hpp>

#include <mavlink/common/mavlink.h>

#include <nlohmann/json.hpp>

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
  std::int64_t published_unix_ms{};
};

struct Snapshot {
  std::unordered_map<std::string, Vehicle> vehicles;
  std::string error;
  Clock::time_point received_at{};
};

struct Source {
  Source(std::string vehicle_id, const std::uint16_t port) : vehicle{std::move(vehicle_id)} {
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
  : vehicle(std::move(other.vehicle)), parser_buffer(other.parser_buffer), parser_status(other.parser_status),
    fd(std::exchange(other.fd, -1)) {}
  ~Source() { if (fd >= 0) close(fd); }

  Vehicle vehicle;
  mavlink_message_t parser_buffer{};
  mavlink_status_t parser_status{};
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

bool fresh(const NetworkStatus & status, const std::int64_t now)
{
  return status.observed_unix_ms > 0 && now >= status.observed_unix_ms &&
    now - status.observed_unix_ms <= 1500;
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
  return {ned.y, -ned.z, ned.x};
}

void draw_drone(const Vehicle & vehicle, const Vector3 center)
{
  const Color color = vehicle_color(vehicle.id);
  const Vector3 forward = Vector3Normalize(rotate_by_px4_quaternion(vehicle, {1.0F, 0.0F, 0.0F}));
  const Vector3 right = Vector3Normalize(rotate_by_px4_quaternion(vehicle, {0.0F, 1.0F, 0.0F}));
  const Vector3 up = Vector3Negate(
    Vector3Normalize(rotate_by_px4_quaternion(vehicle, {0.0F, 0.0F, 1.0F})));
  const Vector3 front = Vector3Add(center, Vector3Scale(forward, 0.55F));
  const Vector3 back = Vector3Add(center, Vector3Scale(forward, -0.42F));
  DrawCylinderEx(back, front, 0.16F, 0.10F, 10, color);
  DrawSphere(front, 0.13F, Fade(YELLOW, 0.9F));

  const Vector3 arm_a = Vector3Normalize(Vector3Add(forward, right));
  const Vector3 arm_b = Vector3Normalize(Vector3Subtract(forward, right));
  for (const Vector3 arm : {arm_a, arm_b}) {
    DrawCylinderEx(
      Vector3Add(center, Vector3Scale(arm, -0.72F)),
      Vector3Add(center, Vector3Scale(arm, 0.72F)), 0.035F, 0.035F, 6, DARKGRAY);
  }
  for (const Vector3 direction : {arm_a, Vector3Negate(arm_a), arm_b, Vector3Negate(arm_b)}) {
    const Vector3 rotor = Vector3Add(center, Vector3Scale(direction, 0.72F));
    DrawCylinderEx(
      Vector3Add(rotor, Vector3Scale(up, -0.025F)),
      Vector3Add(rotor, Vector3Scale(up, 0.025F)), 0.28F, 0.28F, 18, Fade(color, 0.72F));
  }
  DrawSphere(center, 0.19F, vehicle.failsafe ? RED : (vehicle.armed ? LIME : LIGHTGRAY));
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
    if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
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
      vehicle.published_unix_ms = unix_time_ms();
    } else if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
      mavlink_heartbeat_t state{};
      mavlink_msg_heartbeat_decode(&message, &state);
      vehicle.armed = (state.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
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
  if (kind == "awdl") return "AWDL";
  if (kind == "wifi_aware") return "WIFI-AW";
  if (kind == "multicast_beta") return "MCAST";
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
bool transport_actionable(const ditto::observer::TransportStatus & transport) {
  if (transport.kind == "tcp_connect" || transport.kind == "websocket_connect") {
    return !transport.endpoints.empty();
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
  try {
    for (int index = 1; index < argc; ++index) {
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
          "usage: %s [--network-metrics PATH] [--observer ADDR] --vehicle ID --port PX4_SIH_PORT [...]",
          argv[0]);
        return 2;
      }
      std::string vehicle_id(argv[index]);
      if (++index == argc || std::string(argv[index]) != "--port" || ++index == argc) {
        TraceLog(LOG_ERROR,
          "usage: %s [--network-metrics PATH] [--observer ADDR] --vehicle ID --port PX4_SIH_PORT [...]",
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

  // Applying a change to twenty-one nodes took about a third of a second, so
  // it runs off the render thread. Everything is copied in: the thread may
  // outlive the click that started it.
  const auto send_toggle = [&observer_endpoint](
    std::vector<std::string> node_ids, std::string kind, const bool enabled) {
      if (observer_endpoint.empty()) return;
      std::thread(
        [endpoint = observer_endpoint, node_ids = std::move(node_ids), kind = std::move(kind),
          enabled]() {
          ditto::observer::Client client(endpoint, std::chrono::seconds(30));
          std::vector<ditto::observer::TransportResult> results;
          const auto status = client.set_transports(node_ids, {{kind, enabled}}, results);
          if (!status.ok()) {
            TraceLog(LOG_WARNING, "transport change refused: %s", status.error_message().c_str());
            return;
          }
          for (const auto & result : results) {
            if (!result.applied) {
              TraceLog(LOG_WARNING, "%s: %s", result.node_id.c_str(), result.error.c_str());
            }
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
  Font ui_font = LoadFontEx("/System/Library/Fonts/SFNS.ttf", 48, nullptr, 0);
  SetTextureFilter(ui_font.texture, TEXTURE_FILTER_BILINEAR);
  const auto draw_text = [&ui_font](
    const char * text, const float x, const float y, const float size, const Color color) {
      DrawTextEx(ui_font, text, {x, y}, size, 0.5F, color);
    };

  const float map_size = sources.size() > 4 ? 240.0F : 100.0F;
  const float initial_distance = sources.size() > 4 ? 140.0F : 48.0F;
  float yaw = 0.78F;
  float pitch = 0.55F;
  float distance = initial_distance;
  Camera3D camera{};
  camera.target = {0.0F, 3.0F, 0.0F};
  camera.up = {0.0F, 1.0F, 0.0F};
  camera.fovy = 48.0F;
  camera.projection = CAMERA_PERSPECTIVE;
  std::unordered_map<std::string, Presentation> presentations;
  std::unordered_map<std::string, NetworkStatus> link_status;
  auto next_metrics_read = Clock::now();
  std::string selected_vehicle;
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

  while (!shutdown_requested && !WindowShouldClose()) {
    {
      std::lock_guard<std::mutex> lock(network_mutex);
      if (network.snapshot.observed_unix_ms != net.snapshot.observed_unix_ms ||
        network.connected != net.connected || network.error != net.error)
      {
        net = network;
        net_ever_seen = net_ever_seen || net.snapshot.observed_unix_ms != 0;
      }
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
    const float inspector_height = inspected != nullptr && !inspected->transports.empty() ?
      234.0F : 126.0F;
    const Rectangle inspector{24.0F, 106.0F, 330.0F, inspector_height};
    const bool pointer_on_inspector = !selected_vehicle.empty() &&
      CheckCollisionPointRec(GetMousePosition(), inspector);

    const float sidebar_width = std::clamp(GetScreenWidth() * 0.30F, 280.0F, 340.0F);
    const float sidebar_left = static_cast<float>(GetScreenWidth()) - sidebar_width;
    if (!pointer_on_inspector && GetMousePosition().x < sidebar_left &&
      IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
      const auto delta = GetMouseDelta();
      yaw -= delta.x * 0.006F;
      pitch = std::clamp(pitch + delta.y * 0.006F, 0.12F, 1.35F);
    }
    if (!pointer_on_inspector && GetMousePosition().x < sidebar_left &&
      IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
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
    distance = std::clamp(distance - GetMouseWheelMove() * 3.0F, 10.0F, map_size * 2.0F);
    if (IsKeyPressed(KEY_F)) {
      yaw = 0.78F;
      pitch = 0.55F;
      distance = initial_distance;
      camera.target = {0.0F, 3.0F, 0.0F};
    }
    camera.position = {
      camera.target.x + std::sin(yaw) * std::cos(pitch) * distance,
      camera.target.y + std::sin(pitch) * distance,
      camera.target.z + std::cos(yaw) * std::cos(pitch) * distance,
    };

    Snapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(snapshot_mutex);
      snapshot = latest;
    }
    const auto now = unix_time_ms();
    const auto render_time = Clock::now();
    if (render_time >= next_metrics_read) {
      link_status = read_network_metrics(network_metrics);
      next_metrics_read = render_time + std::chrono::milliseconds(250);
    }
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

    BeginDrawing();
    ClearBackground({238, 243, 247, 255});
    BeginScissorMode(0, 0, static_cast<int>(sidebar_left), GetScreenHeight());
    BeginMode3D(camera);
    DrawPlane({0.0F, -0.03F, 0.0F}, {map_size, map_size}, {229, 235, 240, 255});
    DrawGrid(static_cast<int>(map_size), 1.0F);
    for (const auto & [id, vehicle] : visible) {
      const Vector3 position{vehicle.east_m, -vehicle.down_m, vehicle.north_m};
      draw_drone(vehicle, position);
    }

    // The network the observer reports, drawn over the fleet.
    std::unordered_map<std::string, Vector3> node_positions;
    if (net_ever_seen) {
      for (const auto & [id, vehicle] : visible) {
        node_positions[id] = {vehicle.east_m, -vehicle.down_m, vehicle.north_m};
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
        const auto position = node_positions.find(node.node_id);
        if (position == node_positions.end()) continue;
        const Vector3 top = Vector3Add(position->second, {0.0F, 6.0F, 0.0F});
        DrawLine3D(position->second, top, path_color(ditto::observer::PathType::cloud));
        DrawSphere(top, 0.45F, path_color(ditto::observer::PathType::cloud));
      }
    }
    EndMode3D();
    EndScissorMode();

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
            [](const ditto::observer::TransportStatus & transport) {
              return !transport_actionable(transport);
            });
          const char * chip_hint = !inspected->reachable ? "TRANSPORTS  (node unreachable)" :
            any_unconfigured ? "TRANSPORTS  (dimmed: nothing configured to dial)" :
            "TRANSPORTS  (click to toggle)";
          draw_text(chip_hint, 38, 230, 12,
            inspected->reachable ? Color{174, 193, 212, 255} : Color{255, 183, 77, 255});
          float chip_x = 38.0F;
          float chip_y = 248.0F;
          for (const auto & transport : inspected->transports) {
            const Rectangle chip{chip_x, chip_y, 98.0F, 24.0F};
            const bool actionable = transport_actionable(transport);
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
              send_toggle({inspected->node_id}, transport.kind, !transport.enabled);
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
      (GetScreenHeight() - 94.0F) / std::max(1.0F, static_cast<float>(vehicle_ids.size())),
      21.0F, 27.0F);
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
        selected_vehicle = id;
      }
      row += row_height;
    }
    // What the observer says the network is, and the one control that makes
    // the point: the cloud is a path nobody configured a mesh for.
    if (!observer_endpoint.empty()) {
      // Only nodes that were started with a cloud URL can be told to use one,
      // so they are the only ones the fleet button addresses. Sending to the
      // whole fleet would earn a FailedPrecondition from every node with none.
      std::vector<std::string> cloud_nodes;
      for (const auto & node : net.snapshot.nodes) {
        if (!node.reachable) continue;
        for (const auto & transport : node.transports) {
          if (transport.kind == "websocket_connect" && transport_actionable(transport)) {
            cloud_nodes.push_back(node.node_id);
            break;
          }
        }
      }

      // Sized before anything is drawn, because the unconfigured case needs a
      // second line and would otherwise spill past the panel.
      const bool cloud_available = !cloud_nodes.empty();
      const float legend_height = (cloud_available ? 92.0F : 110.0F) +
        16.0F * static_cast<float>(std::max<std::size_t>(1, net.snapshot.links_by_type.size()));
      const float legend_top = static_cast<float>(GetScreenHeight()) - 60.0F - legend_height;
      DrawRectangleRounded({24.0F, legend_top, 300.0F, legend_height}, 0.06F, 6,
        {30, 41, 59, 238});
      draw_text("NETWORK OBSERVER", 38, legend_top + 10.0F, 15, RAYWHITE);
      const char * state = net.connected ? "WATCHING" :
        (net.error.empty() ? "CONNECTING" : net.error.c_str());
      draw_text(state, 38, legend_top + 30.0F, 12,
        net.connected ? Color{101, 214, 159, 255} : Color{255, 183, 77, 255});

      float legend_row = legend_top + 48.0F;
      if (net.snapshot.links_by_type.empty()) {
        draw_text("no links reported", 38, legend_row, 12, {148, 163, 184, 255});
        legend_row += 16.0F;
      }
      for (const auto & [kind, count] : net.snapshot.links_by_type) {
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
      const bool any_cloud = net.snapshot.links_by_type.count("cloud") != 0;
      const float button_top = cloud_available ? legend_height - 32.0F : legend_height - 50.0F;
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
          send_toggle(cloud_nodes, "websocket_connect", !any_cloud);
        }
      }
    }

    if (!snapshot.error.empty()) {
      draw_text(snapshot.error.c_str(), 24, static_cast<float>(GetScreenHeight() - 48), 14, MAROON);
    }
    EndDrawing();

    if (capture_path != nullptr && !captured && net_ever_seen && ++settle_frames > 90) {
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
  UnloadFont(ui_font);
  CloseWindow();
  return 0;
}
