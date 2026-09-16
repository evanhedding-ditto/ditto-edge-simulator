// Synthetic vehicle fleet: N kinematic vehicles in one process.
//
// Purpose is scale, not fidelity. A 20-node PX4 run is 72 processes -- 20 SIH
// autopilots at ~20 threads each, 10 XRCE agents, 10 ROS 2 bridges, 10 MAVLink
// bridges -- which thermally throttles a fanless M4 within eight minutes. None
// of that exercises the Edge Server, which is autopilot-agnostic: it only ever
// sees vehicle_state documents and fleet_commands over EdgeStore. This replaces
// the autopilot tier with an integrator so the Edge Server can be tested at 100+
// nodes. `mvp-four-mixed` on real PX4 stays the fidelity and adapter gate.
//
// Three things are deliberately reused verbatim from the ROS 2 adapter rather
// than reimplemented, so a synthetic vehicle is indistinguishable downstream:
//   documents.cpp  the Ditto schema and DQL, so documents are byte-identical
//   mission.cpp    the guidance, so synthetic vehicles fly the same paths
//   edge_link.cpp  the Edge Server I/O, so the transport is the same code
// Only the autopilot is replaced: PX4's estimator and controllers become a
// velocity- and acceleration-limited kinematic integrator.
//
// The viewer does NOT read Ditto -- it parses MAVLink UDP straight from each
// autopilot (viewer/src/main.cpp) -- so writing vehicle_state is not enough.
// GLOBAL_POSITION_INT, ATTITUDE and HEARTBEAT are emitted on 19410+index, which
// is also what ditto_px4_telemetry_ready gates on, so wait-telemetry.sh works
// against a synthetic fleet unchanged.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <mavlink/common/mavlink.h>

#include "px4_ditto_bridge/documents.hpp"
#include "px4_ditto_bridge/edge_link.hpp"
#include "px4_ditto_bridge/mission.hpp"

namespace
{

using px4_ditto_bridge::Command;
using px4_ditto_bridge::EdgeLink;
using px4_ditto_bridge::Kind;
using px4_ditto_bridge::Mission;
using px4_ditto_bridge::Setpoint;
using px4_ditto_bridge::Snapshot;
using px4_ditto_bridge::unix_ms;

using Clock = std::chrono::steady_clock;

/// Flight envelope. These bound the integrator, not a controller: guidance in
/// mission.cpp already commands at most kCruiseMps (8) horizontally and
/// kClimbMps (3) vertically, so the horizontal cap only ever binds during the
/// velocity feed-forward of a tight orbit, and the acceleration cap is what
/// stops a vehicle teleporting when a new command arrives.
constexpr float kMaxSpeedMps = 12.0F;
constexpr float kMaxClimbMps = 3.0F;
constexpr float kMaxAccelMps2 = 6.0F;
constexpr float kMaxYawRateRadS = 1.5F;
/// Proportional pull toward the position setpoint, on top of the feed-forward.
/// Without it the vehicle tracks velocity but never closes a position error.
constexpr float kPositionGain = 1.2F;

/// Integration and command-machine tick. PX4's SIH runs at 250 Hz and the ROS 2
/// adapter's control loop at 10 Hz; 50 Hz is well inside mission.cpp's
/// kMaxStepS clamp and keeps viewer motion smooth at 100+ vehicles.
constexpr auto kControlPeriod = std::chrono::milliseconds(20);

/// MAVLink stream rates. PX4's onboard link streams position and attitude at
/// roughly this rate, and the viewer interpolates nothing, so it is what makes
/// motion look smooth.
constexpr auto kTelemetryPeriod = std::chrono::milliseconds(100);
constexpr auto kHeartbeatPeriod = std::chrono::seconds(1);

/// Command lifecycle, identical to bridge.cpp so receipts look the same.
constexpr auto kArmTimeout = std::chrono::seconds(5);
constexpr auto kOffboardTimeout = std::chrono::seconds(15);
constexpr auto kOffboardWarmup = std::chrono::seconds(1);
constexpr auto kTelemetryTimeout = std::chrono::seconds(5);
constexpr std::size_t kQueueLimit = 8;

/// Modelled actuation latency between asking for arm/offboard and the mode
/// taking effect. A real autopilot is never instant, and without some delay the
/// dispatched -> accepted path would complete on the same tick it started and
/// stop exercising the timeout machinery at all.
constexpr auto kActuationLatency = std::chrono::milliseconds(200);

/// Must match viewer/src/main.cpp, which converts the other way. Any mismatch
/// shows up as vehicles drawn at the wrong offset from the origin.
constexpr double kMetresPerDegree = 111319.49079327357;
constexpr double kDegreesToRadians = 0.017453292519943295;

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTau = 2.0F * kPi;

std::atomic<bool> g_stopping{false};

void handle_signal(int) {g_stopping.store(true);}

float wrap(const float angle) {return std::remainder(angle, kTau);}

struct Origin
{
  double latitude_deg{36.01883233670948};
  double longitude_deg{-78.9684198511774};
  double altitude_m{0.0};
};

/// One command in flight, ended by the vehicle agreeing or by the deadline.
struct Pending
{
  Command command;
  Clock::time_point deadline;
  Clock::time_point request_at;
};

/// UDP sender aimed at one viewer port. PX4 binds 19450+index and streams to
/// 19410+index; only the streaming half is reproduced here, because nothing
/// sends MAVLink *to* a synthetic vehicle -- commands arrive over Ditto.
class MavlinkStream final
{
public:
  MavlinkStream(const std::uint16_t port, const std::uint8_t system_id)
  : system_id_(system_id)
  {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("could not create MAVLink socket");
    }
    target_.sin_family = AF_INET;
    target_.sin_port = htons(port);
    target_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  }

  MavlinkStream(const MavlinkStream &) = delete;
  MavlinkStream & operator=(const MavlinkStream &) = delete;
  ~MavlinkStream() {if (fd_ >= 0) {close(fd_);}}

  void heartbeat(const bool armed)
  {
    mavlink_message_t message{};
    const std::uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
      (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);
    mavlink_msg_heartbeat_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
      base_mode, 0, armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    send(message);
  }

  void position(
    const std::uint32_t boot_ms, const double latitude_deg, const double longitude_deg,
    const double altitude_m, const double relative_altitude_m, const float north_velocity_m_s,
    const float east_velocity_m_s, const float down_velocity_m_s, const float heading_rad)
  {
    mavlink_message_t message{};
    // Heading is centi-degrees in [0, 35999]; MAVLink has no negative heading.
    auto heading_cdeg = static_cast<std::int32_t>(std::lround(wrap(heading_rad) * 18000.0F / kPi));
    heading_cdeg = ((heading_cdeg % 36000) + 36000) % 36000;
    mavlink_msg_global_position_int_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms,
      static_cast<std::int32_t>(std::llround(latitude_deg * 1e7)),
      static_cast<std::int32_t>(std::llround(longitude_deg * 1e7)),
      static_cast<std::int32_t>(std::llround(altitude_m * 1000.0)),
      static_cast<std::int32_t>(std::llround(relative_altitude_m * 1000.0)),
      static_cast<std::int16_t>(std::lround(north_velocity_m_s * 100.0F)),
      static_cast<std::int16_t>(std::lround(east_velocity_m_s * 100.0F)),
      static_cast<std::int16_t>(std::lround(down_velocity_m_s * 100.0F)),
      static_cast<std::uint16_t>(heading_cdeg));
    send(message);
  }

  void attitude(const std::uint32_t boot_ms, const float yaw_rad, const float yaw_rate_rad_s)
  {
    mavlink_message_t message{};
    // A kinematic point mass has no attitude dynamics; roll and pitch stay zero
    // and only yaw is real. The viewer builds a quaternion from all three.
    mavlink_msg_attitude_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms, 0.0F, 0.0F, wrap(yaw_rad), 0.0F,
      0.0F, yaw_rate_rad_s);
    send(message);
  }

private:
  void send(const mavlink_message_t & message)
  {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    // Loopback UDP to a socket nobody is required to be reading: a viewer that
    // is not running is normal, so a failed send is never fatal.
    (void)sendto(
      fd_, buffer.data(), length, 0, reinterpret_cast<const sockaddr *>(&target_),
      sizeof(target_));
  }

  int fd_{-1};
  std::uint8_t system_id_;
  sockaddr_in target_{};
};

/// One synthetic vehicle: integrator, command state machine, and both outputs.
///
/// Every member is touched only by the single control thread except the command
/// queue, which EdgeLink's observer thread pushes into -- the same split the
/// ROS 2 adapter has, where rclcpp::spin owns everything but the queue.
class Vehicle final
{
public:
  Vehicle(
    const std::size_t index, const std::string & socket_path, const std::string & api_key,
    const Origin & origin, const std::uint16_t mavlink_port)
  : id_("px4_" + std::to_string(index)),
    origin_(origin),
    // PX4 SITL sets MAV_SYS_ID to the instance index plus one; the viewer keys
    // on the port rather than the ID, but ATAK and MAVLink tooling do not.
    stream_(mavlink_port, static_cast<std::uint8_t>(index + 1))
  {
    if (index > 254) {
      throw std::invalid_argument("vehicle index must be in [0, 254]");
    }
    // Every vehicle spawns coincident at the geodetic origin. This replaces the
    // former 25 m / 10-per-row grid, whose purpose was to keep vehicles visually
    // distinct before launch: with a shared spawn they overlap in the viewer
    // until a command separates them, which is expected rather than a fault.
    north_m_ = 0.0F;
    east_m_ = 0.0F;
    down_m_ = 0.0F;

    state_.local_valid = true;
    state_.global_valid = true;
    refresh_snapshot();

    edge_ = std::make_unique<EdgeLink>(
      socket_path, api_key, id_, [this](Command command) {enqueue(std::move(command));},
      [this](const std::string & message) {
        std::clog << "[" << id_ << "] " << message << "\n";
      });
  }

  Vehicle(const Vehicle &) = delete;
  Vehicle & operator=(const Vehicle &) = delete;

  /// Stop Edge I/O before anything its callback touches is destroyed.
  ~Vehicle() {edge_.reset();}

  void tick(const Clock::time_point now, const float dt)
  {
    control(now, dt);
    integrate(dt);
    refresh_snapshot();
    publish(now);
    emit(now);
  }

private:
  // --- command state machine, mirroring bridge.cpp ---

  void enqueue(Command command)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= kQueueLimit) {
      reject(command, "rejected", "connector_queue_full");
      return;
    }
    queue_.push_back(std::move(command));
  }

  std::optional<Command> dequeue()
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.empty()) {
      return std::nullopt;
    }
    auto command = std::move(queue_.front());
    queue_.pop_front();
    return command;
  }

  bool done(const Pending & pending) const
  {
    return pending.command.kind == Kind::set_armed ?
           state_.armed == pending.command.armed : state_.armed && state_.offboard;
  }

  static const char * timeout_reason(const Pending & pending)
  {
    return pending.command.kind == Kind::set_armed ? "px4_arm_timeout" : "px4_offboard_timeout";
  }

  void reject(const Command & command, const char * status, const char * error)
  {
    std::clog << "[" << id_ << "] command " << command.id << " " << status << ": " << error
              << "\n";
    edge_->set_status(command, status, error);
  }

  void control(const Clock::time_point now, const float dt)
  {
    // The modelled actuation latency landing is what makes done() true.
    if (actuation_at_ && now >= *actuation_at_) {
      armed_ = requested_armed_;
      offboard_ = requested_offboard_;
      actuation_at_.reset();
    }

    if (pending_) {
      if (done(*pending_)) {
        edge_->set_status(pending_->command, "accepted");
        pending_.reset();
      } else if (now >= pending_->deadline) {
        const auto * reason = timeout_reason(*pending_);
        std::clog << "[" << id_ << "] command " << pending_->command.id << " failed: " << reason
                  << "\n";
        edge_->set_status(pending_->command, "failed", reason);
        if (pending_->command.kind != Kind::set_armed) {
          mission_.reset();
        }
        pending_.reset();
      }
    }

    if (!pending_) {
      if (auto command = dequeue()) {
        start(std::move(*command), now);
      }
    }

    if (mission_) {
      setpoint_ = mission_->setpoint(state_, dt);
      has_setpoint_ = true;
    }
    if (pending_ && now >= pending_->request_at) {
      dispatch(pending_->command, now);
      pending_->request_at = now + kOffboardWarmup;
    }
  }

  void start(Command command, const Clock::time_point now)
  {
    if (command.expires_unix_ms <= unix_ms()) {
      reject(command, "expired", "command_expired");
      return;
    }
    // Kept even though a synthetic vehicle's telemetry cannot go stale: this is
    // the gate that distinguishes a live autopilot from a frozen one, and
    // removing it here would make the synthetic path structurally different
    // from the real one in exactly the place a regression would hide.
    if (!state_.updated || now - *state_.updated >= kTelemetryTimeout) {
      reject(command, "failed", "autopilot_unavailable");
      return;
    }
    if (command.kind != Kind::set_armed && !state_.local_valid) {
      reject(command, "failed", "local_position_unavailable");
      return;
    }
    Pending pending{std::move(command), now + kOffboardTimeout, now + kOffboardWarmup};
    if (pending.command.kind == Kind::set_armed) {
      if (!pending.command.armed) {
        mission_.reset();
        has_setpoint_ = false;
      }
      pending.deadline = now + kArmTimeout;
      pending.request_at = now;
    } else {
      mission_ = Mission::create(pending.command);
    }
    edge_->set_status(pending.command, "dispatched");
    pending_ = std::move(pending);
  }

  /// What PX4 would do on receiving the VehicleCommand the adapter sends.
  void dispatch(const Command & command, const Clock::time_point now)
  {
    if (command.kind == Kind::set_armed) {
      requested_armed_ = command.armed;
      requested_offboard_ = command.armed && offboard_;
    } else {
      requested_armed_ = true;
      requested_offboard_ = true;
    }
    if (!actuation_at_) {
      actuation_at_ = now + kActuationLatency;
    }
  }

  // --- integrator: the replacement for PX4's estimator and controllers ---

  void integrate(const float dt)
  {
    if (dt <= 0.0F) {
      return;
    }
    float target_north_velocity = 0.0F;
    float target_east_velocity = 0.0F;
    float target_down_velocity = 0.0F;
    float target_yaw = yaw_rad_;

    if (armed_ && offboard_ && has_setpoint_) {
      // Velocity feed-forward from guidance plus a proportional pull on the
      // remaining position error, which is what closes the last few metres.
      target_north_velocity = setpoint_.north_velocity_m_s +
        (setpoint_.north_m - north_m_) * kPositionGain;
      target_east_velocity = setpoint_.east_velocity_m_s +
        (setpoint_.east_m - east_m_) * kPositionGain;
      target_down_velocity = setpoint_.down_velocity_m_s +
        (setpoint_.down_m - down_m_) * kPositionGain;
      target_yaw = setpoint_.yaw_rad;

      const float speed = std::hypot(target_north_velocity, target_east_velocity);
      if (speed > kMaxSpeedMps) {
        const float scale = kMaxSpeedMps / speed;
        target_north_velocity *= scale;
        target_east_velocity *= scale;
      }
      target_down_velocity = std::clamp(target_down_velocity, -kMaxClimbMps, kMaxClimbMps);
    }
    // Disarmed or without a mission the vehicle sheds velocity rather than
    // stopping dead, so a disarm mid-flight looks like a decelerating vehicle.

    approach_velocity(north_velocity_m_s_, target_north_velocity, dt);
    approach_velocity(east_velocity_m_s_, target_east_velocity, dt);
    approach_velocity(down_velocity_m_s_, target_down_velocity, dt);

    north_m_ += north_velocity_m_s_ * dt;
    east_m_ += east_velocity_m_s_ * dt;
    down_m_ += down_velocity_m_s_ * dt;
    // Never integrate through the ground.
    if (down_m_ > 0.0F) {
      down_m_ = 0.0F;
      if (down_velocity_m_s_ > 0.0F) {
        down_velocity_m_s_ = 0.0F;
      }
    }

    const float yaw_error = wrap(target_yaw - yaw_rad_);
    const float yaw_step = std::clamp(yaw_error, -kMaxYawRateRadS * dt, kMaxYawRateRadS * dt);
    yaw_rate_rad_s_ = dt > 0.0F ? yaw_step / dt : 0.0F;
    yaw_rad_ = wrap(yaw_rad_ + yaw_step);
  }

  static void approach_velocity(float & velocity, const float target, const float dt)
  {
    const float limit = kMaxAccelMps2 * dt;
    velocity += std::clamp(target - velocity, -limit, limit);
  }

  // --- outputs ---

  void refresh_snapshot()
  {
    state_.armed = armed_;
    state_.offboard = offboard_;
    state_.north_m = north_m_;
    state_.east_m = east_m_;
    state_.down_m = down_m_;
    state_.north_velocity_m_s = north_velocity_m_s_;
    state_.east_velocity_m_s = east_velocity_m_s_;
    state_.down_velocity_m_s = down_velocity_m_s_;
    state_.heading_rad = yaw_rad_;
    state_.latitude_deg = origin_.latitude_deg + north_m_ / kMetresPerDegree;
    state_.longitude_deg = origin_.longitude_deg +
      east_m_ / (kMetresPerDegree * std::cos(origin_.latitude_deg * kDegreesToRadians));
    state_.altitude_m = static_cast<float>(origin_.altitude_m) - down_m_;
    ++state_.revision;
    // The staleness gate both adapters use reads this; a synthetic vehicle is
    // always live, so it is stamped every tick.
    state_.updated = Clock::now();
  }

  void publish(const Clock::time_point now)
  {
    if (now < next_state_) {
      return;
    }
    next_state_ = now + state_period_;
    edge_->publish(state_);
  }

  void emit(const Clock::time_point now)
  {
    const auto boot_ms = static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - started_).count());
    if (now >= next_heartbeat_) {
      next_heartbeat_ = now + kHeartbeatPeriod;
      stream_.heartbeat(armed_);
    }
    if (now < next_telemetry_) {
      return;
    }
    next_telemetry_ = now + kTelemetryPeriod;
    stream_.position(
      boot_ms, state_.latitude_deg, state_.longitude_deg, state_.altitude_m, -down_m_,
      north_velocity_m_s_, east_velocity_m_s_, down_velocity_m_s_, yaw_rad_);
    stream_.attitude(boot_ms, yaw_rad_, yaw_rate_rad_s_);
  }

public:
  /// Spread this vehicle's outbound work across its own period.
  ///
  /// Every vehicle ticks on one thread, so without a per-vehicle phase offset
  /// all N publish in the same millisecond and then go quiet for the rest of the
  /// period. That thundering herd is an artefact of the single control thread:
  /// real PX4 instances each keep their own clock and drift apart, so the Edge
  /// Servers see an evenly spread load. Measured at 20 vehicles, the
  /// synchronised burst produced 118 `Deadline Exceeded` retries during startup.
  void set_schedule(const double hz, const std::size_t index, const std::size_t fleet_size)
  {
    state_period_ = std::chrono::milliseconds(static_cast<int>(std::lround(1000.0 / hz)));
    const auto now = Clock::now();
    const auto share = [index, fleet_size](const std::chrono::milliseconds period) {
        return fleet_size == 0 ?
               std::chrono::milliseconds(0) :
               std::chrono::milliseconds(
          static_cast<int>(period.count() * static_cast<long>(index) /
          static_cast<long>(fleet_size)));
      };
    next_state_ = now + share(state_period_);
    next_telemetry_ = now + share(kTelemetryPeriod);
    next_heartbeat_ = now + share(kHeartbeatPeriod);
  }

private:
  std::string id_;
  Origin origin_;
  MavlinkStream stream_;

  float north_m_{0.0F};
  float east_m_{0.0F};
  float down_m_{0.0F};
  float north_velocity_m_s_{0.0F};
  float east_velocity_m_s_{0.0F};
  float down_velocity_m_s_{0.0F};
  float yaw_rad_{0.0F};
  float yaw_rate_rad_s_{0.0F};

  bool armed_{false};
  bool offboard_{false};
  bool requested_armed_{false};
  bool requested_offboard_{false};
  std::optional<Clock::time_point> actuation_at_{};

  Snapshot state_{};
  Setpoint setpoint_{};
  bool has_setpoint_{false};
  std::optional<Mission> mission_{};
  std::optional<Pending> pending_{};

  std::mutex queue_mutex_;
  std::deque<Command> queue_;

  std::unique_ptr<EdgeLink> edge_;

  Clock::time_point started_{Clock::now()};
  Clock::time_point next_state_{};
  Clock::time_point next_telemetry_{};
  Clock::time_point next_heartbeat_{};
  std::chrono::milliseconds state_period_{100};
};

[[noreturn]] void usage(const char * program, const char * error = nullptr)
{
  if (error != nullptr) {
    std::cerr << "error: " << error << "\n";
  }
  std::cerr
    << "usage: " << program << " --runtime-dir DIR --count N [options]\n"
    << "  --runtime-dir DIR      simulator runtime directory; vehicle sockets are\n"
    << "                         DIR/nodes/px4_<index>/edge.sock\n"
    << "  --count N              number of synthetic vehicles (1-255)\n"
    << "  --origin-lat DEG       geodetic origin latitude  (default 36.01883233670948)\n"
    << "  --origin-lon DEG       geodetic origin longitude (default -78.9684198511774)\n"
    << "  --origin-alt M         geodetic origin altitude  (default 0)\n"
    << "  --mavlink-port-base P  first viewer port (default 19410)\n"
    << "  --state-rate-hz R      vehicle_state publish rate (default 10)\n"
    << "  --api-key KEY          Edge Server API key (default none)\n";
  std::exit(2);
}

double number(const char * text, const char * name)
{
  try {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != std::string_view(text).size()) {
      throw std::invalid_argument("trailing characters");
    }
    return value;
  } catch (const std::exception &) {
    std::cerr << "error: " << name << " must be a number\n";
    std::exit(2);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string runtime_dir;
  std::string api_key;
  long count = 0;
  Origin origin;
  long port_base = 19410;
  double state_rate_hz = 10.0;

  for (int index = 1; index < argc; ++index) {
    const std::string_view flag(argv[index]);
    const auto value = [&]() -> const char * {
        if (index + 1 >= argc) {
          usage(argv[0], "missing value");
        }
        return argv[++index];
      };
    if (flag == "--runtime-dir") {
      runtime_dir = value();
    } else if (flag == "--count") {
      count = std::lround(number(value(), "--count"));
    } else if (flag == "--origin-lat") {
      origin.latitude_deg = number(value(), "--origin-lat");
    } else if (flag == "--origin-lon") {
      origin.longitude_deg = number(value(), "--origin-lon");
    } else if (flag == "--origin-alt") {
      origin.altitude_m = number(value(), "--origin-alt");
    } else if (flag == "--mavlink-port-base") {
      port_base = std::lround(number(value(), "--mavlink-port-base"));
    } else if (flag == "--state-rate-hz") {
      state_rate_hz = number(value(), "--state-rate-hz");
    } else if (flag == "--api-key") {
      api_key = value();
    } else {
      usage(argv[0], "unknown option");
    }
  }
  if (runtime_dir.empty()) {
    usage(argv[0], "--runtime-dir is required");
  }
  if (count < 1 || count > 255) {
    usage(argv[0], "--count must be in [1, 255]");
  }
  if (state_rate_hz <= 0.0 || state_rate_hz > 100.0) {
    usage(argv[0], "--state-rate-hz must be in (0, 100]");
  }
  if (port_base < 1 || port_base + count > 65535) {
    usage(argv[0], "--mavlink-port-base and --count must stay inside the port range");
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::vector<std::unique_ptr<Vehicle>> fleet;
  fleet.reserve(static_cast<std::size_t>(count));
  try {
    for (long index = 0; index < count; ++index) {
      const std::string socket_path =
        runtime_dir + "/nodes/px4_" + std::to_string(index) + "/edge.sock";
      auto vehicle = std::make_unique<Vehicle>(
        static_cast<std::size_t>(index), socket_path, api_key, origin,
        static_cast<std::uint16_t>(port_base + index));
      vehicle->set_schedule(state_rate_hz, static_cast<std::size_t>(index), static_cast<std::size_t>(count));
      fleet.push_back(std::move(vehicle));
    }
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }

  std::cout << "synthetic fleet: " << count << " vehicles, state " << state_rate_hz
            << " Hz, MAVLink " << port_base << "-" << (port_base + count - 1) << "\n"
            << std::flush;

  // One thread drives every vehicle. The per-vehicle work is a handful of
  // floating-point operations and two datagrams, so 100+ vehicles at 50 Hz is a
  // fraction of a core -- which is the entire point of this tool. EdgeLink
  // still owns two threads per vehicle for its own I/O.
  auto previous = Clock::now();
  auto next = previous;
  while (!g_stopping.load()) {
    next += kControlPeriod;
    const auto now = Clock::now();
    const float dt = std::chrono::duration<float>(now - previous).count();
    previous = now;
    for (auto & vehicle : fleet) {
      vehicle->tick(now, dt);
    }
    const auto sleep_until = next;
    if (sleep_until > Clock::now()) {
      std::this_thread::sleep_until(sleep_until);
    } else {
      // Fell behind: resynchronise rather than accumulating a backlog of ticks
      // that would each integrate a stale dt.
      next = Clock::now();
    }
  }

  std::cout << "synthetic fleet stopping\n" << std::flush;
  fleet.clear();
  return 0;
}
