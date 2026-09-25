// Synthetic vehicle fleet: N emulated PX4 autopilots in one process.
//
// Purpose is scale, not fidelity. A 20-node PX4 run is 72 processes -- 20 SIH
// autopilots at ~20 threads each, 10 XRCE agents, 10 ROS 2 bridges, 10 MAVLink
// bridges -- which thermally throttles a fanless M4 within eight minutes. What
// that cost buys is the autopilot itself: the estimator, the controllers, and
// rcS. None of it exercises Edge Server. So the autopilot is what this replaces,
// with a velocity- and acceleration-limited kinematic integrator, and nothing
// else is replaced along with it.
//
// In particular the adapter is NOT replaced. Each vehicle speaks MAVLink to a
// real `px4-mavlink-ditto-bridge` process, exactly as PX4 does, and that adapter
// does every Ditto write. This process owns no Edge Server connection and no
// command state machine; it is an autopilot, and only an autopilot. The earlier
// version of this file linked the adapter's documents/mission/edge_link and
// re-implemented its command machine, which meant the synthetic tier tested
// neither adapter as a process nor the autopilot transport.
//
// `mvp-four-mixed` on real PX4 stays the fidelity gate for PX4 itself.
//
// Two links per vehicle, mirroring PX4's two `mavlink` instances:
//
//   display  ephemeral -> 19410+i   what the viewer and the telemetry gate read
//   control  25540+i   -> 24540+i   what the adapter talks to, and the only one
//                                   that is read from
//
// The display link exists because the viewer does NOT read Ditto -- it parses
// MAVLink UDP straight from each autopilot (viewer/src/main.cpp) -- and because
// ditto_px4_telemetry_ready gates on the same port.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <csignal>
#include <iostream>
#include <memory>
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

namespace
{

using Clock = std::chrono::steady_clock;

/// Flight envelope. These bound the integrator, not a controller: the adapter's
/// guidance already commands at most 8 m/s horizontally and 3 m/s vertically, so
/// the horizontal cap only ever binds during the velocity feed-forward of a
/// tight orbit, and the acceleration cap is what stops a vehicle teleporting
/// when a new command arrives.
constexpr float kMaxSpeedMps = 12.0F;
constexpr float kMaxClimbMps = 3.0F;
constexpr float kMaxAccelMps2 = 6.0F;
constexpr float kMaxYawRateRadS = 1.5F;
/// Proportional pull toward the position setpoint, on top of the feed-forward.
/// Without it the vehicle tracks velocity but never closes a position error.
constexpr float kPositionGain = 1.2F;

/// Integration tick. PX4's SIH runs at 250 Hz and the adapter's control loop at
/// 10 Hz; 50 Hz keeps viewer motion smooth at 100+ vehicles.
constexpr auto kControlPeriod = std::chrono::milliseconds(20);

/// MAVLink stream rates. PX4's `onboard` profile is faster than this, but the
/// adapter reads position at 10 Hz and the viewer interpolates nothing, so this
/// is what makes motion look smooth without flooding loopback at 100 vehicles.
constexpr auto kTelemetryPeriod = std::chrono::milliseconds(100);
constexpr auto kHeartbeatPeriod = std::chrono::seconds(1);

/// Modelled actuation latency between a request arriving and the mode taking
/// effect. A real autopilot is never instant, and without some delay the
/// dispatched -> accepted path would complete on the same tick it started and
/// stop exercising the adapter's timeout machinery at all.
constexpr auto kActuationLatency = std::chrono::milliseconds(200);

/// How long a setpoint stays current. PX4 refuses to enter offboard without a
/// live setpoint stream and leaves offboard when one stops, and this is the
/// window it uses. Emulating that refusal is the point of the control link: the
/// adapter's one-second warm-up and its retry loop exist because of it, and
/// accepting the mode switch instantly would leave both untested.
constexpr auto kOffboardSignalTimeout = std::chrono::milliseconds(500);

/// PX4 packs its main mode into bits 16-23 of `custom_mode`.
constexpr std::uint32_t kMainModeOffboard = 6;
constexpr std::uint32_t kMainModePosctl = 3;

/// Command IDs from the MAVLink common specification. PX4's build trims the
/// `MAV_CMD` enum away, so the wire values are written out.
constexpr std::uint16_t kCommandSetMode = 176;
constexpr std::uint16_t kCommandArmDisarm = 400;

/// Setpoint type-mask bits, by group. Honouring the mask by group rather than
/// matching the adapter's exact value keeps this honest if the adapter ever
/// commands a different combination.
constexpr std::uint16_t kIgnorePosition = 0b0000'0000'0000'0111;
constexpr std::uint16_t kIgnoreVelocity = 0b0000'0000'0011'1000;
constexpr std::uint16_t kIgnoreYaw = 0b0000'0100'0000'0000;

/// Datagrams handled per vehicle per tick, so one noisy link cannot starve the
/// rest of the fleet.
constexpr int kDrainLimit = 16;

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

/// What the autopilot is being told to fly.
struct Setpoint
{
  float north_m{0.0F};
  float east_m{0.0F};
  float down_m{0.0F};
  float north_velocity_m_s{0.0F};
  float east_velocity_m_s{0.0F};
  float down_velocity_m_s{0.0F};
  float yaw_rad{0.0F};
  bool position{false};
  bool velocity{false};
  bool yaw{false};
};

/// One MAVLink link. Optionally bound, in which case it also reads.
///
/// A bound link sends from the socket it listens on, which is not incidental:
/// the adapter dials in with `udpin`, so it learns where to reply from the
/// source address of whatever arrived most recently. Sending from a second
/// socket would send its replies somewhere else entirely.
class Link final
{
public:
  /// `bind_port` of zero leaves the source port ephemeral, which is right for a
  /// send-only link.
  Link(const std::uint16_t bind_port, const std::uint16_t target_port,
    const std::uint8_t system_id)
  : system_id_(system_id)
  {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
      throw std::runtime_error("could not create MAVLink socket");
    }
    if (bind_port != 0) {
      sockaddr_in local{};
      local.sin_family = AF_INET;
      local.sin_port = htons(bind_port);
      local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      if (bind(fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) {
        close(fd_);
        throw std::runtime_error("could not bind UDP port " + std::to_string(bind_port));
      }
    }
    target_.sin_family = AF_INET;
    target_.sin_port = htons(target_port);
    target_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  }

  Link(const Link &) = delete;
  Link & operator=(const Link &) = delete;
  ~Link() {if (fd_ >= 0) {close(fd_);}}

  void heartbeat(const bool armed, const bool offboard)
  {
    mavlink_message_t message{};
    const std::uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
      (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) |
      (offboard ? MAV_MODE_FLAG_GUIDED_ENABLED : 0);
    // The adapter reads the main mode out of bits 16-23. Zero, which this used
    // to send, is not a PX4 main mode at all.
    const std::uint32_t custom_mode =
      (offboard ? kMainModeOffboard : kMainModePosctl) << 16;
    mavlink_msg_heartbeat_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_PX4,
      base_mode, custom_mode, armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    send(message);
  }

  void global_position(
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

  void local_position(
    const std::uint32_t boot_ms, const float north_m, const float east_m, const float down_m,
    const float north_velocity_m_s, const float east_velocity_m_s,
    const float down_velocity_m_s)
  {
    mavlink_message_t message{};
    mavlink_msg_local_position_ned_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms, north_m, east_m, down_m,
      north_velocity_m_s, east_velocity_m_s, down_velocity_m_s);
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

  /// Decode whatever is waiting, up to `kDrainLimit` datagrams.
  ///
  /// The parser state is per link, not the global `MAVLINK_COMM_0` channel: this
  /// process carries N vehicles, and a shared channel would let one link's
  /// partial frame corrupt another's.
  template<typename Handler>
  void drain(Handler && handler)
  {
    for (int datagram = 0; datagram < kDrainLimit; ++datagram) {
      std::array<std::uint8_t, 2048> buffer{};
      const auto size = recv(fd_, buffer.data(), buffer.size(), MSG_DONTWAIT);
      if (size <= 0) {
        return;
      }
      for (ssize_t offset = 0; offset < size; ++offset) {
        mavlink_message_t message{};
        if (mavlink_frame_char_buffer(
            &parser_buffer_, &parser_status_, buffer[static_cast<std::size_t>(offset)], &message,
            nullptr) == MAVLINK_FRAMING_OK)
        {
          handler(message);
        }
      }
    }
  }

private:
  void send(const mavlink_message_t & message)
  {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto length = mavlink_msg_to_send_buffer(buffer.data(), &message);
    // Loopback UDP to a socket nobody is required to be reading: a viewer that
    // is not running, or an adapter that has not started yet, is normal, so a
    // failed send is never fatal.
    (void)sendto(
      fd_, buffer.data(), length, 0, reinterpret_cast<const sockaddr *>(&target_),
      sizeof(target_));
  }

  int fd_{-1};
  std::uint8_t system_id_;
  sockaddr_in target_{};
  mavlink_message_t parser_buffer_{};
  mavlink_status_t parser_status_{};
};

/// One synthetic vehicle: a kinematic integrator wearing PX4's MAVLink manners.
///
/// Everything here runs on the single control thread. There is no other thread
/// in this process now that Edge I/O belongs to the adapter.
class Vehicle final
{
public:
  Vehicle(
    const std::size_t index, const Origin & origin, const std::uint16_t display_port,
    const std::uint16_t control_local_port, const std::uint16_t control_remote_port)
  : id_("px4_" + std::to_string(index)),
    origin_(origin),
    // PX4 SITL sets MAV_SYS_ID to the instance index plus one. The viewer keys
    // on the port rather than the ID, but the adapter addresses this system.
    system_id_(static_cast<std::uint8_t>(index + 1)),
    display_(0, display_port, system_id_),
    control_(control_local_port, control_remote_port, system_id_)
  {
    if (index > 254) {
      throw std::invalid_argument("vehicle index must be in [0, 254]");
    }
    // Every vehicle spawns coincident at the geodetic origin, so they overlap in
    // the viewer until a command separates them. That is expected, not a fault.
  }

  Vehicle(const Vehicle &) = delete;
  Vehicle & operator=(const Vehicle &) = delete;

  void tick(const Clock::time_point now, const float dt)
  {
    receive(now);
    actuate(now);
    integrate(dt);
    emit(now);
  }

private:
  // --- the autopilot's side of the MAVLink conversation ---

  void receive(const Clock::time_point now)
  {
    control_.drain([this, now](const mavlink_message_t & message) {apply(message, now);});
  }

  /// Is this frame for us? The adapter does not filter its inbound traffic, but
  /// an autopilot must: on loopback a mistaken port base would otherwise deliver
  /// another vehicle's setpoints to this one, and the whole fleet would fly the
  /// same path with nothing in any log to say why.
  bool addressed(const std::uint8_t system, const std::uint8_t component) const
  {
    return (system == 0 || system == system_id_) &&
           (component == 0 || component == MAV_COMP_ID_AUTOPILOT1);
  }

  void apply(const mavlink_message_t & message, const Clock::time_point now)
  {
    if (!attached_) {
      attached_ = true;
      std::clog << "[" << id_ << "] adapter attached\n";
    }
    switch (message.msgid) {
      case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED: {
          mavlink_set_position_target_local_ned_t target;
          mavlink_msg_set_position_target_local_ned_decode(&message, &target);
          if (!addressed(target.target_system, target.target_component) ||
            target.coordinate_frame != MAV_FRAME_LOCAL_NED)
          {
            return;
          }
          setpoint_.position = (target.type_mask & kIgnorePosition) == 0;
          setpoint_.velocity = (target.type_mask & kIgnoreVelocity) == 0;
          setpoint_.yaw = (target.type_mask & kIgnoreYaw) == 0;
          setpoint_.north_m = target.x;
          setpoint_.east_m = target.y;
          setpoint_.down_m = target.z;
          setpoint_.north_velocity_m_s = target.vx;
          setpoint_.east_velocity_m_s = target.vy;
          setpoint_.down_velocity_m_s = target.vz;
          setpoint_.yaw_rad = target.yaw;
          setpoint_at_ = now;
          break;
        }
      case MAVLINK_MSG_ID_COMMAND_LONG: {
          mavlink_command_long_t command;
          mavlink_msg_command_long_decode(&message, &command);
          if (!addressed(command.target_system, command.target_component)) {
            return;
          }
          if (command.command == kCommandArmDisarm) {
            // Accepted unconditionally. The adapter's `arm` verb dispatches with
            // no setpoints streaming at all, and a real PX4 sitting on the ground
            // does arm from a bare COMMAND_LONG; modelling preflight checks here
            // would break `./scripts/command.sh arm` and would not be truer.
            request_arm(command.param1 >= 0.5F, now);
          } else if (command.command == kCommandSetMode &&
            command.param1 >= 0.5F &&
            static_cast<std::uint32_t>(command.param2) == kMainModeOffboard)
          {
            request_offboard(now);
          }
          break;
        }
      default:
        break;
    }
  }

  void request_arm(const bool armed, const Clock::time_point now)
  {
    if (requested_armed_ == armed) {
      return;  // A repeat, not a new request: do not push acceptance out again.
    }
    requested_armed_ = armed;
    arm_at_ = now + kActuationLatency;
    if (!armed) {
      // PX4 leaves offboard when it disarms.
      requested_offboard_ = false;
      mode_at_ = now + kActuationLatency;
    }
  }

  void request_offboard(const Clock::time_point now)
  {
    if (!setpoint_live(now)) {
      if (!denied_) {
        denied_ = true;
        std::clog << "[" << id_ << "] offboard denied: no live setpoint stream\n";
      }
      return;
    }
    denied_ = false;
    if (requested_offboard_) {
      return;
    }
    requested_offboard_ = true;
    mode_at_ = now + kActuationLatency;
  }

  bool setpoint_live(const Clock::time_point now) const
  {
    return setpoint_at_ && now - *setpoint_at_ < kOffboardSignalTimeout;
  }

  /// Let pending requests take effect, and drop offboard when the setpoint
  /// stream stops -- PX4's offboard-loss failsafe. Without it, killing an
  /// adapter leaves its vehicle flying the last setpoint forever, and the
  /// failure is invisible.
  void actuate(const Clock::time_point now)
  {
    if (arm_at_ && now >= *arm_at_) {
      armed_ = requested_armed_;
      arm_at_.reset();
      if (!armed_) {
        offboard_ = false;
      }
    }
    if (mode_at_ && now >= *mode_at_) {
      offboard_ = requested_offboard_;
      mode_at_.reset();
    }
    if (offboard_ && !setpoint_live(now)) {
      std::clog << "[" << id_ << "] offboard lost: setpoint stream stopped\n";
      offboard_ = false;
      requested_offboard_ = false;
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

    if (armed_ && offboard_) {
      // Velocity feed-forward from the commanded setpoint plus a proportional
      // pull on the remaining position error, which is what closes the last few
      // metres. Held at the last value between updates rather than interpolated:
      // PX4 does not interpolate either, and smoothing here would hide a
      // setpoint-rate regression in the adapter.
      if (setpoint_.velocity) {
        target_north_velocity = setpoint_.north_velocity_m_s;
        target_east_velocity = setpoint_.east_velocity_m_s;
        target_down_velocity = setpoint_.down_velocity_m_s;
      }
      if (setpoint_.position) {
        target_north_velocity += (setpoint_.north_m - north_m_) * kPositionGain;
        target_east_velocity += (setpoint_.east_m - east_m_) * kPositionGain;
        target_down_velocity += (setpoint_.down_m - down_m_) * kPositionGain;
      }
      if (setpoint_.yaw) {
        target_yaw = setpoint_.yaw_rad;
      }

      const float speed = std::hypot(target_north_velocity, target_east_velocity);
      if (speed > kMaxSpeedMps) {
        const float scale = kMaxSpeedMps / speed;
        target_north_velocity *= scale;
        target_east_velocity *= scale;
      }
      target_down_velocity = std::clamp(target_down_velocity, -kMaxClimbMps, kMaxClimbMps);
    }
    // Disarmed or out of offboard the vehicle sheds velocity rather than
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

  double latitude_deg() const {return origin_.latitude_deg + north_m_ / kMetresPerDegree;}

  double longitude_deg() const
  {
    return origin_.longitude_deg +
           east_m_ / (kMetresPerDegree * std::cos(origin_.latitude_deg * kDegreesToRadians));
  }

  /// Both links stream unconditionally from the first tick, armed or not. The
  /// adapter binds and waits to be dialled, so it learns nothing about this
  /// vehicle until a frame arrives -- a vehicle that only spoke once commanded
  /// could never be commanded at all.
  void emit(const Clock::time_point now)
  {
    const auto boot_ms = static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - started_).count());
    if (now >= next_heartbeat_) {
      next_heartbeat_ = now + kHeartbeatPeriod;
      display_.heartbeat(armed_, offboard_);
      control_.heartbeat(armed_, offboard_);
    }
    if (now < next_telemetry_) {
      return;
    }
    next_telemetry_ = now + kTelemetryPeriod;
    const auto altitude_m = static_cast<float>(origin_.altitude_m) - down_m_;
    display_.global_position(
      boot_ms, latitude_deg(), longitude_deg(), altitude_m, -down_m_, north_velocity_m_s_,
      east_velocity_m_s_, down_velocity_m_s_, yaw_rad_);
    display_.attitude(boot_ms, yaw_rad_, yaw_rate_rad_s_);

    control_.global_position(
      boot_ms, latitude_deg(), longitude_deg(), altitude_m, -down_m_, north_velocity_m_s_,
      east_velocity_m_s_, down_velocity_m_s_, yaw_rad_);
    control_.attitude(boot_ms, yaw_rad_, yaw_rate_rad_s_);
    // Only the control link carries this. It is what validates the adapter's
    // local position estimate, and the display stream is kept deliberately
    // minimal -- see the non-regression rules in DEBUGGING.md.
    control_.local_position(
      boot_ms, north_m_, east_m_, down_m_, north_velocity_m_s_, east_velocity_m_s_,
      down_velocity_m_s_);
  }

public:
  /// Spread this vehicle's outbound work across its own period.
  ///
  /// Every vehicle ticks on one thread, so without a per-vehicle phase offset
  /// all N transmit in the same millisecond and then go quiet for the rest of
  /// the period. That thundering herd is an artefact of the single control
  /// thread: real PX4 instances each keep their own clock and drift apart.
  /// Measured at 20 vehicles, the synchronised burst produced 118
  /// `Deadline Exceeded` retries during startup.
  void set_schedule(const std::size_t index, const std::size_t fleet_size)
  {
    const auto now = Clock::now();
    const auto share = [index, fleet_size](const std::chrono::milliseconds period) {
        return fleet_size == 0 ?
               std::chrono::milliseconds(0) :
               std::chrono::milliseconds(
          static_cast<int>(period.count() * static_cast<long>(index) /
          static_cast<long>(fleet_size)));
      };
    next_telemetry_ = now + share(kTelemetryPeriod);
    next_heartbeat_ = now + share(kHeartbeatPeriod);
  }

private:
  std::string id_;
  Origin origin_;
  std::uint8_t system_id_;
  Link display_;
  Link control_;

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
  bool attached_{false};
  bool denied_{false};
  /// Separate latches. One shared latch swallows a disarm that arrives while an
  /// arm is still taking effect.
  std::optional<Clock::time_point> arm_at_{};
  std::optional<Clock::time_point> mode_at_{};

  Setpoint setpoint_{};
  std::optional<Clock::time_point> setpoint_at_{};

  Clock::time_point started_{Clock::now()};
  Clock::time_point next_telemetry_{};
  Clock::time_point next_heartbeat_{};
};

[[noreturn]] void usage(const char * program, const char * error = nullptr)
{
  if (error != nullptr) {
    std::cerr << "error: " << error << "\n";
  }
  std::cerr
    << "usage: " << program << " --count N [options]\n"
    << "  --count N                    number of synthetic vehicles (1-255)\n"
    << "  --start-index N              first simulator vehicle index (default 0)\n"
    << "  --origin-lat DEG             geodetic origin latitude  (default 36.01883233670948)\n"
    << "  --origin-lon DEG             geodetic origin longitude (default -78.9684198511774)\n"
    << "  --origin-alt M               geodetic origin altitude  (default 0)\n"
    << "  --display-port-base P        first viewer port          (default 19410)\n"
    << "  --control-local-port-base P  first autopilot bind port  (default 25540)\n"
    << "  --control-remote-port-base P first adapter port         (default 24540)\n";
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

/// Every port this fleet will occupy, checked before a single socket is opened.
/// A silent overlap between two of these ranges is a whole fleet flying one
/// vehicle's setpoints, which is far harder to recognise than a refusal here.
void check_port_ranges(const long count, const long start_index,
  const std::array<long, 3> & bases,
  const char * program)
{
  for (const auto base : bases) {
    if (base < 1 || base + start_index + count > 65536) {
      usage(program, "a port base plus --count falls outside the port range");
    }
  }
  for (std::size_t first = 0; first < bases.size(); ++first) {
    for (std::size_t second = first + 1; second < bases.size(); ++second) {
      const auto low = std::min(bases[first], bases[second]);
      const auto high = std::max(bases[first], bases[second]);
      if (high - low < count) {
        usage(program, "two port ranges overlap at this --count");
      }
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  long count = 0;
  long start_index = 0;
  Origin origin;
  long display_base = 19410;
  long control_local_base = 25540;
  long control_remote_base = 24540;

  for (int index = 1; index < argc; ++index) {
    const std::string_view flag(argv[index]);
    const auto value = [&]() -> const char * {
        if (index + 1 >= argc) {
          usage(argv[0], "missing value");
        }
        return argv[++index];
      };
    if (flag == "--count") {
      count = std::lround(number(value(), "--count"));
    } else if (flag == "--start-index") {
      start_index = std::lround(number(value(), "--start-index"));
    } else if (flag == "--origin-lat") {
      origin.latitude_deg = number(value(), "--origin-lat");
    } else if (flag == "--origin-lon") {
      origin.longitude_deg = number(value(), "--origin-lon");
    } else if (flag == "--origin-alt") {
      origin.altitude_m = number(value(), "--origin-alt");
    } else if (flag == "--display-port-base") {
      display_base = std::lround(number(value(), "--display-port-base"));
    } else if (flag == "--control-local-port-base") {
      control_local_base = std::lround(number(value(), "--control-local-port-base"));
    } else if (flag == "--control-remote-port-base") {
      control_remote_base = std::lround(number(value(), "--control-remote-port-base"));
    } else {
      usage(argv[0], "unknown option");
    }
  }
  if (count < 1 || count > 255) {
    usage(argv[0], "--count must be in [1, 255]");
  }
  if (start_index < 0 || start_index + count > 255) {
    usage(argv[0], "--start-index plus --count must fit vehicle indices [0, 254]");
  }
  check_port_ranges(
    count, start_index, {display_base, control_local_base, control_remote_base}, argv[0]);

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::vector<std::unique_ptr<Vehicle>> fleet;
  fleet.reserve(static_cast<std::size_t>(count));
  try {
    for (long index = 0; index < count; ++index) {
      auto vehicle = std::make_unique<Vehicle>(
        static_cast<std::size_t>(start_index + index), origin,
        static_cast<std::uint16_t>(display_base + start_index + index),
        static_cast<std::uint16_t>(control_local_base + start_index + index),
        static_cast<std::uint16_t>(control_remote_base + start_index + index));
      vehicle->set_schedule(static_cast<std::size_t>(index), static_cast<std::size_t>(count));
      fleet.push_back(std::move(vehicle));
    }
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }

  std::cout << "synthetic fleet: " << count << " vehicles, display "
            << (display_base + start_index) << "-" << (display_base + start_index + count - 1)
            << ", control " << (control_local_base + start_index) << "-"
            << (control_local_base + start_index + count - 1) << " -> "
            << (control_remote_base + start_index) << "-"
            << (control_remote_base + start_index + count - 1) << "\n"
            << std::flush;

  // One thread drives every vehicle. The per-vehicle work is a handful of
  // floating-point operations and a few datagrams, so 100+ vehicles at 50 Hz is
  // a fraction of a core -- which is the entire point of this tool. With Edge
  // I/O gone to the adapters, this is now the only thread in the process.
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
