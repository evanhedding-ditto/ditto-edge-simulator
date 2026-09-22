// Factory robot fleet: N ground robots driving around a multi-level building.
//
// Sibling to synthetic_fleet.cpp, and deliberately not a mode of it. That tool
// emulates a PX4 autopilot -- arm/disarm, offboard entry, setpoint timeouts --
// because its job is to exercise the adapter's control machinery against
// something that refuses the way a real autopilot refuses. A factory robot has
// none of that, and bolting a flag onto the one tool that is supposed to be an
// apples-to-apples PX4 stand-in would make it a worse stand-in. The two share
// the MAVLink emitter idiom and nothing else.
//
// Like synthetic_fleet, this links nothing from the adapters repository and
// holds no Edge Server connection: it speaks MAVLink on loopback and that is
// all. Ditto writes are an adapter's job, one process per robot, exactly as for
// real PX4. Nothing here needs to change when the network work lands.
//
// THE WORLD FILE IS READ HERE, ON THE VEHICLE SIDE OF THE MAVLINK BOUNDARY.
// The viewer reads the same file for its own drawing and the two never talk.
// This matters: the simulator's standing rule is that viewer truth never
// reaches an autonomy decision, and a robot that learned about walls from the
// renderer would break it. A robot knowing the floor plan of the building it
// works in is ordinary -- a real AGV has exactly that map on board.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

#include <mavlink/common/mavlink.h>

namespace {

using Clock = std::chrono::steady_clock;

/// Ground-robot motion limits. A factory AGV moves at about walking pace; the
/// acceleration limit is what stops a robot snapping to full speed the instant
/// it finishes a turn, and the yaw rate is what makes a corner read as a turn
/// rather than a slide.
constexpr float kMaxSpeedMps = 1.4F;
constexpr float kMaxAccelMps2 = 0.8F;
constexpr float kMaxYawRateRadS = 1.2F;

/// A unicycle drives along its heading and nowhere else, which is the whole
/// difference between reading as a robot and reading as a drone strafing. Below
/// this alignment error the robot drives; past it, it turns in place. The
/// cosine taper between the two keeps it from lurching at the threshold.
constexpr float kDriveAlignmentRad = 0.45F;

/// How close counts as arrived. Smaller than this and a robot circles a
/// waypoint it can never quite satisfy at 50 Hz.
constexpr float kWaypointRadiusM = 0.30F;

/// Perpendicular approach distance either side of a doorway. Waypoints are
/// placed here, not on the threshold itself, so a robot crosses a wall opening
/// square-on. Without them a diagonal leg lets a robot cut the corner through
/// the wall beside the door -- the one artefact that would make the whole
/// building look fake.
constexpr float kDoorApproachM = 1.3F;

/// Integration tick and stream rates, matched to synthetic_fleet so the viewer
/// sees both fleets move with the same smoothness.
constexpr auto kControlPeriod = std::chrono::milliseconds(20);
constexpr auto kTelemetryPeriod = std::chrono::milliseconds(100);
constexpr auto kHeartbeatPeriod = std::chrono::seconds(1);

/// Datagrams discarded per link per tick, so one noisy adapter cannot stall the
/// rest of the fleet.
constexpr int kDrainLimit = 16;

/// PX4 packs its main mode into bits 16-23 of `custom_mode`; the adapter reads
/// it from there to decide whether a vehicle is in offboard.
constexpr std::uint32_t kMainModeOffboard = 6;
constexpr std::uint32_t kMainModePosctl = 3;

/// Command IDs from the MAVLink common specification, written out because PX4's
/// build trims the `MAV_CMD` enum away.
constexpr std::uint16_t kCommandSetMode = 176;
constexpr std::uint16_t kCommandArmDisarm = 400;

/// How far a commanded target must move before it counts as a new order. The
/// adapter streams its setpoint continuously at 10 Hz, so without this every
/// packet would re-plan the route and the robot would never leave the spot.
constexpr float kNewTargetThresholdM = 0.5F;

/// Must match viewer/src/main.cpp, which converts the other way.
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

struct Point
{
  float north_m{};
  float east_m{};
};

// ---------------------------------------------------------------------------
// The building
// ---------------------------------------------------------------------------

struct Room
{
  std::string id;
  std::string label;
  int level{};
  float north_min_m{};
  float north_max_m{};
  float east_min_m{};
  float east_max_m{};
  /// Part of the balcony walkway rather than a room proper.
  bool circulation{};

  Point centre() const
  {
    return {(north_min_m + north_max_m) * 0.5F, (east_min_m + east_max_m) * 0.5F};
  }

  /// An inset that cannot invert on a narrow room. A balcony segment is three
  /// metres across, and a fixed 1.5 m inset collapses its range to a point --
  /// or past it, which is undefined for a uniform distribution.
  float inset_north(const float wanted) const
  {
    return std::min(wanted, (north_max_m - north_min_m) / 3.0F);
  }

  float inset_east(const float wanted) const
  {
    return std::min(wanted, (east_max_m - east_min_m) / 3.0F);
  }

  bool contains(const Point & point, const float margin = 0.0F) const
  {
    return point.north_m >= north_min_m + margin && point.north_m <= north_max_m - margin &&
      point.east_m >= east_min_m + margin && point.east_m <= east_max_m - margin;
  }
};

/// A doorway, already resolved to the two rooms it joins.
///
/// `normal_is_north` records which way the wall runs: a doorway in a wall at a
/// fixed north coordinate is crossed by travelling north or south, so the
/// approach offsets are applied along north. Getting this backwards puts the
/// approach waypoints inside the wall.
struct Doorway
{
  std::size_t room_a{};
  std::size_t room_b{};
  Point at;
  bool normal_is_north{};
  /// Navigable width of the opening. Defaults to the building's door width; a
  /// balcony corner join declares the whole shared edge, because no wall is
  /// drawn there and funnelling robots through a door-sized gap in open floor
  /// would look like a path they had no reason to take.
  float width_m{};
};

struct Level
{
  int index{};
  float base_m{};
  std::string label;
};

struct Building
{
  std::vector<Level> levels;
  std::vector<Room> rooms;
  std::vector<Doorway> doorways;
  /// room index -> (neighbour room index, doorway index)
  std::vector<std::vector<std::pair<std::size_t, std::size_t>>> adjacency;
  std::vector<int> robots_per_level;
  /// Restrict the patrol to the balcony walkway. The rooms still exist and are
  /// still drawn; the robots simply do not go into them. A setting rather than
  /// a hard rule because the two make different demos: the balcony is where
  /// levels are in sight of each other across the void, and a building-wide
  /// patrol is where the walls between rooms matter.
  bool patrol_circulation_only{};
  float robot_radius_m{0.35F};
  float door_width_m{1.6F};

  const Level & level_of(const Room & room) const
  {
    for (const auto & level : levels) {
      if (level.index == room.level) return level;
    }
    throw std::runtime_error("room '" + room.id + "' names a level the world does not define");
  }
};

float json_float(const nlohmann::json & node, const char * key)
{
  const auto entry = node.find(key);
  if (entry == node.end() || !entry->is_number()) {
    throw std::runtime_error(std::string("world: missing numeric field '") + key + "'");
  }
  const auto value = entry->get<float>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(std::string("world: field '") + key + "' is not finite");
  }
  return value;
}

/// Read the `building` section of a world file.
///
/// Unlike the viewer's loader, which treats a malformed world as "draw nothing"
/// because a missing decoration is not worth refusing to start over, this one
/// throws. A fleet with no floor plan has nowhere to drive, and twenty robots
/// silently stacked at the origin is a worse outcome than a clear refusal.
Building read_building(const std::string & path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open world file '" + path + "'");
  nlohmann::json document;
  input >> document;

  const auto section = document.find("building");
  if (section == document.end()) {
    throw std::runtime_error("world '" + path + "' has no `building` section");
  }

  Building building;
  building.robot_radius_m = section->value("robot_radius_m", 0.35F);
  building.door_width_m = section->value("door_width_m", 1.6F);

  for (const auto & entry : section->at("levels")) {
    Level level;
    level.index = entry.at("index").get<int>();
    level.base_m = json_float(entry, "base_m");
    level.label = entry.value("label", "");
    building.levels.push_back(std::move(level));
  }
  if (building.levels.empty()) throw std::runtime_error("world has no levels");

  for (const auto & entry : section->at("rooms")) {
    Room room;
    room.id = entry.at("id").get<std::string>();
    room.label = entry.value("label", room.id);
    room.level = entry.at("level").get<int>();
    room.north_min_m = json_float(entry, "north_min_m");
    room.north_max_m = json_float(entry, "north_max_m");
    room.east_min_m = json_float(entry, "east_min_m");
    room.east_max_m = json_float(entry, "east_max_m");
    room.circulation = entry.value("circulation", false);
    if (room.north_max_m <= room.north_min_m || room.east_max_m <= room.east_min_m) {
      throw std::runtime_error("room '" + room.id + "' has non-positive extent");
    }
    building.rooms.push_back(std::move(room));
  }
  if (building.rooms.empty()) throw std::runtime_error("world has no rooms");

  // Resolving level references now means level_of() can never fail later.
  for (const auto & room : building.rooms) {(void) building.level_of(room);}

  const auto index_of = [&](const std::string & id) {
      for (std::size_t index = 0; index < building.rooms.size(); ++index) {
        if (building.rooms[index].id == id) return index;
      }
      throw std::runtime_error("doorway names unknown room '" + id + "'");
    };

  for (const auto & entry : section->at("doorways")) {
    const auto & between = entry.at("between");
    if (!between.is_array() || between.size() != 2) {
      throw std::runtime_error("doorway `between` must name exactly two rooms");
    }
    Doorway doorway;
    doorway.room_a = index_of(between.at(0).get<std::string>());
    doorway.room_b = index_of(between.at(1).get<std::string>());
    doorway.at = {json_float(entry, "north_m"), json_float(entry, "east_m")};
    doorway.width_m = entry.value("width_m", building.door_width_m);

    const Room & a = building.rooms[doorway.room_a];
    const Room & b = building.rooms[doorway.room_b];
    if (a.level != b.level) {
      throw std::runtime_error("doorway between '" + a.id + "' and '" + b.id + "' crosses levels");
    }
    // Which wall is shared decides the crossing direction. Checking the
    // geometry rather than trusting a hand-written flag means a mistyped
    // doorway is caught here instead of appearing as a robot walking through a
    // wall twenty minutes into a demo.
    const bool shares_north = std::fabs(a.north_max_m - b.north_min_m) < 1e-4F ||
      std::fabs(b.north_max_m - a.north_min_m) < 1e-4F;
    const bool shares_east = std::fabs(a.east_max_m - b.east_min_m) < 1e-4F ||
      std::fabs(b.east_max_m - a.east_min_m) < 1e-4F;
    if (shares_north) {
      doorway.normal_is_north = true;
    } else if (shares_east) {
      doorway.normal_is_north = false;
    } else {
      throw std::runtime_error(
        "rooms '" + a.id + "' and '" + b.id + "' share no wall for their doorway");
    }
    building.doorways.push_back(doorway);
  }

  building.adjacency.resize(building.rooms.size());
  for (std::size_t index = 0; index < building.doorways.size(); ++index) {
    const auto & doorway = building.doorways[index];
    building.adjacency[doorway.room_a].emplace_back(doorway.room_b, index);
    building.adjacency[doorway.room_b].emplace_back(doorway.room_a, index);
  }

  const auto fleet = section->find("fleet");
  if (fleet != section->end()) {
    const auto per_level = fleet->find("robots_per_level");
    if (per_level != fleet->end() && per_level->is_array()) {
      for (const auto & entry : *per_level) building.robots_per_level.push_back(entry.get<int>());
    }
    building.patrol_circulation_only = fleet->value("patrol", "") == "balcony";
  }
  if (building.patrol_circulation_only) {
    for (const auto & level : building.levels) {
      const bool any = std::any_of(
        building.rooms.begin(), building.rooms.end(), [&](const Room & room) {
          return room.level == level.index && room.circulation;
        });
      if (!any) {
        throw std::runtime_error(
          "fleet.patrol is 'balcony' but level " + std::to_string(level.index) +
          " has no circulation rooms to patrol");
      }
    }
  }
  return building;
}

/// Breadth-first room-to-room path. Rooms per level are single digits, so the
/// simplest correct search is also the right one; nothing here justifies A*.
std::vector<std::size_t> room_path(
  const Building & building, const std::size_t from, const std::size_t to)
{
  if (from == to) return {from};
  std::vector<std::size_t> previous(building.rooms.size(), SIZE_MAX);
  std::deque<std::size_t> queue{from};
  previous[from] = from;
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop_front();
    if (current == to) break;
    for (const auto & [neighbour, doorway] : building.adjacency[current]) {
      (void) doorway;
      if (previous[neighbour] != SIZE_MAX) continue;
      previous[neighbour] = current;
      queue.push_back(neighbour);
    }
  }
  if (previous[to] == SIZE_MAX) return {};
  std::vector<std::size_t> path;
  for (auto step = to; step != from; step = previous[step]) path.push_back(step);
  path.push_back(from);
  std::reverse(path.begin(), path.end());
  return path;
}

/// The doorway joining two adjacent rooms, or SIZE_MAX if they are not adjacent.
std::size_t doorway_between(
  const Building & building, const std::size_t a, const std::size_t b)
{
  for (const auto & [neighbour, doorway] : building.adjacency[a]) {
    if (neighbour == b) return doorway;
  }
  return SIZE_MAX;
}

// ---------------------------------------------------------------------------
// MAVLink
// ---------------------------------------------------------------------------

/// One MAVLink link, optionally bound.
///
/// A bound link sends from the socket it listens on, which is not incidental:
/// the adapter dials in with `udpin`, so it learns where to reply from the
/// source address of whatever arrived most recently. Sending from a second
/// socket would send its replies somewhere else entirely.
///
/// REPORT-ONLY, for now. The robots run their own patrol and take no setpoints,
/// so inbound traffic is drained and discarded rather than acted on. The bind
/// still matters: it is what makes the adapter's reply path exist, so adding
/// command handling later is a change to `drain` and nothing else.
class Link final
{
public:
  /// `bind_port` of zero leaves the source port ephemeral, which is right for a
  /// send-only link such as the viewer's.
  Link(const std::uint16_t bind_port, const std::uint16_t target_port,
    const std::uint8_t system_id)
  : system_id_(system_id)
  {
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) throw std::runtime_error("could not create MAVLink socket");
    if (bind_port != 0) {
      const int flags = fcntl(fd_, F_GETFL, 0);
      if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        close(fd_);
        throw std::runtime_error("could not configure MAVLink socket");
      }
      sockaddr_in local{};
      local.sin_family = AF_INET;
      local.sin_port = htons(bind_port);
      local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      if (bind(fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) != 0) {
        close(fd_);
        throw std::runtime_error("could not bind UDP port " + std::to_string(bind_port));
      }
      bound_ = true;
    }
    target_.sin_family = AF_INET;
    target_.sin_port = htons(target_port);
    target_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  }

  /// Decode whatever is waiting, up to `kDrainLimit` datagrams.
  template<typename Handler>
  void receive(const Handler & handler)
  {
    if (!bound_) return;
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    for (int count = 0; count < kDrainLimit; ++count) {
      const auto size = recv(fd_, buffer.data(), buffer.size(), 0);
      if (size < 0) return;
      for (ssize_t index = 0; index < size; ++index) {
        mavlink_message_t message{};
        if (mavlink_frame_char_buffer(
            &parser_buffer_, &parser_status_, buffer[static_cast<std::size_t>(index)],
            &message, nullptr) == MAVLINK_FRAMING_OK)
        {
          handler(message);
        }
      }
    }
  }

  Link(const Link &) = delete;
  Link & operator=(const Link &) = delete;
  ~Link() {if (fd_ >= 0) {close(fd_);}}

  /// A ground rover, and always "armed": a factory robot under its own
  /// supervision has no arming ceremony to model, and the viewer colours a
  /// disarmed vehicle as inactive.
  /// The adapter watches this to decide a command has been accepted: a goto is
  /// complete only once the heartbeat reports both armed and offboard, so these
  /// flags are the acknowledgement, not decoration.
  void heartbeat(const bool armed, const bool offboard)
  {
    mavlink_message_t message{};
    const std::uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
      (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0) |
      (offboard ? MAV_MODE_FLAG_GUIDED_ENABLED : 0);
    // PX4 packs its main mode into bits 16-23, and the adapter reads it there.
    const std::uint32_t custom_mode =
      static_cast<std::uint32_t>(offboard ? kMainModeOffboard : kMainModePosctl) << 16;
    mavlink_msg_heartbeat_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_PX4,
      base_mode, custom_mode, armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    send(message);
  }

  void global_position(
    const std::uint32_t boot_ms, const double latitude_deg, const double longitude_deg,
    const double altitude_m, const double relative_altitude_m, const float north_velocity_m_s,
    const float east_velocity_m_s, const float heading_rad)
  {
    mavlink_message_t message{};
    auto heading_cdeg = static_cast<std::int32_t>(std::lround(wrap(heading_rad) * 18000.0F / kPi));
    heading_cdeg = ((heading_cdeg % 36000) + 36000) % 36000;
    mavlink_msg_global_position_int_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms,
      static_cast<std::int32_t>(std::llround(latitude_deg * 1e7)),
      static_cast<std::int32_t>(std::llround(longitude_deg * 1e7)),
      static_cast<std::int32_t>(std::llround(altitude_m * 1000.0)),
      static_cast<std::int32_t>(std::llround(relative_altitude_m * 1000.0)),
      static_cast<std::int16_t>(std::lround(north_velocity_m_s * 100.0F)),
      static_cast<std::int16_t>(std::lround(east_velocity_m_s * 100.0F)), 0,
      static_cast<std::uint16_t>(heading_cdeg));
    send(message);
  }

  void local_position(
    const std::uint32_t boot_ms, const float north_m, const float east_m, const float down_m,
    const float north_velocity_m_s, const float east_velocity_m_s)
  {
    mavlink_message_t message{};
    mavlink_msg_local_position_ned_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms, north_m, east_m, down_m,
      north_velocity_m_s, east_velocity_m_s, 0.0F);
    send(message);
  }

  void attitude(const std::uint32_t boot_ms, const float yaw_rad, const float yaw_rate_rad_s)
  {
    mavlink_message_t message{};
    // A robot on a flat floor has no roll or pitch. Only yaw is real.
    mavlink_msg_attitude_pack(
      system_id_, MAV_COMP_ID_AUTOPILOT1, &message, boot_ms, 0.0F, 0.0F, wrap(yaw_rad), 0.0F,
      0.0F, yaw_rate_rad_s);
    send(message);
  }

private:
  void send(const mavlink_message_t & message)
  {
    std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
    const auto size = mavlink_msg_to_send_buffer(buffer.data(), &message);
    (void) sendto(
      fd_, buffer.data(), size, 0, reinterpret_cast<const sockaddr *>(&target_), sizeof(target_));
  }

  std::uint8_t system_id_{};
  bool bound_{};
  mavlink_message_t parser_buffer_{};
  mavlink_status_t parser_status_{};
  int fd_{-1};
  sockaddr_in target_{};
};

// ---------------------------------------------------------------------------
// The robot
// ---------------------------------------------------------------------------

class Robot final
{
public:
  Robot(
    const std::size_t index, const Building & building, const std::size_t start_room,
    const Origin & origin, const std::uint16_t display_port,
    const std::uint16_t control_local_port, const std::uint16_t control_remote_port,
    const std::uint32_t seed)
  : building_(building), origin_(origin), room_(start_room), random_(seed),
    display_(0, display_port, static_cast<std::uint8_t>(index + 1)),
    control_(control_local_port, control_remote_port, static_cast<std::uint8_t>(index + 1))
  {
    const Room & room = building_.rooms[room_];
    // Spread the starting positions across the room rather than stacking every
    // robot on its centre.
    const float spawn_north = room.inset_north(1.5F);
    const float spawn_east = room.inset_east(1.5F);
    std::uniform_real_distribution<float> north(
      room.north_min_m + spawn_north, room.north_max_m - spawn_north);
    std::uniform_real_distribution<float> east(
      room.east_min_m + spawn_east, room.east_max_m - spawn_east);
    position_ = {north(random_), east(random_)};
    std::uniform_real_distribution<float> heading(-kPi, kPi);
    yaw_rad_ = heading(random_);
    level_base_m_ = building_.level_of(room).base_m;
    boot_ = Clock::now();
    dwell_until_ = boot_;
  }

  void tick(const Clock::time_point now, const float dt)
  {
    control_.receive([this](const mavlink_message_t & message) {apply(message);});
    if (waypoints_.empty()) {
      // A commanded robot holds where it was sent and works there until the
      // next order. Only a robot on its own patrol picks somewhere new to be.
      if (commanded_ || now < dwell_until_) {
        decelerate(dt);
        publish(now);
        return;
      }
      choose_destination();
    }
    steer(dt);
    publish(now);
  }

private:
  /// Act on the adapter's traffic: arm, offboard entry, and position setpoints.
  ///
  /// The setpoint is taken as a DESTINATION, not as a position to track. The
  /// adapter's guidance was written for an aircraft and aims straight at the
  /// target; a robot that followed it literally would drive into the first wall
  /// between here and there. Routing to it through the building is the ground
  /// equivalent of the same order, and it is why a factory robot cannot simply
  /// reuse the flying setpoint loop.
  void apply(const mavlink_message_t & message)
  {
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
      mavlink_command_long_t command{};
      mavlink_msg_command_long_decode(&message, &command);
      if (command.command == kCommandArmDisarm) {
        armed_ = command.param1 != 0.0F;
      } else if (command.command == kCommandSetMode) {
        offboard_ = static_cast<std::uint32_t>(command.param2) == kMainModeOffboard;
      }
      return;
    }
    if (message.msgid != MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) return;
    mavlink_set_position_target_local_ned_t target{};
    mavlink_msg_set_position_target_local_ned_decode(&message, &target);
    if (!std::isfinite(target.x) || !std::isfinite(target.y)) return;
    const Point wanted{target.x, target.y};
    if (commanded_ && std::hypot(wanted.north_m - commanded_at_.north_m,
        wanted.east_m - commanded_at_.east_m) < kNewTargetThresholdM)
    {
      return;   // The same order, restated. It is streamed continuously.
    }
    // Only somewhere this robot could actually stand. A target in another
    // level's room, or in the atrium, has no route and is left alone rather
    // than silently turned into the nearest thing that does.
    const auto destination = room_containing(wanted);
    if (destination == SIZE_MAX) return;
    commanded_ = true;
    commanded_at_ = wanted;
    waypoints_.clear();
    route_to(destination, wanted);
  }

  /// The room on this robot's own level holding a point, or SIZE_MAX.
  std::size_t room_containing(const Point & point) const
  {
    const int level = building_.rooms[room_].level;
    for (std::size_t index = 0; index < building_.rooms.size(); ++index) {
      const Room & room = building_.rooms[index];
      if (room.level == level && room.contains(point)) return index;
    }
    return SIZE_MAX;
  }

  void choose_destination()
  {
    // Pick another room on this level and path to it. Same-level only: the lift
    // is drawn but nothing rides it yet, so a cross-level target would have no
    // route and strand the robot.
    std::vector<std::size_t> candidates;
    for (std::size_t index = 0; index < building_.rooms.size(); ++index) {
      const Room & candidate = building_.rooms[index];
      if (index == room_ || candidate.level != building_.rooms[room_].level) continue;
      if (building_.patrol_circulation_only && !candidate.circulation) continue;
      candidates.push_back(index);
    }
    if (candidates.empty()) return;
    std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
    const auto target = candidates[pick(random_)];

    // Finish somewhere inside the destination room, clear of its walls.
    const Room & destination = building_.rooms[target];
    const float inset_north = destination.inset_north(1.5F);
    const float inset_east = destination.inset_east(1.5F);
    std::uniform_real_distribution<float> north(
      destination.north_min_m + inset_north, destination.north_max_m - inset_north);
    std::uniform_real_distribution<float> east(
      destination.east_min_m + inset_east, destination.east_max_m - inset_east);
    route_to(target, {north(random_), east(random_)});
  }

  /// Lay a waypoint run from the current room to a point in `target`.
  void route_to(const std::size_t target, const Point final_point)
  {
    const auto path = room_path(building_, room_, target);
    if (path.empty()) return;

    for (std::size_t step = 0; step + 1 < path.size(); ++step) {
      const auto doorway_index = doorway_between(building_, path[step], path[step + 1]);
      if (doorway_index == SIZE_MAX) return;   // Should be unreachable; refuse rather than guess.
      const auto & doorway = building_.doorways[doorway_index];
      // Approach square-on: a waypoint short of the opening, the opening, then
      // one past it. Which side is "short" depends on which room we are leaving.
      const Room & leaving = building_.rooms[path[step]];
      const Point leaving_centre = leaving.centre();
      const float sign_north = doorway.at.north_m > leaving_centre.north_m ? -1.0F : 1.0F;
      const float sign_east = doorway.at.east_m > leaving_centre.east_m ? -1.0F : 1.0F;
      if (doorway.normal_is_north) {
        waypoints_.push_back(
          {doorway.at.north_m + sign_north * kDoorApproachM, doorway.at.east_m});
        waypoints_.push_back(doorway.at);
        waypoints_.push_back(
          {doorway.at.north_m - sign_north * kDoorApproachM, doorway.at.east_m});
      } else {
        waypoints_.push_back(
          {doorway.at.north_m, doorway.at.east_m + sign_east * kDoorApproachM});
        waypoints_.push_back(doorway.at);
        waypoints_.push_back(
          {doorway.at.north_m, doorway.at.east_m - sign_east * kDoorApproachM});
      }
    }

    waypoints_.push_back(final_point);
  }

  void decelerate(const float dt)
  {
    speed_mps_ = std::max(0.0F, speed_mps_ - kMaxAccelMps2 * dt);
    advance(dt);
  }

  void steer(const float dt)
  {
    const Point target = waypoints_.front();
    const float to_north = target.north_m - position_.north_m;
    const float to_east = target.east_m - position_.east_m;
    const float range = std::hypot(to_north, to_east);
    if (range < kWaypointRadiusM) {
      waypoints_.pop_front();
      if (waypoints_.empty()) {
        std::uniform_real_distribution<float> pause(2.0F, 6.0F);
        dwell_until_ = Clock::now() + std::chrono::milliseconds(
          static_cast<int>(pause(random_) * 1000.0F));
      }
      decelerate(dt);
      return;
    }

    // Bearing in the NED convention the rest of the simulator uses: yaw zero is
    // north, positive toward east.
    const float bearing = std::atan2(to_east, to_north);
    const float error = wrap(bearing - yaw_rad_);
    const float turn = std::clamp(error, -kMaxYawRateRadS * dt, kMaxYawRateRadS * dt);
    yaw_rate_rad_s_ = turn / std::max(dt, 1e-6F);
    yaw_rad_ = wrap(yaw_rad_ + turn);

    // Drive only when roughly facing the target, tapering to zero as the error
    // opens up. Past the alignment limit the robot turns on the spot.
    const float alignment = std::fabs(wrap(bearing - yaw_rad_));
    float wanted = 0.0F;
    if (alignment < kDriveAlignmentRad) {
      wanted = kMaxSpeedMps * std::cos(alignment * kPi * 0.5F / kDriveAlignmentRad);
      // Ease off approaching the waypoint so the robot settles rather than
      // overshooting and turning back.
      wanted = std::min(wanted, range * 1.5F);
    }
    const float step = kMaxAccelMps2 * dt;
    speed_mps_ += std::clamp(wanted - speed_mps_, -step, step);
    speed_mps_ = std::clamp(speed_mps_, 0.0F, kMaxSpeedMps);
    advance(dt);
  }

  void advance(const float dt)
  {
    position_.north_m += std::cos(yaw_rad_) * speed_mps_ * dt;
    position_.east_m += std::sin(yaw_rad_) * speed_mps_ * dt;
    contain();
  }

  /// Keep the robot inside the room it is in.
  ///
  /// The doorway approach waypoints already route every crossing square through
  /// an opening, so in normal operation this changes nothing. It is here for the
  /// case that matters visually: any numerical drift or a future command that
  /// aims a robot at a wall should stop it at the wall rather than let it walk
  /// through the one feature the whole world exists to show. A robot within half
  /// a door width of an opening is passing through it and is left alone.
  void contain()
  {
    const Room & room = building_.rooms[room_];
    const float margin = building_.robot_radius_m;

    const auto in_doorway = [&](const bool crossing_north) {
        for (const auto & [neighbour, index] : building_.adjacency[room_]) {
          (void) neighbour;
          const auto & doorway = building_.doorways[index];
          if (doorway.normal_is_north != crossing_north) continue;
          const float aperture = doorway.width_m * 0.5F;
          const float along = crossing_north ?
            std::fabs(position_.east_m - doorway.at.east_m) :
            std::fabs(position_.north_m - doorway.at.north_m);
          const float across = crossing_north ?
            std::fabs(position_.north_m - doorway.at.north_m) :
            std::fabs(position_.east_m - doorway.at.east_m);
          if (along < aperture && across < kDoorApproachM + margin) return true;
        }
        return false;
      };

    if (!in_doorway(true)) {
      position_.north_m = std::clamp(
        position_.north_m, room.north_min_m + margin, room.north_max_m - margin);
    }
    if (!in_doorway(false)) {
      position_.east_m = std::clamp(
        position_.east_m, room.east_min_m + margin, room.east_max_m - margin);
    }

    // Re-home to whichever room now holds the robot, so the next containment
    // test uses the room it has moved into rather than the one it left.
    if (!room.contains(position_)) {
      for (const auto & [neighbour, index] : building_.adjacency[room_]) {
        (void) index;
        if (building_.rooms[neighbour].contains(position_)) {
          room_ = neighbour;
          break;
        }
      }
    }
  }

  void publish(const Clock::time_point now)
  {
    if (now >= next_heartbeat_) {
      display_.heartbeat(armed_, offboard_);
      control_.heartbeat(armed_, offboard_);
      next_heartbeat_ = now + kHeartbeatPeriod;
    }
    if (now < next_telemetry_) return;
    next_telemetry_ = now + kTelemetryPeriod;

    const auto boot_ms = static_cast<std::uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now - boot_).count());
    const float north_velocity = std::cos(yaw_rad_) * speed_mps_;
    const float east_velocity = std::sin(yaw_rad_) * speed_mps_;

    // Down is negative up: a robot on level 2 stands 8 m above the home datum.
    const float down_m = -level_base_m_;

    const double latitude = origin_.latitude_deg + position_.north_m / kMetresPerDegree;
    const double longitude = origin_.longitude_deg +
      position_.east_m / (kMetresPerDegree * std::cos(origin_.latitude_deg * kDegreesToRadians));

    // Both links carry the same picture. The viewer reads the display link and
    // the adapter reads the control link; giving them separate sockets is what
    // lets the viewer run with no adapter, and an adapter run with no viewer.
    for (Link * link : {&display_, &control_}) {
      link->global_position(
        boot_ms, latitude, longitude, origin_.altitude_m + level_base_m_, level_base_m_,
        north_velocity, east_velocity, yaw_rad_);
      link->local_position(
        boot_ms, position_.north_m, position_.east_m, down_m, north_velocity, east_velocity);
      link->attitude(boot_ms, yaw_rad_, yaw_rate_rad_s_);
    }
  }

  const Building & building_;
  Origin origin_;
  std::size_t room_{};
  std::deque<Point> waypoints_;
  Point position_{};
  float yaw_rad_{};
  float yaw_rate_rad_s_{};
  float speed_mps_{};
  float level_base_m_{};
  std::mt19937 random_;
  /// Under orders from the adapter rather than on its own patrol.
  bool commanded_{};
  Point commanded_at_{};
  /// Reported in HEARTBEAT. A robot is armed and offboard only once told to be,
  /// so the adapter's dispatch/accept handshake completes against real state
  /// rather than against a flag that was always true.
  bool armed_{};
  bool offboard_{};
  Clock::time_point boot_{};
  Clock::time_point dwell_until_{};
  Clock::time_point next_telemetry_{};
  Clock::time_point next_heartbeat_{};
  Link display_;
  Link control_;
};

[[noreturn]] void usage(const char * program, const char * problem)
{
  std::cerr << "error: " << problem << "\n"
            << "usage: " << program << " --world <path> [--count N]"
            << " [--display-port-base P] [--control-local-port-base P]"
            << " [--control-remote-port-base P]"
            << " [--origin-lat D] [--origin-lon D] [--origin-alt M]\n";
  std::exit(2);
}

double number(const char * text, const char * name)
{
  try {
    std::size_t consumed = 0;
    const double value = std::stod(text, &consumed);
    if (consumed != std::string_view(text).size()) throw std::invalid_argument("trailing");
    return value;
  } catch (const std::exception &) {
    std::cerr << "error: " << name << " must be a number\n";
    std::exit(2);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string world_path;
  long count = 0;
  long display_base = 19410;
  long control_local_base = 25540;
  long control_remote_base = 24540;
  Origin origin;

  for (int index = 1; index < argc; ++index) {
    const std::string_view flag(argv[index]);
    const auto value = [&]() -> const char * {
        if (index + 1 >= argc) usage(argv[0], "missing value");
        return argv[++index];
      };
    if (flag == "--world") {
      world_path = value();
    } else if (flag == "--count") {
      count = std::lround(number(value(), "--count"));
    } else if (flag == "--display-port-base") {
      display_base = std::lround(number(value(), "--display-port-base"));
    } else if (flag == "--control-local-port-base") {
      control_local_base = std::lround(number(value(), "--control-local-port-base"));
    } else if (flag == "--control-remote-port-base") {
      control_remote_base = std::lround(number(value(), "--control-remote-port-base"));
    } else if (flag == "--origin-lat") {
      origin.latitude_deg = number(value(), "--origin-lat");
    } else if (flag == "--origin-lon") {
      origin.longitude_deg = number(value(), "--origin-lon");
    } else if (flag == "--origin-alt") {
      origin.altitude_m = number(value(), "--origin-alt");
    } else {
      usage(argv[0], "unknown option");
    }
  }
  if (world_path.empty()) usage(argv[0], "--world is required");

  Building building;
  try {
    building = read_building(world_path);
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }

  // The world file says how many robots each level carries. --count overrides
  // it, spreading the robots over the levels in the same proportion.
  std::vector<int> per_level = building.robots_per_level;
  if (per_level.size() != building.levels.size()) {
    per_level.assign(building.levels.size(), 0);
  }
  if (count > 0) {
    const auto levels = static_cast<long>(building.levels.size());
    for (long index = 0; index < levels; ++index) {
      per_level[static_cast<std::size_t>(index)] =
        static_cast<int>(count / levels + (index < count % levels ? 1 : 0));
    }
  }
  const long total = std::accumulate(per_level.begin(), per_level.end(), 0L,
    [](const long sum, const int value) {return sum + value;});
  if (total < 1 || total > 255) {
    usage(argv[0], "the fleet must hold between 1 and 255 robots");
  }
  // Every port this fleet will occupy, checked before a single socket is
  // opened. A silent overlap between two ranges is a whole fleet reporting one
  // robot's position, which is far harder to recognise than a refusal here.
  const std::array<long, 3> bases{display_base, control_local_base, control_remote_base};
  for (const auto base : bases) {
    if (base < 1 || base + total > 65536) {
      usage(argv[0], "a port base plus the fleet size falls outside the port range");
    }
  }
  for (std::size_t first = 0; first < bases.size(); ++first) {
    for (std::size_t second = first + 1; second < bases.size(); ++second) {
      if (std::labs(bases[first] - bases[second]) < total) {
        usage(argv[0], "two port ranges overlap at this fleet size");
      }
    }
  }

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  std::vector<std::unique_ptr<Robot>> fleet;
  fleet.reserve(static_cast<std::size_t>(total));
  try {
    std::size_t next_index = 0;
    for (std::size_t level = 0; level < building.levels.size(); ++level) {
      // Start them on the rooms the patrol will actually use, dealt round the
      // list so a level's robots begin spread over its balcony rather than
      // bunched in whichever segment came first.
      std::vector<std::size_t> rooms;
      for (std::size_t room = 0; room < building.rooms.size(); ++room) {
        const Room & candidate = building.rooms[room];
        if (candidate.level != building.levels[level].index) continue;
        if (building.patrol_circulation_only && !candidate.circulation) continue;
        rooms.push_back(room);
      }
      if (rooms.empty()) continue;
      for (int robot = 0; robot < per_level[level]; ++robot) {
        // Deal the robots round the rooms so no room starts crowded.
        const auto room = rooms[static_cast<std::size_t>(robot) % rooms.size()];
        const auto offset = static_cast<long>(next_index);
        fleet.push_back(std::make_unique<Robot>(
          next_index, building, room, origin,
          static_cast<std::uint16_t>(display_base + offset),
          static_cast<std::uint16_t>(control_local_base + offset),
          static_cast<std::uint16_t>(control_remote_base + offset),
          static_cast<std::uint32_t>(0x9E3779B9u * (next_index + 1))));
        ++next_index;
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << "\n";
    return 1;
  }

  std::cout << "factory fleet: " << total << " robots in '" << world_path << "' ("
            << building.rooms.size() << " rooms over " << building.levels.size()
            << " levels), display " << display_base << "-" << (display_base + total - 1)
            << ", control " << control_local_base << "-" << (control_local_base + total - 1)
            << " -> " << control_remote_base << "-" << (control_remote_base + total - 1) << "\n"
            << std::flush;

  auto previous = Clock::now();
  auto next = previous;
  while (!g_stopping.load()) {
    next += kControlPeriod;
    const auto now = Clock::now();
    const float dt = std::chrono::duration<float>(now - previous).count();
    previous = now;
    for (auto & robot : fleet) robot->tick(now, dt);
    if (next > Clock::now()) {
      std::this_thread::sleep_until(next);
    } else {
      next = Clock::now();
    }
  }

  std::cout << "factory fleet stopping\n" << std::flush;
  fleet.clear();
  return 0;
}
