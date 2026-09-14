#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "ditto/edge/client.hpp"

namespace
{

constexpr char kInsert[] =
  "INSERT INTO fleet_commands DOCUMENTS (:doc) ON ID CONFLICT DO UPDATE_LOCAL_DIFF";
constexpr char kFleetCommandsQuery[] =
  "SELECT * FROM fleet_commands WHERE _id = 'fleet-current'";
constexpr char kStateQuery[] = "SELECT * FROM vehicle_state";
constexpr char kReceiptQuery[] = "SELECT * FROM fleet_command_receipts";
constexpr auto kReadinessTimeout = std::chrono::seconds(180);
constexpr auto kCommandTtl = std::chrono::seconds(240);
constexpr std::int64_t kFreshStateMs = 5000;

std::int64_t unix_time_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

[[noreturn]] void usage(const char * program, const char * error = nullptr)
{
  if (error != nullptr) {
    std::cerr << "error: " << error << '\n';
  }
  std::cerr << "usage:\n"
            << "  " << program << " --socket PATH arm|disarm VEHICLE\n"
            << "  " << program << " --socket PATH goto VEHICLE NORTH EAST ALTITUDE [YAW]\n"
            << "  " << program
            << " --socket PATH orbit VEHICLE NORTH EAST ALTITUDE RADIUS SPEED [CLOCKWISE] [ANGLE]\n"
            << "  " << program << " --socket PATH status VEHICLE\n"
            << "  " << program << " --socket PATH ready VEHICLE_COUNT\n"
            << "  " << program << " --socket PATH verify VEHICLE_COUNT\n";
  std::exit(2);
}

float number(const char * text, const char * name)
{
  char * end = nullptr;
  errno = 0;
  const float value = std::strtof(text, &end);
  if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(value)) {
    std::string error(name);
    error += " must be a finite number";
    usage("ditto_fleet_command", error.c_str());
  }
  return value;
}

bool boolean(const char * text)
{
  if (std::string_view(text) == "true") {
    return true;
  }
  if (std::string_view(text) == "false") {
    return false;
  }
  usage("ditto_fleet_command", "CLOCKWISE must be true or false");
}

std::string command_id(const std::string & vehicle, const std::int64_t now)
{
  static std::mt19937_64 random(std::random_device{}());
  return vehicle + '-' + std::to_string(now) + '-' + std::to_string(random());
}

void execute(
  ditto::edge::Client & client, std::string_view statement, const nlohmann::json & arguments,
  ditto::edge::StoreResult & result)
{
  grpc::Status status;
  for (int attempt = 0; attempt != 3; ++attempt) {
    status = client.execute(statement, arguments.dump(), result);
    if (status.ok() || status.error_code() != grpc::StatusCode::DEADLINE_EXCEEDED || attempt == 2) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (!status.ok()) {
    std::cerr << "error: " << status.error_code() << ": " << status.error_message() << '\n';
    std::exit(1);
  }
}

void show_status(ditto::edge::Client & client, const std::string & vehicle)
{
  ditto::edge::StoreResult state;
  ditto::edge::StoreResult commands;
  ditto::edge::StoreResult receipts;
  const nlohmann::json arguments{{"vehicle_id", vehicle}};
  execute(client, "SELECT * FROM vehicle_state WHERE vehicle_id = :vehicle_id", arguments, state);
  execute(client, kFleetCommandsQuery, nlohmann::json::object(), commands);
  execute(
    client, "SELECT * FROM fleet_command_receipts WHERE target_vehicle_id = :vehicle_id", arguments,
    receipts);
  std::cout << nlohmann::json{
    {"vehicle_state", nlohmann::json::parse(state.documents_json)},
    {"fleet_commands", nlohmann::json::parse(commands.documents_json)},
    {"fleet_command_receipts", nlohmann::json::parse(receipts.documents_json)},
  }.dump(2) << '\n';
}

nlohmann::json fleet_commands(ditto::edge::Client & client)
{
  ditto::edge::StoreResult result;
  execute(client, kFleetCommandsQuery, nlohmann::json::object(), result);
  const auto documents = nlohmann::json::parse(result.documents_json);
  if (documents.empty()) {
    return {{"_id", "fleet-current"}, {"schema", "ditto.fleet_commands.v1"},
      {"commands", nlohmann::json::object()}};
  }
  if (!documents.is_array() || documents.size() != 1 || !documents.front().is_object() ||
    documents.front().value("_id", "") != "fleet-current") {
    throw std::runtime_error("fleet_commands contains an invalid fleet-current document");
  }
  auto document = documents.front();
  if (!document.contains("commands") || !document["commands"].is_object()) {
    document["commands"] = nlohmann::json::object();
  }
  return document;
}

int show_ready(ditto::edge::Client & client, const std::size_t expected)
{
  ditto::edge::StoreResult state;
  execute(client, kStateQuery, nlohmann::json::object(), state);
  std::size_t ready{};
  for (const auto & document : nlohmann::json::parse(state.documents_json)) {
    const auto local = document.find("local");
    if (document.value("vehicle_id", "").empty() || local == document.end() ||
      !local->value("position_valid", false)) continue;
    ++ready;
  }
  std::cout << ready << '/' << expected << " vehicles ready\n";
  return ready >= expected ? 0 : 1;
}

struct Position {float north; float east; float altitude;};

std::map<std::string, Position> fresh_positions(
  const std::string & operator_socket, const std::size_t expected, const std::int64_t now,
  std::string * error = nullptr)
{
  std::map<std::string, Position> positions;
  if (error) error->clear();
  const auto nodes = std::filesystem::path(operator_socket).parent_path().parent_path();
  for (std::size_t index = 0; index < expected; ++index) {
    const auto id = "px4_" + std::to_string(index);
    ditto::edge::Client client((nodes / id / "edge.sock").string(), std::string{}, std::chrono::seconds(5));
    ditto::edge::StoreResult result;
    const auto status = client.execute(
      "SELECT * FROM vehicle_state WHERE vehicle_id = :vehicle_id", nlohmann::json{{"vehicle_id", id}}.dump(), result);
    if (!status.ok()) {
      if (error && error->empty()) *error = id + " state query failed: " + status.error_message();
      continue;
    }
    const auto documents = nlohmann::json::parse(result.documents_json);
    if (documents.size() != 1U) {
      if (error && error->empty()) *error = id + " state document count=" + std::to_string(documents.size());
      continue;
    }
    const auto & state = documents.front();
    const auto local = state.find("local");
    if (local == state.end() || !local->value("position_valid", false) ||
      state.value("published_unix_ms", 0LL) < now - kFreshStateMs)
    {
      if (error && error->empty()) *error = id + " state stale or invalid";
      continue;
    }
    positions.emplace(id, Position{local->value("x_m", 0.0F), local->value("y_m", 0.0F),
      -local->value("z_m", 0.0F)});
  }
  return positions;
}

void verify_fleet(
  ditto::edge::Client & client, const std::string & operator_socket, const std::size_t expected)
{
  const auto deadline = std::chrono::steady_clock::now() + kReadinessTimeout;
  std::map<std::string, Position> initial;
  std::string state_error;
  auto next_state_status = std::chrono::steady_clock::now();
  while (initial.size() < expected) {
    for (auto && [vehicle, position] : fresh_positions(operator_socket, expected, unix_time_ms(), &state_error)) {
      initial[vehicle] = position;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("fleet state is not fresh: " + state_error);
    }
    if (initial.size() < expected) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_state_status) {
        std::cout << "[Fleet] " << initial.size() << '/' << expected << " fresh local states";
        if (!state_error.empty()) std::cout << "; waiting for " << state_error;
        std::cout << '\n' << std::flush;
        next_state_status = now + std::chrono::seconds(5);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

  const auto now = unix_time_ms();
  auto document = fleet_commands(client);
  std::map<std::string, std::string> ids;
  for (std::size_t index = 0; index < expected; ++index) {
    const auto vehicle = "px4_" + std::to_string(index);
    const auto current = document["commands"].find(vehicle);
    const auto sequence = current == document["commands"].end() ? 1ULL :
      current->value("sequence", 0ULL) + 1ULL;
    const auto & position = initial.at(vehicle);
    const auto id = command_id(vehicle, now);
    ids.emplace(vehicle, id);
    document["commands"][vehicle] = {{"command_id", id}, {"schema", "ditto.fleet_command.v1"},
      {"target_vehicle_id", vehicle}, {"created_unix_ms", now},
      {"expires_unix_ms", now + std::chrono::duration_cast<std::chrono::milliseconds>(kCommandTtl).count()},
      {"sequence", sequence}, {"kind", "goto_local"},
      {"north_m", position.north + (index % 2U == 0U ? 3.0F : -3.0F)},
      {"east_m", position.east}, {"altitude_m", std::max(2.0F, position.altitude + 1.0F)}};
  }
  document["updated_unix_ms"] = now;
  ditto::edge::StoreResult ignored;
  execute(client, kInsert, nlohmann::json{{"doc", document}}, ignored);

  auto next_command_status = std::chrono::steady_clock::now();
  std::size_t previous_accepted = expected + 1;
  std::size_t previous_moved = expected + 1;
  std::map<std::string, bool> moved;
  while (std::chrono::steady_clock::now() < deadline) {
    ditto::edge::StoreResult receipts;
    execute(client, kReceiptQuery, nlohmann::json::object(), receipts);
    std::map<std::string, bool> accepted;
    for (const auto & receipt : nlohmann::json::parse(receipts.documents_json)) {
      const auto vehicle = receipt.value("target_vehicle_id", "");
      const auto id = receipt.value("command_id", "");
      if (ids.count(vehicle) && ids.at(vehicle) == id && receipt.value("status", "") == "accepted") {
        accepted[vehicle] = true;
      }
    }
    const auto current = fresh_positions(operator_socket, expected, unix_time_ms());
    std::size_t accepted_count{};
    for (const auto &[vehicle, position] : initial) {
      const auto next = current.find(vehicle);
      if (next != current.end() &&
        std::hypot(next->second.north - position.north, next->second.east - position.east) >= 1.0F) moved[vehicle] = true;
      accepted_count += accepted[vehicle];
    }
    const auto moved_count = static_cast<std::size_t>(std::count_if(
      moved.begin(), moved.end(), [](const auto & item) { return item.second; }));
    const bool complete = accepted_count == expected && moved_count == expected;
    if (complete) {
      std::cout << expected << '/' << expected << " vehicles command-ready and moving\n";
      return;
    }
    const auto now_status = std::chrono::steady_clock::now();
    if (accepted_count != previous_accepted || moved_count != previous_moved || now_status >= next_command_status) {
      std::cout << "[Fleet] " << accepted_count << '/' << expected << " command receipts; " << moved_count
                << '/' << expected << " moving\n" << std::flush;
      previous_accepted = accepted_count;
      previous_moved = moved_count;
      next_command_status = now_status + std::chrono::seconds(5);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  ditto::edge::StoreResult receipts;
  execute(client, kReceiptQuery, nlohmann::json::object(), receipts);
  std::map<std::string, std::string> statuses;
  for (const auto & receipt : nlohmann::json::parse(receipts.documents_json)) {
    const auto vehicle = receipt.value("target_vehicle_id", "");
    if (ids.count(vehicle) && ids.at(vehicle) == receipt.value("command_id", "")) {
      statuses[vehicle] = receipt.value("status", "unknown") +
        (receipt.contains("error") ? ":" + receipt.value("error", "unknown") : "");
    }
  }
  std::string detail;
  for (const auto &[vehicle, position] : initial) {
    const auto status = statuses.find(vehicle);
    const auto movement = moved[vehicle];
    if (status != statuses.end() && status->second == "accepted" && movement) continue;
    if (!detail.empty()) detail += ", ";
    detail += vehicle + "(receipt=" + (status != statuses.end() ? status->second : "missing") +
      ", movement=" + (movement ? "ok" : "missing") + ')';
  }
  throw std::runtime_error("fleet command or movement verification timed out: " + detail);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 5 || std::string_view(argv[1]) != "--socket") {
    usage(argv[0]);
  }
  const std::string action(argv[3]);
  const std::string vehicle(argv[4]);
  if (vehicle.empty()) {
    usage(argv[0], "VEHICLE is required");
  }

  try {
    // A fleet write remains local, but the debug Edge Server can briefly be
    // backlogged while 20 simulators publish state.  Bound a CLI operation
    // without treating that transient queueing as a failed command.
    ditto::edge::Client client(argv[2], std::string{}, std::chrono::seconds(30));
    if (action == "status") {
      if (argc != 5) {
        usage(argv[0]);
      }
      show_status(client, vehicle);
      return 0;
    }
    if (action == "ready") {
      if (argc != 5) {
        usage(argv[0]);
      }
      char * end = nullptr;
      errno = 0;
      const auto expected = std::strtoul(vehicle.c_str(), &end, 10);
      if (end == vehicle.c_str() || *end != '\0' || errno == ERANGE || expected == 0) {
        usage(argv[0], "VEHICLE_COUNT must be a positive integer");
      }
      return show_ready(client, expected);
    }
    if (action == "verify") {
      if (argc != 5) usage(argv[0]);
      char * end = nullptr;
      errno = 0;
      const auto expected = std::strtoul(vehicle.c_str(), &end, 10);
      if (end == vehicle.c_str() || *end != '\0' || errno == ERANGE || expected == 0) {
        usage(argv[0], "VEHICLE_COUNT must be a positive integer");
      }
      verify_fleet(client, argv[2], expected);
      return 0;
    }

    const auto now = unix_time_ms();
    nlohmann::json command{
      {"command_id", command_id(vehicle, now)},
      {"schema", "ditto.fleet_command.v1"},
      {"target_vehicle_id", vehicle},
      {"created_unix_ms", now},
      {"expires_unix_ms", now + 30000},
    };
    if (action == "arm" || action == "disarm") {
      if (argc != 5) {
        usage(argv[0]);
      }
      command["kind"] = "set_armed";
      command["desired_armed"] = action == "arm";
    } else if (action == "goto") {
      if (argc != 8 && argc != 9) {
        usage(argv[0]);
      }
      command["kind"] = "goto_local";
      command["north_m"] = number(argv[5], "NORTH");
      command["east_m"] = number(argv[6], "EAST");
      command["altitude_m"] = number(argv[7], "ALTITUDE");
      if (argc == 9) {
        command["yaw_rad"] = number(argv[8], "YAW");
      }
    } else if (action == "orbit") {
      if (argc < 10 || argc > 12) {
        usage(argv[0]);
      }
      command["kind"] = "orbit_local";
      command["center_north_m"] = number(argv[5], "NORTH");
      command["center_east_m"] = number(argv[6], "EAST");
      command["altitude_m"] = number(argv[7], "ALTITUDE");
      command["radius_m"] = number(argv[8], "RADIUS");
      command["speed_m_s"] = number(argv[9], "SPEED");
      if (argc >= 11) {
        command["clockwise"] = boolean(argv[10]);
      }
      if (argc == 12) {
        command["initial_angle_rad"] = number(argv[11], "ANGLE");
      }
    } else {
      usage(argv[0], "unknown action");
    }

    auto document = fleet_commands(client);
    const auto current = document["commands"].find(vehicle);
    const auto sequence = current == document["commands"].end() ? 1ULL :
      current->value("sequence", 0ULL) + 1ULL;
    command["sequence"] = sequence;
    document["commands"][vehicle] = std::move(command);
    document["updated_unix_ms"] = now;
    ditto::edge::StoreResult result;
    execute(client, kInsert, nlohmann::json{{"doc", document}}, result);
    std::cout << document.at("commands").at(vehicle).at("command_id") << '\n';
  } catch (const std::exception & exception) {
    std::cerr << "error: " << exception.what() << '\n';
    return 1;
  }
}
