#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
constexpr std::size_t kMaxQueueBytes = 1024 * 1024;
constexpr std::uint16_t kRelayPortBase = 10000;
constexpr auto kDestinationRetryInterval = std::chrono::milliseconds(100);
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }

struct Queue {
  std::vector<std::uint8_t> bytes;
  std::size_t head{};
  std::size_t size() const { return bytes.size() - head; }
  const std::uint8_t * data() const { return bytes.data() + head; }
  void append(const std::uint8_t * data, const std::size_t count) { bytes.insert(bytes.end(), data, data + count); }
  void consume(const std::size_t count)
  {
    head += count;
    if (head == bytes.size()) { bytes.clear(); head = 0; }
    else if (head >= 65536 && head * 2 >= bytes.size()) { bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(head)); head = 0; }
  }
  void clear() { bytes.clear(); head = 0; }
};

struct Link {
  std::string source;
  std::string destination;
  std::uint16_t destination_port{};
  int listener{-1};
  int source_fd{-1};
  int destination_fd{-1};
  Queue to_destination;
  Queue to_source;
  Clock::time_point next_destination_attempt{};
  double destination_budget{};
  double source_budget{};
  std::uint64_t destination_bytes{};
  std::uint64_t source_bytes{};
  std::uint64_t sampled_destination_bytes{};
  std::uint64_t sampled_source_bytes{};
  double destination_bps{};
  double source_bps{};
};

struct Arguments {
  int vehicles{};
  int mesh_peers{};
  int operator_peers{};
  std::uint64_t capacity_bps{};
  std::string metrics;
};

[[noreturn]] void usage(const char * program)
{
  std::cerr << "usage: " << program << " --vehicles COUNT --mesh-peers COUNT --operator-peers COUNT"
            << " --capacity-kbps KBIT_PER_SECOND --metrics PATH\n";
  std::exit(2);
}

int integer(const char * text)
{
  char * end = nullptr;
  errno = 0;
  const auto value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || errno == ERANGE || value <= 0 || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument("expected a positive integer");
  }
  return static_cast<int>(value);
}

Arguments arguments(const int argc, char ** argv)
{
  if (argc != 11) usage(argv[0]);
  Arguments result;
  for (int index = 1; index < argc; index += 2) {
    const std::string_view option(argv[index]);
    if (option == "--vehicles") result.vehicles = integer(argv[index + 1]);
    else if (option == "--mesh-peers") result.mesh_peers = integer(argv[index + 1]);
    else if (option == "--operator-peers") result.operator_peers = integer(argv[index + 1]);
    else if (option == "--capacity-kbps") result.capacity_bps = static_cast<std::uint64_t>(integer(argv[index + 1])) * 1000;
    else if (option == "--metrics") result.metrics = argv[index + 1];
    else usage(argv[0]);
  }
  if (result.vehicles < 2 || result.mesh_peers >= result.vehicles || result.operator_peers > result.vehicles ||
    result.capacity_bps == 0 || result.metrics.empty()) usage(argv[0]);
  const auto max_port = static_cast<std::uint32_t>(kRelayPortBase) +
    static_cast<std::uint32_t>(result.vehicles) * static_cast<std::uint32_t>(result.vehicles + 1) + result.vehicles - 1;
  if (max_port > std::numeric_limits<std::uint16_t>::max()) throw std::invalid_argument("fleet is too large for relay ports");
  return result;
}

void close_fd(int & fd) { if (fd >= 0) close(fd); fd = -1; }
void nonblocking(const int fd)
{
  const auto flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) throw std::runtime_error("could not set non-blocking socket");
}

int listener(const std::uint16_t port)
{
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) throw std::runtime_error("could not create listener");
  int reuse = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) { close(fd); throw std::runtime_error("could not configure listener"); }
  sockaddr_in address{};
  address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(port);
  if (bind(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0 || listen(fd, 4) < 0) {
    close(fd); throw std::runtime_error("could not bind relay port " + std::to_string(port));
  }
  nonblocking(fd); return fd;
}

int destination_connection(const std::uint16_t port)
{
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) return -1;
  sockaddr_in address{};
  address.sin_family = AF_INET; address.sin_addr.s_addr = htonl(INADDR_LOOPBACK); address.sin_port = htons(port);
  if (connect(fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) { close(fd); return -1; }
  try { nonblocking(fd); } catch (...) { close(fd); return -1; }
  return fd;
}

void disconnect_destination(Link & link, const Clock::time_point next_attempt)
{
  close_fd(link.destination_fd);
  link.to_destination.clear(); link.to_source.clear(); link.destination_budget = 0.0; link.source_budget = 0.0;
  link.next_destination_attempt = next_attempt;
}

void disconnect(Link & link)
{
  close_fd(link.source_fd);
  disconnect_destination(link, Clock::time_point{});
}

void connect_destination(Link & link, const Clock::time_point now)
{
  if (link.source_fd < 0 || link.destination_fd >= 0 || now < link.next_destination_attempt) return;
  link.destination_fd = destination_connection(link.destination_port);
  if (link.destination_fd < 0) link.next_destination_attempt = now + kDestinationRetryInterval;
}

bool read_into(const int fd, Queue & queue)
{
  std::array<std::uint8_t, 16384> buffer{};
  for (;;) {
    const auto received = recv(fd, buffer.data(), std::min(buffer.size(), kMaxQueueBytes - queue.size()), 0);
    if (received > 0) { queue.append(buffer.data(), static_cast<std::size_t>(received)); continue; }
    if (received == 0) return false;
    if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
    if (errno == EINTR) continue;
    return false;
  }
}

bool write_from(const int fd, Queue & queue, double & budget, std::uint64_t & total)
{
  while (queue.size() && budget >= 1.0) {
    const auto sent = send(fd, queue.data(), std::min<std::size_t>(queue.size(), static_cast<std::size_t>(budget)), MSG_NOSIGNAL);
    if (sent > 0) { queue.consume(static_cast<std::size_t>(sent)); budget -= sent; total += static_cast<std::uint64_t>(sent); continue; }
    if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
    if (sent < 0 && errno == EINTR) continue;
    return false;
  }
  return true;
}

std::string node_name(const int node, const int vehicles) { return node == vehicles ? "operator" : "px4_" + std::to_string(node); }
std::uint16_t edge_port(const int node, const int vehicles) { return static_cast<std::uint16_t>(node == vehicles ? 9200 : 9100 + node); }
std::uint16_t relay_port(const int source, const int destination, const int vehicles)
{
  return static_cast<std::uint16_t>(kRelayPortBase + source * (vehicles + 1) + destination);
}
std::int64_t unix_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// `observed_unix_ms` is passed in rather than read here, because it has to mark
// when the sample window closed, not when this function got around to writing.
// Reading the clock at write time made the published interval jitter by however
// long the surrounding poll iteration took, on both sides: measured on a live
// twenty-node fleet, a loop intending one second published at anywhere from
// 631 ms to 1588 ms apart. Consumers judge staleness from this field, so that
// jitter became their problem.
void write_metrics(
  const Arguments & args, const std::vector<Link> & links, const std::int64_t observed_unix_ms)
{
  nlohmann::json document{{"schema", "ditto.sim_network.v2"}, {"observed_unix_ms", observed_unix_ms},
    {"capacity_bps", args.capacity_bps}, {"links", nlohmann::json::array()}};
  for (const auto & link : links) {
    const double capacity = static_cast<double>(args.capacity_bps);
    document["links"].push_back({{"source_node_id", link.source}, {"destination_node_id", link.destination},
      {"connected", link.source_fd >= 0 && link.destination_fd >= 0},
      {"tx_bps", link.source_bps}, {"rx_bps", link.destination_bps},
      {"tx_utilization_percent", 100.0 * link.source_bps / capacity},
      {"rx_utilization_percent", 100.0 * link.destination_bps / capacity}});
  }
  const auto temporary = args.metrics + ".new";
  { std::ofstream output(temporary, std::ios::trunc); output << document.dump(); }
  if (rename(temporary.c_str(), args.metrics.c_str()) != 0) std::cerr << "warning: could not publish network metrics: " << std::strerror(errno) << '\n';
}

void add_link(std::vector<Link> & links, const int source, const int destination, const Arguments & args)
{
  Link link;
  link.source = node_name(source, args.vehicles); link.destination = node_name(destination, args.vehicles);
  link.destination_port = edge_port(destination, args.vehicles);
  link.listener = listener(relay_port(source, destination, args.vehicles));
  links.push_back(std::move(link));
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto args = arguments(argc, argv);
    std::vector<Link> links;
    links.reserve(args.vehicles * args.mesh_peers + args.operator_peers);
    for (int source = 0; source < args.vehicles; ++source) {
      for (int offset = 1; offset <= args.mesh_peers; ++offset) add_link(links, source, (source + offset) % args.vehicles, args);
    }
    for (int peer = 0; peer < args.operator_peers; ++peer) add_link(links, args.vehicles, peer * args.vehicles / args.operator_peers, args);
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    auto previous = Clock::now(); auto sampled = previous;
    while (!stopping) {
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - previous).count(); previous = now;
      const double burst = std::max(16384.0, static_cast<double>(args.capacity_bps) / 8.0 * 0.05);
      for (auto & link : links) {
        link.destination_budget = std::min(burst, link.destination_budget + elapsed * args.capacity_bps / 8.0);
        link.source_budget = std::min(burst, link.source_budget + elapsed * args.capacity_bps / 8.0);
      }
      std::vector<pollfd> descriptors;
      std::vector<std::pair<std::size_t, int>> owners;
      descriptors.reserve(links.size() * 3); owners.reserve(links.size() * 3);
      for (std::size_t index = 0; index < links.size(); ++index) {
        auto & link = links[index];
        connect_destination(link, now);
        descriptors.push_back({link.listener, POLLIN, 0}); owners.push_back({index, 0});
        if (link.source_fd < 0) continue;
        short source_events = 0, destination_events = 0;
        if (link.destination_fd >= 0) {
          if (link.to_destination.size() < kMaxQueueBytes) source_events |= POLLIN;
          if (link.to_source.size() && link.source_budget >= 1.0) source_events |= POLLOUT;
          if (link.to_source.size() < kMaxQueueBytes) destination_events |= POLLIN;
          if (link.to_destination.size() && link.destination_budget >= 1.0) destination_events |= POLLOUT;
        }
        descriptors.push_back({link.source_fd, source_events, 0}); owners.push_back({index, 1});
        if (link.destination_fd >= 0) {
          descriptors.push_back({link.destination_fd, destination_events, 0}); owners.push_back({index, 2});
        }
      }
      if (poll(descriptors.data(), descriptors.size(), 20) < 0 && errno != EINTR) throw std::runtime_error("network poll failed");
      for (std::size_t descriptor = 0; descriptor < descriptors.size(); ++descriptor) {
        const auto events = descriptors[descriptor].revents;
        if (!events) continue;
        auto & link = links[owners[descriptor].first];
        const int side = owners[descriptor].second;
        if (side == 0) {
          const int accepted = accept(link.listener, nullptr, nullptr);
          if (accepted < 0) continue;
          disconnect(link); link.source_fd = accepted; nonblocking(link.source_fd);
          link.next_destination_attempt = now;
          continue;
        }
        const bool source_side = side == 1;
        if (events & (POLLERR | POLLHUP | POLLNVAL)) {
          if (source_side) disconnect(link);
          else disconnect_destination(link, now + kDestinationRetryInterval);
          continue;
        }
        const int fd = source_side ? link.source_fd : link.destination_fd;
        if ((events & POLLIN) && !read_into(fd, source_side ? link.to_destination : link.to_source)) {
          if (source_side) disconnect(link);
          else disconnect_destination(link, now + kDestinationRetryInterval);
          continue;
        }
        if ((events & POLLOUT) && !write_from(fd, source_side ? link.to_source : link.to_destination,
          source_side ? link.source_budget : link.destination_budget,
          source_side ? link.source_bytes : link.destination_bytes)) {
          if (source_side) disconnect(link);
          else disconnect_destination(link, now + kDestinationRetryInterval);
        }
      }
      if (now - sampled >= std::chrono::seconds(1)) {
        const auto observed = unix_ms();
        const double seconds = std::chrono::duration<double>(now - sampled).count();
        for (auto & link : links) {
          link.destination_bps = 8.0 * (link.destination_bytes - link.sampled_destination_bytes) / seconds;
          link.source_bps = 8.0 * (link.source_bytes - link.sampled_source_bytes) / seconds;
          link.sampled_destination_bytes = link.destination_bytes; link.sampled_source_bytes = link.source_bytes;
        }
        write_metrics(args, links, observed); sampled = now;
      }
    }
    for (auto & link : links) { disconnect(link); close_fd(link.listener); }
  } catch (const std::exception & exception) {
    std::cerr << "network relay error: " << exception.what() << '\n';
    return 1;
  }
}
