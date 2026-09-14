#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <netinet/in.h>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <mavlink/common/mavlink.h>

namespace
{

struct Source {
  Source(std::string name, const std::uint16_t port) : name(std::move(name))
  {
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
      throw std::runtime_error("could not bind PX4 telemetry socket");
    }
  }

  Source(const Source &) = delete;
  Source & operator=(const Source &) = delete;
  Source(Source && other) noexcept
  : name(std::move(other.name)), parser_buffer(other.parser_buffer), parser_status(other.parser_status),
    position_ready(other.position_ready), attitude_ready(other.attitude_ready),
    datagrams(other.datagrams), frames(other.frames),
    fd(std::exchange(other.fd, -1)) {}
  ~Source() { if (fd >= 0) close(fd); }

  std::string name;
  mavlink_message_t parser_buffer{};
  mavlink_status_t parser_status{};
  bool position_ready{};
  bool attitude_ready{};
  std::size_t datagrams{};
  std::size_t frames{};
  bool ready() const { return position_ready && attitude_ready; }
  int fd{-1};
};

void receive(Source & source)
{
  std::array<std::uint8_t, MAVLINK_MAX_PACKET_LEN> packet{};
  for (;;) {
    const auto received = recv(source.fd, packet.data(), packet.size(), 0);
    if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
    if (received < 0) throw std::runtime_error(source.name + ": PX4 telemetry receive failed");
    ++source.datagrams;
    for (std::size_t offset = 0; offset < static_cast<std::size_t>(received); ++offset) {
      mavlink_message_t message{};
      if (mavlink_frame_char_buffer(&source.parser_buffer, &source.parser_status, packet[offset],
        &message, nullptr) == MAVLINK_FRAMING_OK) {
        ++source.frames;
        if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) source.position_ready = true;
        if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) source.attitude_ready = true;
      }
      if (source.ready()) return;
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  int timeout_seconds = 60;
  std::vector<Source> sources;
  try {
    for (int index = 1; index < argc; ++index) {
      const std::string argument(argv[index]);
      if (argument == "--timeout" && ++index < argc) {
        timeout_seconds = std::stoi(argv[index]);
      } else if (argument == "--vehicle" && ++index < argc) {
        std::string name(argv[index]);
        if (++index == argc || std::string(argv[index]) != "--port" || ++index == argc) {
          throw std::invalid_argument("expected --vehicle ID --port PX4_SIH_PORT");
        }
        const auto port = std::stoul(argv[index]);
        if (port == 0 || port > 65535) throw std::invalid_argument("PX4 SIH port out of range");
        sources.emplace_back(std::move(name), static_cast<std::uint16_t>(port));
      } else {
        throw std::invalid_argument("usage: px4_telemetry_ready [--timeout SECONDS] --vehicle ID --port PX4_SIH_PORT [...]");
      }
    }
    if (timeout_seconds <= 0 || sources.empty()) throw std::invalid_argument("invalid telemetry readiness arguments");
    std::vector<pollfd> poll_fds;
    poll_fds.reserve(sources.size());
    for (const auto & source : sources) poll_fds.push_back({source.fd, POLLIN, 0});
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_seconds);
    auto next_status = std::chrono::steady_clock::now();
    std::size_t previous_ready = sources.size() + 1;
    for (;;) {
      const int ready = poll(poll_fds.data(), static_cast<nfds_t>(poll_fds.size()), 200);
      if (ready < 0 && errno != EINTR) throw std::runtime_error("PX4 telemetry poll failed");
      for (std::size_t index = 0; index < sources.size(); ++index) {
        if (!sources[index].ready() && ready > 0 && poll_fds[index].revents != 0) receive(sources[index]);
        if (sources[index].ready()) poll_fds[index].events = 0;
        poll_fds[index].revents = 0;
      }
      std::size_t count{};
      for (const auto & source : sources) count += source.ready();
      const auto now = std::chrono::steady_clock::now();
      if (count != previous_ready || now >= next_status) {
        std::cerr << "[PX4] " << count << '/' << sources.size() << " direct telemetry ready";
        if (count != sources.size()) {
          for (const auto & source : sources) {
            if (!source.ready()) { std::cerr << "; waiting for " << source.name; break; }
          }
        }
        std::cerr << '\n';
        previous_ready = count;
        next_status = now + std::chrono::seconds(5);
      }
      if (count == sources.size()) {
        std::cout << count << '/' << sources.size() << " direct PX4 telemetry ready\n";
        return 0;
      }
      if (now >= deadline) {
        std::cerr << count << '/' << sources.size() << " direct PX4 telemetry ready; missing:";
        for (const auto & source : sources) {
          if (source.ready()) continue;
          std::cerr << ' ' << source.name << '(';
          if (!source.position_ready) std::cerr << "position";
          if (!source.position_ready && !source.attitude_ready) std::cerr << ',';
          if (!source.attitude_ready) std::cerr << "attitude";
          std::cerr << "; datagrams=" << source.datagrams << ", frames=" << source.frames << ')';
        }
        std::cerr << '\n';
        return 1;
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << '\n';
    return 2;
  }
}
