// A ground station on the LAN -- UAS Tool on a phone -- reaches each PX4 here.
//
// PX4 speaks MAVLink over UDP only, and a UDP link latches onto the first
// address that writes to it and never lets go (mavlink_receiver.cpp): a phone
// that reconnects from a new port is never answered again. So each PX4's GCS
// link points at this relay's fixed loopback port instead, and ground stations
// connect over TCP, where a reconnect is just a new connection.
//
//   ground station --TCP <port>--> relay --UDP 127.0.0.1:<port>--> PX4 GCS link
//
// A vehicle's TCP listener takes every interface; its UDP socket takes only
// loopback, and is the address PX4 sends to (`mavlink start -o <port>`).
// Several stations may share a vehicle, so bytes bound for PX4 go as whole
// MAVLink frames: two stations' writes must never interleave inside PX4's one
// parser. Bytes from PX4 already arrive as whole frames, one datagram each.

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kMaxClients = 4;           // per vehicle; the oldest makes way for a new one
constexpr std::size_t kMaxBacklog = 256 * 1024;  // telemetry queued for one slow station, bytes

volatile std::sig_atomic_t stopping = 0;

struct Client {
  int fd;
  std::string peer, in, out;
};

struct Link {
  std::string vehicle;
  std::uint16_t port{};
  sockaddr_in px4{};
  int listener{-1};
  int udp{-1};
  std::vector<Client> clients;
};

[[noreturn]] void fail(const char * what)
{
  std::fprintf(stderr, "gcs-relay: %s: %s\n", what, std::strerror(errno));
  std::exit(1);
}

[[noreturn]] void usage(const char * program)
{
  std::fprintf(stderr, "usage: %s --vehicle ID --port PORT --px4-port PX4_GCS_PORT [...]\n", program);
  std::exit(2);
}

std::uint16_t port_number(const char * text, const char * program)
{
  char * end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || value <= 0 || value > 65535) usage(program);
  return static_cast<std::uint16_t>(value);
}

sockaddr_in address(const std::uint32_t host, const std::uint16_t port)
{
  sockaddr_in result{};
  result.sin_family = AF_INET;
  result.sin_addr.s_addr = htonl(host);
  result.sin_port = htons(port);
  return result;
}

void nonblocking(const int fd)
{
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) fail("fcntl");
}

int open_socket(const int type, const std::uint32_t host, const std::uint16_t port)
{
  const int fd = socket(AF_INET, type, 0);
  if (fd < 0) fail("socket");
  const int on = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  const sockaddr_in local = address(host, port);
  if (bind(fd, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) < 0) fail("bind");
  if (type == SOCK_STREAM && listen(fd, 4) < 0) fail("listen");
  nonblocking(fd);
  return fd;
}

void configure_client(const int fd)
{
  nonblocking(fd);
  const int on = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
  setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
  // A phone that leaves Wi-Fi never closes its connection. Keepalive finds it
  // within about half a minute, so it stops holding a client slot.
  setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  const int idle = 10, interval = 5, count = 3;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPALIVE, &idle, sizeof(idle));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
}

/// False when the connection has failed.
bool flush(Client & client)
{
  while (!client.out.empty()) {
    const ssize_t sent = send(client.fd, client.out.data(), client.out.size(), 0);
    if (sent > 0) {
      client.out.erase(0, static_cast<std::size_t>(sent));
      continue;
    }
    return sent < 0 && (errno == EAGAIN || errno == EINTR);
  }
  return true;
}

/// Sends every complete MAVLink frame buffered from a station to PX4, keeping
/// any partial frame. Bytes before a start marker are line noise and dropped.
void forward_frames(Link & link, Client & client)
{
  std::size_t at = 0;
  const auto & in = client.in;
  while (at < in.size()) {
    const auto marker = static_cast<unsigned char>(in[at]);
    if (marker != 0xFD && marker != 0xFE) {
      ++at;
      continue;
    }
    if (in.size() - at < 3) break;
    const std::size_t payload = static_cast<unsigned char>(in[at + 1]);
    // v2: 10 header bytes, 2 CRC, and a 13-byte signature when flagged; v1: 6 + 2.
    const std::size_t length = marker == 0xFD
      ? 12 + payload + ((static_cast<unsigned char>(in[at + 2]) & 0x01) != 0 ? 13 : 0)
      : 8 + payload;
    if (in.size() - at < length) break;
    sendto(link.udp, in.data() + at, length, 0, reinterpret_cast<const sockaddr *>(&link.px4), sizeof(link.px4));
    at += length;
  }
  client.in.erase(0, at);
}

void drop(Link & link, const std::size_t index, const char * why)
{
  std::printf("[gcs] %s: %s %s (%zu left)\n", link.vehicle.c_str(), link.clients[index].peer.c_str(), why,
    link.clients.size() - 1);
  close(link.clients[index].fd);
  link.clients.erase(link.clients.begin() + static_cast<std::ptrdiff_t>(index));
}

void accept_clients(Link & link)
{
  for (;;) {
    sockaddr_in peer{};
    socklen_t size = sizeof(peer);
    const int fd = accept(link.listener, reinterpret_cast<sockaddr *>(&peer), &size);
    if (fd < 0) return;
    configure_client(fd);
    if (link.clients.size() == kMaxClients) drop(link, 0, "replaced by a newer station");
    char text[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &peer.sin_addr, text, sizeof(text));
    link.clients.push_back({fd, std::string(text) + ":" + std::to_string(ntohs(peer.sin_port)), {}, {}});
    std::printf("[gcs] %s: %s connected (%zu total)\n", link.vehicle.c_str(), link.clients.back().peer.c_str(),
      link.clients.size());
  }
}

/// PX4 to every station. A station that cannot keep up misses whole
/// datagrams, never parts of one, so its stream stays frame-aligned.
void receive_from_px4(Link & link)
{
  static char datagram[65536];
  for (;;) {
    sockaddr_in from{};
    socklen_t size = sizeof(from);
    const ssize_t received = recvfrom(link.udp, datagram, sizeof(datagram), 0, reinterpret_cast<sockaddr *>(&from), &size);
    if (received <= 0) return;
    if (from.sin_port != link.px4.sin_port) continue;
    for (std::size_t index = link.clients.size(); index-- > 0;) {
      Client & client = link.clients[index];
      if (client.out.size() + static_cast<std::size_t>(received) > kMaxBacklog) continue;
      client.out.append(datagram, static_cast<std::size_t>(received));
      if (!flush(client)) drop(link, index, "disconnected");
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  std::setvbuf(stdout, nullptr, _IOLBF, 0);  // Process Compose reads a pipe
  std::vector<Link> links;
  for (int index = 1; index < argc; index += 6) {
    if (index + 5 >= argc || std::strcmp(argv[index], "--vehicle") != 0 ||
      std::strcmp(argv[index + 2], "--port") != 0 || std::strcmp(argv[index + 4], "--px4-port") != 0) usage(argv[0]);
    Link link;
    link.vehicle = argv[index + 1];
    link.port = port_number(argv[index + 3], argv[0]);
    link.px4 = address(INADDR_LOOPBACK, port_number(argv[index + 5], argv[0]));
    links.push_back(std::move(link));
  }
  if (links.empty()) usage(argv[0]);
  std::signal(SIGINT, [](int) { stopping = 1; });
  std::signal(SIGTERM, [](int) { stopping = 1; });
  for (auto & link : links) {
    link.listener = open_socket(SOCK_STREAM, INADDR_ANY, link.port);
    link.udp = open_socket(SOCK_DGRAM, INADDR_LOOPBACK, link.port);
    std::printf("[gcs] %s: TCP %u -> PX4 UDP %u\n", link.vehicle.c_str(), link.port, ntohs(link.px4.sin_port));
  }

  std::vector<pollfd> fds;
  while (stopping == 0) {
    fds.clear();
    for (const auto & link : links) {
      fds.push_back({link.listener, POLLIN, 0});
      fds.push_back({link.udp, POLLIN, 0});
      for (const auto & client : link.clients) {
        fds.push_back({client.fd, static_cast<short>(POLLIN | (client.out.empty() ? 0 : POLLOUT)), 0});
      }
    }
    if (poll(fds.data(), static_cast<nfds_t>(fds.size()), 500) <= 0) continue;
    std::size_t at = 0;
    for (auto & link : links) {
      const bool incoming = fds[at++].revents != 0;
      const bool from_px4 = fds[at++].revents != 0;
      // Clients are handled against this poll's snapshot, before the accepts
      // and PX4 traffic below can add or drop any.
      for (std::size_t index = 0, polled = link.clients.size(); polled-- > 0; ++at) {
        const short events = fds[at].revents;
        Client & client = link.clients[index];
        if ((events & POLLIN) != 0) {
          char buffer[4096];
          const ssize_t received = recv(client.fd, buffer, sizeof(buffer), 0);
          if (received == 0 || (received < 0 && errno != EAGAIN && errno != EINTR)) {
            drop(link, index, "disconnected");
            continue;
          }
          if (received > 0) {
            client.in.append(buffer, static_cast<std::size_t>(received));
            forward_frames(link, client);
          }
        } else if ((events & (POLLERR | POLLHUP)) != 0) {
          drop(link, index, "disconnected");
          continue;
        }
        if ((events & POLLOUT) != 0 && !flush(client)) {
          drop(link, index, "disconnected");
          continue;
        }
        ++index;
      }
      if (incoming) accept_clients(link);
      if (from_px4) receive_from_px4(link);
    }
  }
  for (auto & link : links) {
    for (const auto & client : link.clients) close(client.fd);
    close(link.listener);
    close(link.udp);
  }
  return 0;
}
