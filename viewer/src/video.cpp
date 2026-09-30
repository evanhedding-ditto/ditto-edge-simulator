#include "video.hpp"

#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl3.h>
#include <VideoToolbox/VideoToolbox.h>
#include <objc/message.h>
#include <objc/runtime.h>
#include <raylib.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <strings.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <list>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>

namespace sim::video {
namespace {

using Clock = std::chrono::steady_clock;

constexpr int kBitRate = 3'000'000;
constexpr auto kInterval = std::chrono::microseconds(1'000'000 / Feeds::kFps);
constexpr std::size_t kFrameBytes = static_cast<std::size_t>(Feeds::kWidth) * Feeds::kHeight * 4;
constexpr std::size_t kMaxConnections = 8;
constexpr std::size_t kMaxRequest = 16 * 1024;
// About five seconds of video queued for one TCP client. Past it the client
// skips to the next keyframe rather than falling further behind.
constexpr std::size_t kMaxBacklog = 2 * 1024 * 1024;
// RTP payload per packet: each packet fits one 1500-byte Wi-Fi frame.
constexpr std::size_t kMaxPayload = 1400;

struct Stream {
  std::size_t index{};
  std::string name;
  std::uint32_t ssrc{};
  std::uint32_t timestamp_base{};
  // Render thread only.
  RenderTexture2D target{};
  GLuint pixels[2]{};  // pack buffers, written and read on alternate frames
  bool pending[2]{};
  Clock::time_point captured[2]{};
  int next{};
  Clock::time_point due_at{};
  VTCompressionSessionRef session{};
  // Any thread.
  std::atomic<int> watchers{0};
  std::atomic<bool> keyframe_wanted{true};
  // Under Impl::mutex.
  std::string sps, pps;
};

struct Client {
  int fd{-1};
  std::string in, out;
  int stream{-1};
  std::string session, url;
  bool playing{};
  bool keyed{};  // a keyframe has gone out since PLAY; frames before one cannot decode
  bool tcp{true};
  std::uint8_t channel{};
  sockaddr_in6 rtp_to{};
  std::uint16_t sequence{};
  bool dead{};
};

std::string base64(const std::string & in)
{
  static constexpr char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  std::uint32_t bits = 0;
  int count = 0;
  for (const unsigned char c : in) {
    bits = bits << 8 | c;
    for (count += 8; count >= 6; count -= 6) out += table[bits >> (count - 6) & 63];
  }
  if (count > 0) out += table[bits << (6 - count) & 63];
  while (out.size() % 4 != 0) out += '=';
  return out;
}

/// The value of header `name` in a request head that ends in CRLF, or empty.
std::string field(const std::string & head, const char * name)
{
  const std::size_t length = std::strlen(name);
  for (std::size_t line = head.find("\r\n"); line != std::string::npos; line = head.find("\r\n", line + 2)) {
    const std::size_t at = line + 2;
    if (head.size() > at + length && head[at + length] == ':' && strncasecmp(head.data() + at, name, length) == 0) {
      std::size_t start = at + length + 1;
      const std::size_t end = head.find("\r\n", start);
      while (start < end && head[start] == ' ') ++start;
      return head.substr(start, end - start);
    }
  }
  return {};
}

/// `px4_0` from rtsp://host:8554/px4_0/track0.
std::string stream_name(const std::string & url)
{
  std::size_t at = url.find("://");
  at = url.find('/', at == std::string::npos ? 0 : at + 3);
  if (at == std::string::npos) return {};
  const std::size_t end = url.find_first_of("/?", at + 1);
  return url.substr(at + 1, end == std::string::npos ? std::string::npos : end - at - 1);
}

/// For as long as the process lives, keeps macOS from putting it in App Nap,
/// which stretches the render loop's sleeps once the window is hidden and would
/// stall every feed, and from idle sleep, which would stop the simulator.
void stay_awake()
{
  using Send = id (*)(id, SEL);
  const id info = reinterpret_cast<Send>(objc_msgSend)(
    reinterpret_cast<id>(objc_getClass("NSProcessInfo")), sel_registerName("processInfo"));
  const id reason = reinterpret_cast<id (*)(id, SEL, const char *)>(objc_msgSend)(
    reinterpret_cast<id>(objc_getClass("NSString")), sel_registerName("stringWithUTF8String:"),
    "serving vehicle camera feeds");
  // NSActivityUserInitiated | NSActivityLatencyCritical
  const id activity = reinterpret_cast<id (*)(id, SEL, unsigned long long, id)>(objc_msgSend)(
    info, sel_registerName("beginActivityWithOptions:reason:"), 0x00FFFFFFULL | 0xFF00000000ULL, reason);
  reinterpret_cast<Send>(objc_msgSend)(activity, sel_registerName("retain"));
}

void nonblocking(const int fd)
{
  const int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) throw std::runtime_error("fcntl failed");
}

/// Numeric text for an address on a dual-stack socket; IPv4 loses its "::ffff:".
std::string host_text(const in6_addr & address)
{
  char text[INET6_ADDRSTRLEN] = "::";
  inet_ntop(AF_INET6, &address, text, sizeof(text));
  return IN6_IS_ADDR_V4MAPPED(&address) ? text + 7 : text;
}

// Dual-stack, IPv6 as well as IPv4, since a phone hotspot may give the Mac IPv6 alone.
int open_socket(const int type, const std::uint16_t port)
{
  const int fd = socket(AF_INET6, type, 0);
  if (fd < 0) throw std::runtime_error("socket failed");
  const int on = 1, off = 0;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &off, sizeof(off));
  sockaddr_in6 local{};
  local.sin6_family = AF_INET6;
  local.sin6_port = htons(port);
  if (bind(fd, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) < 0 ||
    (type == SOCK_STREAM && listen(fd, 8) < 0))
  {
    close(fd);
    throw std::runtime_error((type == SOCK_STREAM ? "TCP port " : "UDP port ") + std::to_string(port) + " is in use");
  }
  nonblocking(fd);
  return fd;
}

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

bool keyframe(CMSampleBufferRef sample)
{
  const CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sample, false);
  if (attachments == nullptr || CFArrayGetCount(attachments) == 0) return true;
  const auto frame = static_cast<CFDictionaryRef>(CFArrayGetValueAtIndex(attachments, 0));
  const void * not_sync = nullptr;
  return !CFDictionaryGetValueIfPresent(frame, kCMSampleAttachmentKey_NotSync, &not_sync) ||
    !CFBooleanGetValue(static_cast<CFBooleanRef>(not_sync));
}

}  // namespace

struct Feeds::Impl {
  std::vector<std::unique_ptr<Stream>> streams;
  std::uint16_t port{};
  int listener{-1}, rtp{-1}, rtcp{-1}, wake[2]{-1, -1};
  std::mutex mutex;
  std::list<Client> clients;
  std::string packets;  // one access unit as RTP packets, back to back; under `mutex`
  std::vector<std::pair<std::size_t, std::size_t>> spans;
  std::atomic<bool> stopping{false};
  std::thread server;
  const Clock::time_point epoch = Clock::now();
  CFDictionaryRef force_keyframe{};
  std::mt19937 random{std::random_device{}()};

  Impl(const std::uint16_t rtsp_port, const std::vector<std::string> & names) : port(rtsp_port)
  {
    const void * key = kVTEncodeFrameOptionKey_ForceKeyFrame;
    const void * value = kCFBooleanTrue;
    force_keyframe = CFDictionaryCreate(
      nullptr, &key, &value, 1, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    for (const auto & name : names) {
      auto stream = std::make_unique<Stream>();
      stream->index = streams.size();
      stream->name = name;
      stream->ssrc = static_cast<std::uint32_t>(random());
      stream->timestamp_base = static_cast<std::uint32_t>(random());
      streams.push_back(std::move(stream));
    }
  }

  ~Impl()
  {
    stopping = true;
    wake_server();
    if (server.joinable()) server.join();
    for (auto & stream : streams) {
      close_session(*stream);
      if (stream->pixels[0] != 0) glDeleteBuffers(2, stream->pixels);
      if (stream->target.id != 0) UnloadRenderTexture(stream->target);
    }
    for (const auto & client : clients) close(client.fd);
    for (const int fd : {listener, rtp, rtcp, wake[0], wake[1]}) {
      if (fd >= 0) close(fd);
    }
    if (force_keyframe != nullptr) CFRelease(force_keyframe);
  }

  /// Everything that can fail, after construction so the destructor cleans up.
  void start()
  {
    listener = open_socket(SOCK_STREAM, port);
    rtp = open_socket(SOCK_DGRAM, port);
    rtcp = open_socket(SOCK_DGRAM, static_cast<std::uint16_t>(port + 1));
    if (pipe(wake) != 0) throw std::runtime_error("pipe failed");
    nonblocking(wake[0]);
    nonblocking(wake[1]);
    for (auto & stream : streams) {
      stream->target = LoadRenderTexture(Feeds::kWidth, Feeds::kHeight);
      if (stream->target.id == 0) throw std::runtime_error("no render target for " + stream->name);
      glGenBuffers(2, stream->pixels);
      for (const GLuint buffer : stream->pixels) {
        glBindBuffer(GL_PIXEL_PACK_BUFFER, buffer);
        glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(kFrameBytes), nullptr, GL_STREAM_READ);
      }
      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
      if (!open_session(*stream)) throw std::runtime_error("no H.264 encoder");
      // A client's DESCRIBE needs the parameter sets before anyone has watched
      // a frame, so encode one black frame now to learn them.
      encode(*stream, nullptr, epoch);
      VTCompressionSessionCompleteFrames(stream->session, kCMTimeInvalid);
      std::lock_guard<std::mutex> lock(mutex);
      if (stream->sps.empty() || stream->pps.empty()) throw std::runtime_error("encoder gave no parameter sets");
    }
    server = std::thread([this] { serve(); });
  }

  bool open_session(Stream & stream)
  {
    const auto number = [](const int value) { return CFNumberCreate(nullptr, kCFNumberIntType, &value); };
    CFMutableDictionaryRef source = CFDictionaryCreateMutable(
      nullptr, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    const CFDictionaryRef surface = CFDictionaryCreate(
      nullptr, nullptr, nullptr, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    const CFNumberRef format = number(kCVPixelFormatType_32BGRA), width = number(Feeds::kWidth),
      height = number(Feeds::kHeight);
    CFDictionarySetValue(source, kCVPixelBufferPixelFormatTypeKey, format);
    CFDictionarySetValue(source, kCVPixelBufferWidthKey, width);
    CFDictionarySetValue(source, kCVPixelBufferHeightKey, height);
    CFDictionarySetValue(source, kCVPixelBufferIOSurfacePropertiesKey, surface);
    const OSStatus status = VTCompressionSessionCreate(nullptr, Feeds::kWidth, Feeds::kHeight,
      kCMVideoCodecType_H264, nullptr, source, nullptr, encoded, this, &stream.session);
    for (const CFTypeRef value : {static_cast<CFTypeRef>(source), static_cast<CFTypeRef>(surface),
      static_cast<CFTypeRef>(format), static_cast<CFTypeRef>(width), static_cast<CFTypeRef>(height)})
    {
      CFRelease(value);
    }
    if (status != noErr) {
      stream.session = nullptr;
      TraceLog(LOG_ERROR, "video: no H.264 encoder for %s (%d)", stream.name.c_str(), static_cast<int>(status));
      return false;
    }
    const auto set = [&stream](const CFStringRef key, const CFTypeRef value) {
      VTSessionSetProperty(stream.session, key, value);
    };
    const auto set_number = [&set, &number](const CFStringRef key, const int value) {
      const CFNumberRef boxed = number(value);
      set(key, boxed);
      CFRelease(boxed);
    };
    // Baseline without B-frames: what every RTSP player decodes, at one frame
    // of encoder latency. A keyframe each second lets a client join quickly.
    set(kVTCompressionPropertyKey_RealTime, kCFBooleanTrue);
    set(kVTCompressionPropertyKey_ProfileLevel, kVTProfileLevel_H264_ConstrainedBaseline_AutoLevel);
    set(kVTCompressionPropertyKey_AllowFrameReordering, kCFBooleanFalse);
    set_number(kVTCompressionPropertyKey_MaxKeyFrameInterval, Feeds::kFps);
    set_number(kVTCompressionPropertyKey_ExpectedFrameRate, Feeds::kFps);
    set_number(kVTCompressionPropertyKey_AverageBitRate, kBitRate);
    // Say which colours these are. An HD stream that does not is BT.601 to some
    // players and BT.709 to others.
    set(kVTCompressionPropertyKey_ColorPrimaries, kCVImageBufferColorPrimaries_ITU_R_709_2);
    set(kVTCompressionPropertyKey_TransferFunction, kCVImageBufferTransferFunction_ITU_R_709_2);
    set(kVTCompressionPropertyKey_YCbCrMatrix, kCVImageBufferYCbCrMatrix_ITU_R_709_2);
    VTCompressionSessionPrepareToEncodeFrames(stream.session);
    stream.keyframe_wanted = true;
    return true;
  }

  void close_session(Stream & stream)
  {
    if (stream.session == nullptr) return;
    VTCompressionSessionCompleteFrames(stream.session, kCMTimeInvalid);
    VTCompressionSessionInvalidate(stream.session);
    CFRelease(stream.session);
    stream.session = nullptr;
  }

  std::uint32_t rtp_time(const Stream & stream, const Clock::time_point at) const
  {
    return stream.timestamp_base + static_cast<std::uint32_t>(ticks(at));
  }

  std::int64_t ticks(const Clock::time_point at) const
  {
    return std::chrono::duration_cast<std::chrono::microseconds>(at - epoch).count() * 9 / 100;
  }

  /// Top-down BGRA from bottom-up GL rows; null encodes black.
  void encode(Stream & stream, const std::uint8_t * pixels, const Clock::time_point captured)
  {
    if (stream.session == nullptr && !open_session(stream)) return;
    const CVPixelBufferPoolRef pool = VTCompressionSessionGetPixelBufferPool(stream.session);
    CVPixelBufferRef buffer = nullptr;
    if (pool == nullptr || CVPixelBufferPoolCreatePixelBuffer(nullptr, pool, &buffer) != kCVReturnSuccess) return;
    CVPixelBufferLockBaseAddress(buffer, 0);
    auto * base = static_cast<std::uint8_t *>(CVPixelBufferGetBaseAddress(buffer));
    const std::size_t stride = CVPixelBufferGetBytesPerRow(buffer);
    constexpr std::size_t row = static_cast<std::size_t>(Feeds::kWidth) * 4;
    for (std::size_t y = 0; y < static_cast<std::size_t>(Feeds::kHeight); ++y) {
      if (pixels == nullptr) std::memset(base + y * stride, 0, row);
      else std::memcpy(base + y * stride, pixels + (Feeds::kHeight - 1 - y) * row, row);
    }
    CVPixelBufferUnlockBaseAddress(buffer, 0);
    const OSStatus status = VTCompressionSessionEncodeFrame(stream.session, buffer,
      CMTimeMake(ticks(captured), 90000), kCMTimeInvalid,
      stream.keyframe_wanted.exchange(false) ? force_keyframe : nullptr, &stream, nullptr);
    CVPixelBufferRelease(buffer);
    // A session dies with a GPU reset or across sleep; the next frame rebuilds it.
    if (status == kVTInvalidSessionErr) {
      TraceLog(LOG_WARNING, "video: %s encoder lost; restarting it", stream.name.c_str());
      close_session(stream);
    }
  }

  /// One RTP packet per NAL unit, or FU-A fragments of it (RFC 6184).
  void packetize(const Stream & stream, const std::uint8_t * nal, const std::size_t size, const std::uint32_t time)
  {
    const auto header = [&](const std::size_t payload) {
      spans.emplace_back(packets.size(), 12 + payload);
      const char bytes[12] = {'\x80', 96, 0, 0, static_cast<char>(time >> 24), static_cast<char>(time >> 16),
        static_cast<char>(time >> 8), static_cast<char>(time), static_cast<char>(stream.ssrc >> 24),
        static_cast<char>(stream.ssrc >> 16), static_cast<char>(stream.ssrc >> 8), static_cast<char>(stream.ssrc)};
      packets.append(bytes, sizeof(bytes));
    };
    if (size <= kMaxPayload) {
      header(size);
      packets.append(reinterpret_cast<const char *>(nal), size);
      return;
    }
    for (std::size_t at = 1; at < size;) {
      const std::size_t chunk = std::min(kMaxPayload - 2, size - at);
      header(chunk + 2);
      packets += static_cast<char>((nal[0] & 0xE0) | 28);
      packets += static_cast<char>((at == 1 ? 0x80 : 0) | (at + chunk == size ? 0x40 : 0) | (nal[0] & 0x1F));
      packets.append(reinterpret_cast<const char *>(nal) + at, chunk);
      at += chunk;
    }
  }

  static void encoded(void * impl, void * stream, const OSStatus status, VTEncodeInfoFlags, CMSampleBufferRef sample)
  {
    if (status != noErr || sample == nullptr || !CMSampleBufferDataIsReady(sample)) return;
    static_cast<Impl *>(impl)->deliver(*static_cast<Stream *>(stream), sample);
  }

  /// The encoder's thread: one access unit to every client playing `stream`.
  void deliver(Stream & stream, CMSampleBufferRef sample)
  {
    const bool key = keyframe(sample);
    const CMFormatDescriptionRef format = CMSampleBufferGetFormatDescription(sample);
    const std::uint8_t * sps = nullptr, * pps = nullptr;
    std::size_t sps_size = 0, pps_size = 0;
    int length_size = 4;
    CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 0, &sps, &sps_size, nullptr, &length_size);
    CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, 1, &pps, &pps_size, nullptr, nullptr);
    const CMBlockBufferRef block = CMSampleBufferGetDataBuffer(sample);
    const std::size_t total = CMBlockBufferGetDataLength(block);
    std::string copy;
    char * data = nullptr;
    std::size_t contiguous = 0;
    if (CMBlockBufferGetDataPointer(block, 0, &contiguous, nullptr, &data) != kCMBlockBufferNoErr || contiguous < total) {
      copy.resize(total);
      if (CMBlockBufferCopyDataBytes(block, 0, total, copy.data()) != kCMBlockBufferNoErr) return;
      data = copy.data();
    }
    const auto time = rtp_time(stream, epoch) + static_cast<std::uint32_t>(
      CMTimeConvertScale(CMSampleBufferGetPresentationTimeStamp(sample), 90000, kCMTimeRoundingMethod_Default).value);

    std::lock_guard<std::mutex> lock(mutex);
    if (key && sps != nullptr && pps != nullptr) {
      stream.sps.assign(reinterpret_cast<const char *>(sps), sps_size);
      stream.pps.assign(reinterpret_cast<const char *>(pps), pps_size);
    }
    if (stream.watchers.load() == 0) return;
    packets.clear();
    spans.clear();
    // Parameter sets ahead of every keyframe, so a decoder can start there.
    if (key) {
      packetize(stream, reinterpret_cast<const std::uint8_t *>(stream.sps.data()), stream.sps.size(), time);
      packetize(stream, reinterpret_cast<const std::uint8_t *>(stream.pps.data()), stream.pps.size(), time);
    }
    const auto * bytes = reinterpret_cast<const std::uint8_t *>(data);
    for (std::size_t at = 0; at + static_cast<std::size_t>(length_size) <= total;) {
      std::size_t size = 0;
      for (int i = 0; i < length_size; ++i) size = size << 8 | bytes[at++];
      if (size == 0 || size > total - at) break;
      packetize(stream, bytes + at, size, time);
      at += size;
    }
    if (spans.empty()) return;
    packets[spans.back().first + 1] |= '\x80';  // marker: the access unit's last packet
    bool wake = false;
    for (auto & client : clients) {
      if (!client.playing || client.dead || client.stream != static_cast<int>(stream.index)) continue;
      if (!client.keyed && !key) continue;
      if (client.tcp && client.out.size() > kMaxBacklog) {
        client.keyed = false;
        stream.keyframe_wanted = true;
        continue;
      }
      client.keyed = true;
      for (const auto & [offset, size] : spans) {
        char * packet = packets.data() + offset;
        packet[2] = static_cast<char>(client.sequence >> 8);
        packet[3] = static_cast<char>(client.sequence++);
        if (client.tcp) {
          const char frame[4] = {'$', static_cast<char>(client.channel), static_cast<char>(size >> 8),
            static_cast<char>(size)};
          client.out.append(frame, sizeof(frame)).append(packet, size);
        } else {
          sendto(rtp, packet, size, 0, reinterpret_cast<const sockaddr *>(&client.rtp_to), sizeof(client.rtp_to));
        }
      }
      if (!flush(client)) client.dead = true;
      wake = wake || client.dead || !client.out.empty();
    }
    if (wake) wake_server();
  }

  void wake_server()
  {
    const char byte = 0;
    if (wake[1] >= 0) (void)!write(wake[1], &byte, 1);
  }

  void stop(Client & client)
  {
    if (!client.playing) return;
    client.playing = false;
    --streams[static_cast<std::size_t>(client.stream)]->watchers;
  }

  void respond(Client & client, const std::string & head)
  {
    const std::size_t first = head.find(' ');
    const std::size_t second = first == std::string::npos ? first : head.find(' ', first + 1);
    const std::string method = head.substr(0, first);
    const std::string url = second == std::string::npos ? std::string() : head.substr(first + 1, second - first - 1);
    const std::string cseq = field(head, "CSeq");
    const auto reply = [&](const char * status, const std::string & headers = {}, const std::string & body = {}) {
      client.out += std::string("RTSP/1.0 ") + status + "\r\nCSeq: " + cseq + "\r\n" + headers;
      if (!body.empty()) client.out += "Content-Length: " + std::to_string(body.size()) + "\r\n";
      client.out += "\r\n" + body;
    };
    if (method == "OPTIONS" || method == "GET_PARAMETER" || method == "SET_PARAMETER") {
      return reply("200 OK", "Public: OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER\r\n");
    }
    if (method == "PAUSE" || method == "TEARDOWN") {
      stop(client);
      if (method == "TEARDOWN") client.stream = -1;
      return reply("200 OK");
    }
    if (method == "PLAY") {
      if (client.stream < 0) return reply("455 Method Not Valid in This State");
      Stream & stream = *streams[static_cast<std::size_t>(client.stream)];
      if (!client.playing) {
        client.playing = true;
        client.keyed = false;
        ++stream.watchers;
      }
      stream.keyframe_wanted = true;
      return reply("200 OK", "Session: " + client.session + "\r\nRange: npt=0.000-\r\nRTP-Info: url=" + client.url +
        ";seq=" + std::to_string(client.sequence) + ";rtptime=" + std::to_string(rtp_time(stream, Clock::now())) +
        "\r\n");
    }
    if (method != "DESCRIBE" && method != "SETUP") return reply("501 Not Implemented");
    const std::string name = stream_name(url);
    int index = -1;
    for (const auto & stream : streams) {
      if (stream->name == name) index = static_cast<int>(stream->index);
    }
    if (index < 0) return reply("404 Not Found");
    const Stream & stream = *streams[static_cast<std::size_t>(index)];
    if (method == "DESCRIBE") {
      sockaddr_in6 local{};
      socklen_t size = sizeof(local);
      const bool named = getsockname(client.fd, reinterpret_cast<sockaddr *>(&local), &size) == 0;
      const bool ip4 = !named || IN6_IS_ADDR_V4MAPPED(&local.sin6_addr);
      const std::string host = named ? host_text(local.sin6_addr) : "0.0.0.0";
      const std::string family = ip4 ? "IN IP4 " : "IN IP6 ";
      char profile[7];
      std::snprintf(profile, sizeof(profile), "%02X%02X%02X", static_cast<unsigned char>(stream.sps[1]),
        static_cast<unsigned char>(stream.sps[2]), static_cast<unsigned char>(stream.sps[3]));
      const std::string sdp = "v=0\r\no=- " + std::to_string(stream.ssrc) + " 1 " + family + host + "\r\ns=" +
        stream.name + "\r\nc=" + family + (ip4 ? "0.0.0.0" : "::") + "\r\nt=0 0\r\na=control:*\r\nm=video 0 RTP/AVP 96\r\n"
        "a=rtpmap:96 H264/90000\r\na=fmtp:96 packetization-mode=1;profile-level-id=" + profile +
        ";sprop-parameter-sets=" + base64(stream.sps) + "," + base64(stream.pps) + "\r\n"
        "a=framerate:" + std::to_string(Feeds::kFps) + "\r\na=control:track0\r\n";
      const std::string base = url.back() == '/' ? url : url + "/";
      return reply("200 OK", "Content-Base: " + base + "\r\nContent-Type: application/sdp\r\n", sdp);
    }
    if (client.stream >= 0 && client.stream != index) return reply("459 Aggregate Operation Not Allowed");
    const std::string transport = field(head, "Transport");
    std::string answer;
    if (transport.find("RTP/AVP/TCP") != std::string::npos) {
      const std::size_t at = transport.find("interleaved=");
      client.tcp = true;
      client.channel = static_cast<std::uint8_t>(at == std::string::npos ? 0 : std::strtoul(transport.c_str() + at + 12, nullptr, 10));
      answer = "RTP/AVP/TCP;unicast;interleaved=" + std::to_string(client.channel) + "-" +
        std::to_string(client.channel + 1);
    } else {
      const std::size_t at = transport.find("client_port=");
      const unsigned long client_port = at == std::string::npos ? 0 : std::strtoul(transport.c_str() + at + 12, nullptr, 10);
      sockaddr_in6 peer{};
      socklen_t size = sizeof(peer);
      if (client_port == 0 || client_port > 65534 || transport.find("multicast") != std::string::npos ||
        getpeername(client.fd, reinterpret_cast<sockaddr *>(&peer), &size) != 0)
      {
        return reply("461 Unsupported Transport");
      }
      peer.sin6_port = htons(static_cast<std::uint16_t>(client_port));
      client.tcp = false;
      client.rtp_to = peer;
      answer = "RTP/AVP;unicast;client_port=" + std::to_string(client_port) + "-" + std::to_string(client_port + 1) +
        ";server_port=" + std::to_string(port) + "-" + std::to_string(port + 1);
    }
    char ssrc[9];
    std::snprintf(ssrc, sizeof(ssrc), "%08X", stream.ssrc);
    client.stream = index;
    client.url = url;
    if (client.session.empty()) client.session = std::to_string(random());
    reply("200 OK", "Transport: " + answer + ";ssrc=" + ssrc + "\r\nSession: " + client.session + ";timeout=60\r\n");
  }

  void receive(Client & client)
  {
    char buffer[4096];
    const ssize_t received = recv(client.fd, buffer, sizeof(buffer), 0);
    if (received == 0 || (received < 0 && errno != EAGAIN && errno != EINTR)) {
      client.dead = true;
      return;
    }
    if (received < 0) return;
    client.in.append(buffer, static_cast<std::size_t>(received));
    while (!client.in.empty()) {
      // RTCP the client interleaves on the connection. Nothing here needs it.
      if (client.in[0] == '$') {
        if (client.in.size() < 4) break;
        const std::size_t length = 4 + (static_cast<std::size_t>(static_cast<unsigned char>(client.in[2])) << 8 |
          static_cast<unsigned char>(client.in[3]));
        if (client.in.size() < length) break;
        client.in.erase(0, length);
        continue;
      }
      const std::size_t end = client.in.find("\r\n\r\n");
      if (end == std::string::npos) {
        client.dead = client.in.size() > kMaxRequest;
        break;
      }
      const std::string head = client.in.substr(0, end + 2);
      const std::size_t body = std::strtoul(field(head, "Content-Length").c_str(), nullptr, 10);
      if (body > kMaxRequest) {
        client.dead = true;
        break;
      }
      if (client.in.size() < end + 4 + body) break;
      client.in.erase(0, end + 4 + body);
      respond(client, head);
    }
    if (!flush(client)) client.dead = true;
  }

  void accept_clients()
  {
    for (;;) {
      sockaddr_in6 peer{};
      socklen_t size = sizeof(peer);
      const int fd = accept(listener, reinterpret_cast<sockaddr *>(&peer), &size);
      if (fd < 0) return;
      if (clients.size() >= kMaxConnections) {
        close(fd);
        continue;
      }
      const int on = 1;
      setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
      setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
      // A phone that leaves Wi-Fi never closes its connection; keepalive ends
      // its session within half a minute, so UDP video stops going to it.
      setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
      const int idle = 10, interval = 5, count = 3;
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPALIVE, &idle, sizeof(idle));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
      try {
        nonblocking(fd);
      } catch (const std::exception &) {
        close(fd);
        continue;
      }
      Client client;
      client.fd = fd;
      client.sequence = static_cast<std::uint16_t>(random());
      clients.push_back(std::move(client));
      TraceLog(LOG_INFO, "video: RTSP client %s:%u connected", host_text(peer.sin6_addr).c_str(),
        ntohs(peer.sin6_port));
    }
  }

  /// The RTSP thread. A session lives as long as its connection, which
  /// keepalive bounds; video itself is sent from the encoder's thread.
  void serve()
  {
    std::vector<pollfd> fds;
    std::vector<Client *> polled;
    char drain[2048];
    while (!stopping) {
      fds.assign({{listener, POLLIN, 0}, {rtcp, POLLIN, 0}, {wake[0], POLLIN, 0}});
      polled.clear();
      {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto & client : clients) {
          fds.push_back({client.fd, static_cast<short>(POLLIN | (client.out.empty() ? 0 : POLLOUT)), 0});
          polled.push_back(&client);
        }
      }
      if (poll(fds.data(), static_cast<nfds_t>(fds.size()), 1000) < 0) continue;
      std::lock_guard<std::mutex> lock(mutex);
      while ((fds[2].revents & POLLIN) != 0 && read(wake[0], drain, sizeof(drain)) > 0) {}
      while ((fds[1].revents & POLLIN) != 0 && recv(rtcp, drain, sizeof(drain), 0) > 0) {}
      // Only this thread removes clients, so the pointers are still good.
      for (std::size_t index = 0; index < polled.size(); ++index) {
        Client & client = *polled[index];
        const short events = fds[3 + index].revents;
        if ((events & POLLIN) != 0) receive(client);
        else if ((events & (POLLERR | POLLHUP)) != 0) client.dead = true;
        if (!client.dead && (events & POLLOUT) != 0 && !flush(client)) client.dead = true;
      }
      if ((fds[0].revents & POLLIN) != 0) accept_clients();
      for (auto client = clients.begin(); client != clients.end();) {
        if (!client->dead) {
          ++client;
          continue;
        }
        stop(*client);
        close(client->fd);
        TraceLog(LOG_INFO, "video: RTSP client disconnected");
        client = clients.erase(client);
      }
    }
  }
};

Feeds::Feeds(const std::uint16_t port, const std::vector<std::string> & names)
: _impl(std::make_unique<Impl>(port, names))
{
  _impl->start();
  stay_awake();
  TraceLog(LOG_INFO, "video: %zu camera feeds on rtsp://<this host>:%u/<vehicle>", names.size(), port);
}

Feeds::~Feeds() = default;

std::size_t Feeds::size() const noexcept { return _impl->streams.size(); }

const std::string & Feeds::name(const std::size_t stream) const noexcept { return _impl->streams[stream]->name; }

bool Feeds::due(const std::size_t index, const Clock::time_point now)
{
  Stream & stream = *_impl->streams[index];
  if (stream.watchers.load() == 0) {
    // Frames read back for an earlier viewer are stale for the next one.
    stream.pending[0] = stream.pending[1] = false;
    return false;
  }
  // Half a 60 Hz frame of slack, so render-loop jitter does not skip a frame.
  if (now + std::chrono::milliseconds(8) < stream.due_at) return false;
  stream.due_at = now - stream.due_at > kInterval ? now + kInterval : stream.due_at + kInterval;
  return true;
}

void Feeds::begin(const std::size_t index) { BeginTextureMode(_impl->streams[index]->target); }

void Feeds::end(const std::size_t index, const Clock::time_point captured_at)
{
  EndTextureMode();
  Stream & stream = *_impl->streams[index];
  const int write = stream.next, read = 1 - write;
  glBindFramebuffer(GL_READ_FRAMEBUFFER, stream.target.id);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, stream.pixels[write]);
  glReadPixels(0, 0, kWidth, kHeight, GL_BGRA, GL_UNSIGNED_INT_8_8_8_8_REV, nullptr);
  stream.pending[write] = true;
  stream.captured[write] = captured_at;
  // The other buffer was filled a frame ago, so mapping it does not wait.
  if (stream.pending[read]) {
    glBindBuffer(GL_PIXEL_PACK_BUFFER, stream.pixels[read]);
    if (const auto * pixels = static_cast<const std::uint8_t *>(
      glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, static_cast<GLsizeiptr>(kFrameBytes), GL_MAP_READ_BIT)))
    {
      _impl->encode(stream, pixels, stream.captured[read]);
      glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    }
    stream.pending[read] = false;
  }
  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  stream.next = read;
}

}  // namespace sim::video
