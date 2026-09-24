#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sim::video {

/// Each vehicle's camera as a live H.264 stream at rtsp://<host>:<port>/<vehicle id>,
/// for a ground station such as UAS Tool.
///
/// A camera is drawn only while someone is watching it: the viewer asks due(),
/// then draws the view between begin() and end() into the stream's own target.
/// end() reads the frame back through a pixel buffer a frame later, so it never
/// waits on the GPU, and hands it to the hardware encoder; the encoder's output
/// goes to every client playing that stream, as RTP over UDP or interleaved on
/// the RTSP connection, whichever the client asks for.
class Feeds final {
public:
  static constexpr int kWidth = 1280;
  static constexpr int kHeight = 720;
  static constexpr int kFps = 30;

  /// Needs the window's GL context. Serves RTSP on TCP `port` and sends RTP from
  /// UDP `port` and `port` + 1. Throws when a port or the encoder is unavailable.
  Feeds(std::uint16_t port, const std::vector<std::string> & names);
  ~Feeds();
  Feeds(const Feeds &) = delete;
  Feeds & operator=(const Feeds &) = delete;

  std::size_t size() const noexcept;
  const std::string & name(std::size_t stream) const noexcept;
  /// True when someone is watching `stream` and its next frame is due.
  bool due(std::size_t stream, std::chrono::steady_clock::time_point now);
  void begin(std::size_t stream);
  void end(std::size_t stream, std::chrono::steady_clock::time_point captured_at);

private:
  struct Impl;
  std::unique_ptr<Impl> _impl;
};

}  // namespace sim::video
