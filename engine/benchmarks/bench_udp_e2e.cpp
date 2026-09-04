#include "ingestion/observation_queue.h"
#include "ingestion/udp_receiver.h"
#include "ingestion/validator.h"
#include "observation.pb.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

#ifdef _WIN32
class SocketRuntime {
 public:
  SocketRuntime() noexcept {
    WSADATA data{};
    initialized_ = WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }

  ~SocketRuntime() {
    if (initialized_) {
      WSACleanup();
    }
  }

  bool initialized() const noexcept { return initialized_; }

 private:
  bool initialized_{false};
};

using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;

void close_socket(SocketHandle socket) {
  closesocket(socket);
}
#else
class SocketRuntime {
 public:
  bool initialized() const noexcept { return true; }
};

using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;

void close_socket(SocketHandle socket) {
  ::close(socket);
}
#endif

std::uint64_t monotonic_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          Clock::now().time_since_epoch())
          .count());
}

double epoch_sec() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

void busy_wait_us(int delay_us) {
  if (delay_us <= 0) {
    return;
  }

  const auto deadline =
      Clock::now() +
      std::chrono::microseconds(delay_us);

  while (Clock::now() < deadline) {
  }
}

void append_varint(std::string& output, std::uint64_t value) {
  while (value >= 0x80) {
    output.push_back(static_cast<char>((value & 0x7f) | 0x80));
    value >>= 7;
  }
  output.push_back(static_cast<char>(value));
}

void append_unknown_padding(std::string& payload, std::size_t bytes) {
  if (bytes == 0) {
    return;
  }

  constexpr std::uint64_t field_number = 1000;
  constexpr std::uint64_t wire_type_length_delimited = 2;

  append_varint(
      payload,
      (field_number << 3) | wire_type_length_delimited);

  append_varint(payload, static_cast<std::uint64_t>(bytes));
  payload.append(bytes, '\0');
}

double percentile(
    const std::vector<double>& sorted,
    double probability) {
  if (sorted.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto index = static_cast<std::size_t>(
      probability * static_cast<double>(sorted.size() - 1));

  return sorted[index];
}

struct CaseSpec {
  std::string name;
  int messages;
  int target_rate_msg_s;
  int consumer_delay_us;
  std::size_t padding_bytes;
};

void run_case(
    const CaseSpec& spec,
    std::uint32_t sensor_id,
    std::uint16_t port,
    SocketHandle sender,
    aurora::ingestion::UdpReceiver& receiver,
    aurora::ingestion::SpscRing<
        aurora::proto::Observation,
        4096>& queue) {

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(port);
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

  aurora::ingestion::Validator validator(5.0);

  const auto before = receiver.counters();

  std::atomic<bool> producer_done{false};
  std::atomic<int> sent{0};
  std::atomic<std::size_t> payload_bytes{0};

  std::vector<double> latency_us;
  latency_us.reserve(static_cast<std::size_t>(spec.messages));

  std::uint64_t accepted = 0;
  std::uint64_t rejected = 0;

  const auto case_start = Clock::now();
  Clock::time_point producer_end = case_start;

  std::thread producer([&] {
    auto next_send = Clock::now();

    const auto interval =
        spec.target_rate_msg_s > 0
            ? std::chrono::nanoseconds(
                  1'000'000'000LL /
                  spec.target_rate_msg_s)
            : std::chrono::nanoseconds(0);

    for (int i = 0; i < spec.messages; ++i) {
      aurora::proto::Observation observation;

      observation.set_sensor_id(sensor_id);
      observation.set_sequence_number(
          static_cast<std::uint64_t>(i + 1));
      observation.set_timestamp_sec(epoch_sec());
      observation.set_range_m(1000.0);
      observation.set_azimuth_rad(0.4);
      observation.set_elevation_rad(0.1);
      observation.set_range_sigma(10.0);
      observation.set_azimuth_sigma(0.0025);
      observation.set_elevation_sigma(0.0025);
      observation.set_sent_monotonic_ns(monotonic_ns());

      std::string payload;

      if (!observation.SerializeToString(&payload)) {
        continue;
      }

      append_unknown_padding(payload, spec.padding_bytes);
      payload_bytes.store(payload.size());

#ifdef _WIN32
      const int result = ::sendto(
          sender,
          payload.data(),
          static_cast<int>(payload.size()),
          0,
          reinterpret_cast<const sockaddr*>(&destination),
          sizeof(destination));
#else
      const auto result = ::sendto(
          sender,
          payload.data(),
          payload.size(),
          0,
          reinterpret_cast<const sockaddr*>(&destination),
          sizeof(destination));
#endif

      if (result > 0) {
        ++sent;
      }

      if (spec.target_rate_msg_s > 0) {
        next_send += interval;
        std::this_thread::sleep_until(next_send);
      }
    }

    producer_end = Clock::now();
    producer_done.store(true);
  });

  auto last_activity = Clock::now();

  while (true) {
    auto observation = queue.pop();

    if (observation) {
      last_activity = Clock::now();

      const auto validation =
          validator.validate(*observation, epoch_sec());

      if (validation ==
          aurora::ingestion::ValidationResult::kAccept) {
        ++accepted;

        if (observation->sent_monotonic_ns() != 0) {
          const auto now_ns = monotonic_ns();

          if (now_ns >= observation->sent_monotonic_ns()) {
            latency_us.push_back(
                static_cast<double>(
                    now_ns -
                    observation->sent_monotonic_ns()) /
                1000.0);
          }
        }
      } else {
        ++rejected;
      }

      if (spec.consumer_delay_us > 0) {
        busy_wait_us(spec.consumer_delay_us);
      }

      continue;
    }

    if (producer_done.load() &&
        Clock::now() - last_activity >
            std::chrono::milliseconds(250)) {
      break;
    }

    std::this_thread::yield();
  }

  producer.join();

  const auto after = receiver.counters();

  std::sort(latency_us.begin(), latency_us.end());

  const auto received =
      after.received - before.received;

  const auto malformed =
      after.malformed - before.malformed;

  const auto queue_full =
      after.queue_full - before.queue_full;

  const auto sent_count =
      static_cast<std::uint64_t>(sent.load());

  const auto transport_drop =
      sent_count > received
          ? sent_count - received
          : 0;

  const double producer_sec =
      std::chrono::duration<double>(
          producer_end - case_start)
          .count();

  const double observed_rate =
      producer_sec > 0.0
          ? static_cast<double>(received) /
                producer_sec
          : 0.0;

  std::cout
      << spec.name << ","
      << payload_bytes.load() << ","
      << spec.target_rate_msg_s << ","
      << sent_count << ","
      << received << ","
      << accepted << ","
      << rejected << ","
      << malformed << ","
      << queue_full << ","
      << transport_drop << ","
      << observed_rate << ","
      << percentile(latency_us, 0.50) << ","
      << percentile(latency_us, 0.95) << ","
      << percentile(latency_us, 0.99)
      << "\n";
}

}  // namespace

int main() {
  constexpr std::uint16_t port = 46001;

  SocketRuntime runtime;

  if (!runtime.initialized()) {
    std::cerr << "socket runtime initialization failed\n";
    return 2;
  }

  const SocketHandle sender =
      ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (sender == kInvalidSocket) {
    std::cerr << "sender socket creation failed\n";
    return 3;
  }

  aurora::ingestion::SpscRing<
      aurora::proto::Observation,
      4096>
      queue;

  aurora::ingestion::UdpReceiver receiver(port, queue);

  receiver.start();

  std::this_thread::sleep_for(
      std::chrono::milliseconds(100));

  std::cout << std::fixed << std::setprecision(3);

  std::cout
      << "case,payload_bytes,target_rate_msg_s,"
      << "sent,received,accepted,rejected,malformed,"
      << "queue_full,transport_drop,"
      << "observed_receive_msg_s,"
      << "p50_us,p95_us,p99_us\n";

  const std::vector<CaseSpec> cases = {
      {"nominal_1k", 2000, 1000, 0, 0},
      {"nominal_5k", 10000, 5000, 0, 0},
      {"padded_5k_64", 10000, 5000, 0, 64},
      {"padded_5k_256", 10000, 5000, 0, 256},
      {"unpaced", 100000, 0, 0, 0},
      {"saturation", 50000, 0, 200, 0},
  };

  std::uint32_t sensor_id = 100;

  for (const auto& spec : cases) {
    run_case(
        spec,
        sensor_id++,
        port,
        sender,
        receiver,
        queue);
  }

  receiver.stop();
  close_socket(sender);

  return 0;
}