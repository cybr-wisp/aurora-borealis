#include "ingestion/udp_receiver.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <array>

namespace aurora::ingestion {

namespace {

#ifdef _WIN32

class WinsockRuntime {
public:
    WinsockRuntime() noexcept {
        WSADATA data{};
        initialized_ = WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }

    ~WinsockRuntime() {
        if (initialized_) {
            WSACleanup();
        }
    }

    [[nodiscard]] bool initialized() const noexcept {
        return initialized_;
    }

    WinsockRuntime(const WinsockRuntime&) = delete;
    WinsockRuntime& operator=(const WinsockRuntime&) = delete;

private:
    bool initialized_{false};
};

using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;

void close_socket(SocketHandle socket) {
    closesocket(socket);
}

#else

using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;

void close_socket(SocketHandle socket) {
    ::close(socket);
}

#endif

}  // namespace

UdpReceiver::UdpReceiver(
    std::uint16_t port,
    SpscRing<aurora::proto::Observation, 4096>& queue)
    : port_(port),
      queue_(queue) {}

UdpReceiver::~UdpReceiver() {
    stop();
}

void UdpReceiver::start() {
    if (running_.exchange(true)) {
        return;
    }

    thread_ = std::thread(&UdpReceiver::run, this);
}

void UdpReceiver::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

ReceiverCounters UdpReceiver::counters() const {
    return {
        received_.load(),
        malformed_.load(),
        queue_full_.load()
    };
}

void UdpReceiver::run() {
#ifdef _WIN32
    WinsockRuntime winsock;
    if (!winsock.initialized()) {
        running_.store(false);
        return;
    }
#endif

    const SocketHandle socket_handle =
        ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_handle == kInvalidSocket) {
        running_.store(false);
        return;
    }

#ifdef _WIN32
    const DWORD timeout_ms = 100;

    setsockopt(
        socket_handle,
        SOL_SOCKET,
        SO_RCVTIMEO,
        reinterpret_cast<const char*>(&timeout_ms),
        sizeof(timeout_ms));
#else
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;

    setsockopt(
        socket_handle,
        SOL_SOCKET,
        SO_RCVTIMEO,
        &timeout,
        sizeof(timeout));
#endif

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port_);

    if (::bind(
            socket_handle,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address)) != 0) {
        close_socket(socket_handle);
        running_.store(false);
        return;
    }

    std::array<char, 2048> buffer{};

    while (running_.load(std::memory_order_relaxed)) {
#ifdef _WIN32
        const int received_bytes = ::recvfrom(
            socket_handle,
            buffer.data(),
            static_cast<int>(buffer.size()),
            0,
            nullptr,
            nullptr);
#else
        const auto received_bytes = ::recvfrom(
            socket_handle,
            buffer.data(),
            buffer.size(),
            0,
            nullptr,
            nullptr);
#endif

        if (received_bytes <= 0) {
            continue;
        }

        ++received_;

        aurora::proto::Observation observation;

        if (!observation.ParseFromArray(
                buffer.data(),
                static_cast<int>(received_bytes))) {
            ++malformed_;
            continue;
        }

        if (!queue_.push(std::move(observation))) {
            ++queue_full_;
        }
    }

    close_socket(socket_handle);
}

}  // namespace aurora::ingestion
