#include "ingestion/udp_receiver.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <array>
#include <stdexcept>

namespace aurora::ingestion {
UdpReceiver::UdpReceiver(std::uint16_t p,SpscRing<aurora::proto::Observation,4096>& q):port_(p),queue_(q){}
UdpReceiver::~UdpReceiver(){ stop(); }
void UdpReceiver::start(){ if(running_.exchange(true)) return; thread_=std::thread(&UdpReceiver::run,this); }
void UdpReceiver::stop(){ if(!running_.exchange(false)) return; if(thread_.joinable()) thread_.join(); }
ReceiverCounters UdpReceiver::counters() const { return {received_.load(),malformed_.load(),queue_full_.load()}; }
void UdpReceiver::run(){
  int fd=::socket(AF_INET,SOCK_DGRAM,0); if(fd<0) throw std::runtime_error("socket failed");
  timeval tv{0,100000}; setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
  sockaddr_in addr{}; addr.sin_family=AF_INET; addr.sin_addr.s_addr=htonl(INADDR_ANY); addr.sin_port=htons(port_);
  if(::bind(fd,reinterpret_cast<sockaddr*>(&addr),sizeof(addr))<0){::close(fd); throw std::runtime_error("bind failed");}
  std::array<char,2048> buf{};
  while(running_.load(std::memory_order_relaxed)){
    const auto n=::recvfrom(fd,buf.data(),buf.size(),0,nullptr,nullptr); if(n<=0) continue;
    ++received_; aurora::proto::Observation o;
    if(!o.ParseFromArray(buf.data(),static_cast<int>(n))){++malformed_; continue;}
    if(!queue_.push(std::move(o))) ++queue_full_;
  }
  ::close(fd);
}
}
