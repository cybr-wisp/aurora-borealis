#include "ingestion/observation_queue.h"
#include "ingestion/udp_receiver.h"
#include "ingestion/validator.h"
#include "observation.pb.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

int main(int argc,char** argv){
  const std::uint16_t port = argc>1 ? static_cast<std::uint16_t>(std::stoi(argv[1])) : 46000;
  const int duration_sec = argc>2 ? std::stoi(argv[2]) : 10;
  aurora::ingestion::SpscRing<aurora::proto::Observation,4096> queue;
  aurora::ingestion::UdpReceiver receiver(port,queue);
  aurora::ingestion::Validator validator(1.0);
  std::uint64_t accepted=0,rejected=0;
  receiver.start();
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(duration_sec);
  while(std::chrono::steady_clock::now()<deadline){
    if(auto obs=queue.pop()){
      const double now=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
      if(validator.validate(*obs,now)==aurora::ingestion::ValidationResult::kAccept) ++accepted; else ++rejected;
    } else std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
  receiver.stop();
  const auto c=receiver.counters();
  std::cout<<"received="<<c.received<<" accepted="<<accepted<<" rejected="<<rejected<<" malformed="<<c.malformed<<" queue_full="<<c.queue_full<<"\n";
  return EXIT_SUCCESS;
}
