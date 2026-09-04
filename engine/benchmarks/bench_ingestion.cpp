#include "ingestion/observation_queue.h"
#include "observation.pb.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <string>
#include <vector>

int main(){
  using Clock=std::chrono::steady_clock;
  aurora::proto::Observation prototype;
  prototype.set_sensor_id(1); prototype.set_sequence_number(1); prototype.set_timestamp_sec(1.0);
  prototype.set_range_m(1000.0); prototype.set_azimuth_rad(0.4); prototype.set_elevation_rad(0.1);
  prototype.set_range_sigma(10.0); prototype.set_azimuth_sigma(0.0025); prototype.set_elevation_sigma(0.0025);
  prototype.set_sent_monotonic_ns(1);
  std::string payload; if(!prototype.SerializeToString(&payload)) return 2;

  aurora::ingestion::SpscRing<aurora::proto::Observation,4096> q;
  constexpr int N=300000;
  std::vector<double> latency_us; latency_us.reserve(N);
  auto start=Clock::now();
  for(int i=0;i<N;++i){
    const auto t0=Clock::now();
    aurora::proto::Observation decoded;
    if(!decoded.ParseFromArray(payload.data(),static_cast<int>(payload.size()))) return 3;
    decoded.set_sequence_number(static_cast<std::uint64_t>(i));
    if(!q.push(std::move(decoded))) return 4;
    auto consumed=q.pop(); if(!consumed) return 5;
    const auto t1=Clock::now();
    latency_us.push_back(std::chrono::duration<double,std::micro>(t1-t0).count());
  }
  const auto end=Clock::now();
  std::sort(latency_us.begin(),latency_us.end());
  auto pct=[&](double p){return latency_us[static_cast<std::size_t>(p*static_cast<double>(latency_us.size()-1))];};
  const double sec=std::chrono::duration<double>(end-start).count();
  std::cout << "payload_bytes="<<payload.size()<<" messages="<<N<<" throughput_msg_s="<<(N/sec)
            <<" p50_us="<<pct(.50)<<" p95_us="<<pct(.95)<<" p99_us="<<pct(.99)<<"\n";
  return 0;
}
