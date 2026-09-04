#include "estimation/maneuver_detector.h"
#include <numeric>
#include <stdexcept>
namespace aurora::estimation {
ManeuverDetector::ManeuverDetector(std::size_t window,double threshold,std::size_t hold,double inflation)
  :window_(window),hold_steps_(hold),threshold_(threshold),inflation_(inflation){ if(!window||!hold||inflation<1.0) throw std::invalid_argument("invalid maneuver detector config"); }
bool ManeuverDetector::observe(double nis){
  values_.push_back(nis); if(values_.size()>window_) values_.pop_front();
  if(values_.size()==window_){ double mean=std::accumulate(values_.begin(),values_.end(),0.0)/values_.size(); if(mean>threshold_){active_=true; remaining_=hold_steps_;} }
  if(active_ && remaining_>0) --remaining_;
  if(active_ && remaining_==0) active_=false;
  return active_;
}
}
