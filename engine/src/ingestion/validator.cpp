#include "ingestion/validator.h"
#include <cmath>
namespace aurora::ingestion {
ValidationResult Validator::validate(const aurora::proto::Observation& o,double now){
  if(!std::isfinite(o.timestamp_sec())||!std::isfinite(o.range_m())||o.range_m()<=0.0||o.range_sigma()<=0.0||o.azimuth_sigma()<=0.0||o.elevation_sigma()<=0.0) return ValidationResult::kInvalid;
  if(now-o.timestamp_sec()>max_staleness_sec_) return ValidationResult::kStale;
  auto it=last_sequence_.find(o.sensor_id());
  if(it!=last_sequence_.end() && o.sequence_number()<=it->second) return ValidationResult::kDuplicateOrOldSequence;
  last_sequence_[o.sensor_id()]=o.sequence_number(); return ValidationResult::kAccept;
}
}
