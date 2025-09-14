#include "list_distance_source.hpp"

namespace analog::proximity_sensor
{

ListDistanceSource::ListDistanceSource(Series & series)
: series_(std::move(series)),
  iterator_(this->series_.begin())
{

}

std::optional<float> ListDistanceSource::GetDistance()
{
  if (this->iterator_ == this->series_.end()) {
    return std::nullopt;
  }

  float distance = *this->iterator_;
  ++this->iterator_;
  return distance;
}

}
