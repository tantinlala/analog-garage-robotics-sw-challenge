#pragma once

#include <vector>
#include <string>
#include <optional>

namespace analog::proximity_sensor
{

/**
  * @brief Class that provides distance readings from a predefined list of distances.
 */
class ListDistanceSource
{
public:
  using Series = std::vector<double>;

  /**
   * @brief Construct a new List Distance Source object
    * @param series A vector of distances to return on successive calls to GetDistance()
   */
  ListDistanceSource(Series & series);

  /**
    * @brief Get the next distance reading from the list.
    * @details Each call to GetDistance() returns the next distance in the list until the end
    * of the list is reached, after which it returns std::nullopt.
   */
  std::optional<float> GetDistance();

private:
  Series series_;
  std::vector<double>::iterator iterator_;
};

}
