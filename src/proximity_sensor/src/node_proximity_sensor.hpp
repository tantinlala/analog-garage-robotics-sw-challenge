#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32.hpp>
#include "list_distance_source.hpp"

namespace analog::proximity_sensor
{

/**
 * @brief ROS2 node that publishes proximity distance readings at
 * regular intervals from a list defined in a ROS2 parameter.
 */
class Node : public rclcpp::Node
{
public:
  Node();

private:
  static constexpr const char * kSampleTimeParamName{"sample_time_ms"};
  static constexpr const char * kDistanceSeriesParamName{"distance_series"};

  using ProximityMsgType = std_msgs::msg::Float32;

  std::unique_ptr<ListDistanceSource> distance_source_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ProximityMsgType>::SharedPtr publisher_;

  /**
   * @brief Setup the timer for periodic distance publishing.
   * @details Declares a ROS2 parameter for the sample time and
   * gets the parameter value to create the timer.
   * @return Shared pointer to the created timer
   */
  rclcpp::TimerBase::SharedPtr SetupTimer();

  /**
    * @brief Setup the distance source to provide distance readings.
    * @details Declares ROS2 parameters for the distance series and
    * gets the parameter values to construct the distance source.
    * @return Unique pointer to the created distance source
   */
  std::unique_ptr<ListDistanceSource> SetupDistanceSource();

  /**
   * @brief Timer callback to publish the next distance reading.
   */
  void TimerCallback();
};

}
