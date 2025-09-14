#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "i_publisher.hpp"
#include "speed_limiter_states.hpp"

namespace analog::speed_limiter
{

/**
 * @brief Class for publishing speed state messages over ROS and logging states published.
 */
class SpeedStatePublisher : public IPublisher<StateId>
{
public:
  SpeedStatePublisher(
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
    rclcpp::Logger & logger);

  /**
   * @brief Publish a speed state message over ROS and log the state.
   * @param state The StateId to publish and log
   */
  void Publish(const StateId state) override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Logger * logger_;
};

}
