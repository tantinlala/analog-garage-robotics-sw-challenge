#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/bool.hpp>

namespace analog::estop_monitor
{

/**
 * @brief ROS2 node that publishes an estop triggered event after a specified time.
 */
class Node : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the estop monitor node.
   * @details Initially "clears" the estop as a side effect.
   */
  Node();

private:
  using EstopMsgType = std_msgs::msg::Bool;

  static constexpr const char * kTriggerTimeName{"trigger_time_ms"};
  static constexpr std::size_t kEstopDepth{1};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<EstopMsgType>::SharedPtr publisher_;

  /**
   * @brief Setup the timer for triggering the estop event.
   * @details Declares a ROS2 parameter for the trigger time and
   * gets the parameter value to create the timer.
   * @return Shared pointer to the created timer
   */
  rclcpp::TimerBase::SharedPtr SetupTimer();

  /**
   * @brief Setup the publisher for the estop event.
   * @details Creates a ROS2 publisher for the estop event.
   * @return Shared pointer to the created publisher
   */
  rclcpp::Publisher<EstopMsgType>::SharedPtr SetupPublisher();

  /**
   * @brief Timer callback to publish the estop triggered event.
   */
  void TimerCallback();
};

}
