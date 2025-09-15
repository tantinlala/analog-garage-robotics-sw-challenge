#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "state_machine/state_machine.hpp"
#include "speed_limiter_states.hpp"
#include "speed_state_publisher.hpp"

namespace analog::speed_limiter
{

/**
 * @brief ROS2 node that publishes speed states based on proximity data and estop events.
 */
class Node : public rclcpp::Node
{
public:
  Node();

private:
  using SpeedStateMachine = sm::StateMachine<StateId, Events>;
  using EstopMsgType = std_msgs::msg::Bool;
  using ProximityMsgType = std_msgs::msg::Float32;

  static constexpr const char * kStopBoundaryName{"stop_boundary"};
  static constexpr const char * kSlowBoundaryName{"slow_boundary"};
  static constexpr const char * kHysteresisName{"hysteresis"};

  static constexpr float kDefaultStopBoundary{400.0};
  static constexpr float kDefaultSlowBoundary{800.0};
  static constexpr float kDefaultHysteresis{50.0};

  static constexpr std::size_t kSpeedStateDepth{1};
  static constexpr std::size_t kEstopDepth{1};

  /**
   * @brief Helper function for declaring a ROS2 boundary parameter and updating the params struct.
   * @param name Name of the parameter
   * @param state StateId associated with region right below boundary
   * @param default_value Default value for the boundary distance
   * @param params The Params struct to update
   */
  void DeclareBoundaryParameter(
    const char * name, const StateId state, const float default_value,
    std::shared_ptr<NotEstoppedState::Params> params);

  /**
   * @brief Declare ROS2 parameters and construct the params struct.
   * @return Shared pointer to the constructed params struct
   */
  std::shared_ptr<NotEstoppedState::Params> SetupParams();

  /**
   * @brief Setup the state publisher to be used upon entry by states.
   * @return Shared pointer to the constructed state publisher
   */
  std::shared_ptr<SpeedStatePublisher> SetupStatePublisher();

  /**
   * @brief Construct all the states and compose them into a state machine.
   * @return Unique pointer to the constructed state machine
   */
  std::unique_ptr<SpeedStateMachine> SetupStateMachine(
    std::shared_ptr<NotEstoppedState::Params> params,
    std::shared_ptr<SpeedStatePublisher> state_publisher);

  /**
   * @brief Setup the estop subscription.
   * @return Shared pointer to the created estop subscription
   */
  rclcpp::Subscription<EstopMsgType>::SharedPtr SubscribeToEstop();

  /**
   * @brief Setup the proximity data subscription.
   * @return Shared pointer to the created proximity data subscription
   */
  rclcpp::Subscription<ProximityMsgType>::SharedPtr SubscribeToProximity();

  /**
   * @brief Update the state machine based on an estop message.
   */
  void EstopCallback(const EstopMsgType::SharedPtr msg);

  /**
   * @brief Update the state machine based on a proximity data message.
   */
  void ProximityDataCallback(const ProximityMsgType::SharedPtr msg);

  rclcpp::Logger logger_;
  rclcpp::Subscription<EstopMsgType>::SharedPtr estop_subscription_;
  rclcpp::Subscription<ProximityMsgType>::SharedPtr proximity_subscription_;
  std::unique_ptr<SpeedStateMachine> state_machine_;
};

}
