#include <memory>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "state_machine/state_machine.hpp"
#include "speed_limiter_states.hpp"
#include "speed_state_publisher.hpp"
#include "node_speed_limiter.hpp"

namespace analog::speed_limiter
{

Node::Node()
: rclcpp::Node("speed_limiter"),
  logger_(this->get_logger())
{
  std::shared_ptr<NotEstoppedState::Params> params = this->SetupParams();
  std::shared_ptr<SpeedStatePublisher> state_publisher = this->SetupStatePublisher();
  this->state_machine_ = this->SetupStateMachine(params, state_publisher);
  this->estop_subscription_ = this->SubscribeToEstop();
  this->proximity_subscription_ = this->SubscribeToProximity();
}

void Node::DeclareBoundaryParameter(
  const char * name, const StateId state,
  const float default_value, std::shared_ptr<NotEstoppedState::Params> params)
{
  this->declare_parameter(name, default_value);
  rclcpp::Parameter boundary_distance = this->get_parameter(name);
  auto boundary{std::find_if(
      params->boundaries.begin(),
      params->boundaries.end(),
      [state](auto & boundary) {return boundary.state == state;})};
  boundary->distance_mm = boundary_distance.as_double();
}

std::shared_ptr<NotEstoppedState::Params> Node::SetupParams()
{
  auto params{std::make_shared<NotEstoppedState::Params>()};

  this->DeclareBoundaryParameter(kStopBoundaryName, StateId::STOP, kDefaultStopBoundary, params);
  this->DeclareBoundaryParameter(kSlowBoundaryName, StateId::SLOW, kDefaultSlowBoundary, params);
  params->boundaries.back() =
    NotEstoppedState::ProximityBoundary{StateId::FULL_SPEED,
    std::numeric_limits<float>::infinity()};
  for (const auto & boundary : params->boundaries) {
    RCLCPP_INFO(
      this->logger_,
      "Boundary for state %s set to %.1f mm",
      state_id_to_string(boundary.state),
      boundary.distance_mm);
  }
  this->declare_parameter(kHysteresisName, kDefaultHysteresis);
  rclcpp::Parameter hysteresis = this->get_parameter(kHysteresisName);
  params->hysteresis = hysteresis.as_double();
  RCLCPP_INFO(
    this->logger_,
    "Hysteresis set to %.1f mm",
    params->hysteresis);

  return params;
}

std::shared_ptr<SpeedStatePublisher> Node::SetupStatePublisher()
{
  rclcpp::QoS speed_state_qos{rclcpp::SystemDefaultsQoS()};
  speed_state_qos.keep_last(kSpeedStateDepth).transient_local().reliable();
  auto publisher = this->create_publisher<SpeedStatePublisher::SpeedStateMsgType>(
    "analog/speed_state",
    speed_state_qos);
  return std::make_shared<SpeedStatePublisher>(publisher, this->logger_);
}

std::unique_ptr<Node::SpeedStateMachine> Node::SetupStateMachine(
  std::shared_ptr<NotEstoppedState::Params> params,
  std::shared_ptr<SpeedStatePublisher> state_publisher)
{
  auto estopped_state = std::make_unique<EstoppedState>(state_publisher);
  auto stop_state = std::make_unique<NotEstoppedState>(
    StateId::STOP,
    params,
    state_publisher
  );
  auto slow_state = std::make_unique<NotEstoppedState>(
    StateId::SLOW,
    params,
    state_publisher
  );
  auto full_speed_state = std::make_unique<NotEstoppedState>(
    StateId::FULL_SPEED,
    params,
    state_publisher
  );
  return std::make_unique<SpeedStateMachine>(
    SpeedStateMachine::StateArray{
      std::move(estopped_state),
      std::move(stop_state),
      std::move(slow_state),
      std::move(full_speed_state)
    }
  );
}

rclcpp::Subscription<Node::EstopMsgType>::SharedPtr Node::SubscribeToEstop()
{
  rclcpp::QoS estop_qos{rclcpp::SystemDefaultsQoS()};
  estop_qos.keep_last(kEstopDepth).transient_local().reliable();
  return this->create_subscription<EstopMsgType>(
    "analog/estop_triggered",
    estop_qos,
    std::bind(&Node::EstopCallback, this, std::placeholders::_1)
  );
}

rclcpp::Subscription<Node::ProximityMsgType>::SharedPtr Node::SubscribeToProximity()
{
  return this->create_subscription<ProximityMsgType>(
    "analog/proximity_data",
    rclcpp::SensorDataQoS(),
    std::bind(&Node::ProximityDataCallback, this, std::placeholders::_1)
  );
}

void Node::EstopCallback(const EstopMsgType::SharedPtr msg)
{
  const bool estop_triggered{msg->data};
  if (estop_triggered) {
    state_machine_->ProcessEvent(EstopTriggered{});
  } else {
    state_machine_->ProcessEvent(EstopCleared{});
  }
}
void Node::ProximityDataCallback(const ProximityMsgType::SharedPtr msg)
{
  state_machine_->ProcessEvent(ProximityData{msg->data});
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<analog::speed_limiter::Node>());
  rclcpp::shutdown();
  return 0;
}
