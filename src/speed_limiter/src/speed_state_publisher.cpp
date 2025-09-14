#include "speed_state_publisher.hpp"

namespace analog::speed_limiter
{
SpeedStatePublisher::SpeedStatePublisher(
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
  rclcpp::Logger & logger)
: publisher_(publisher), logger_(&logger)
{

}

void SpeedStatePublisher::Publish(const StateId state)
{
  if (publisher_) {
    auto message = std_msgs::msg::String();
    const char * state_string(this->ToString(state));
    RCLCPP_INFO(*this->logger_, "Publishing %s", state_string);
    message.data = state_string;
    publisher_->publish(message);
  }
}

const char * SpeedStatePublisher::ToString(const StateId state)
{
  switch (state) {
    case StateId::FULL_SPEED:
      return "FULL_SPEED";
    case StateId::SLOW:
      return "SLOW";
    case StateId::STOP:
      return "STOP";
    case StateId::ESTOPPED:
      return "ESTOPPED";
    default:
      return "UNKNOWN_STATE";
  }
}

}
