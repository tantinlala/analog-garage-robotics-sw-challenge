#include "speed_state_publisher.hpp"
#include "speed_limiter_states.hpp"

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
    const char * state_string(state_id_to_string(state));
    RCLCPP_INFO(*this->logger_, "Publishing %s", state_string);
    message.data = state_string;
    publisher_->publish(message);
  }
}

}
