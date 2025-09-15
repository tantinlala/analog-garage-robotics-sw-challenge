#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/bool.hpp>
#include "node_estop_monitor.hpp"

namespace analog::estop_monitor
{

Node::Node()
: rclcpp::Node("estop_monitor")
{
  this->timer_ = this->SetupTimer();
  this->publisher_ = this->SetupPublisher();

  auto message {EstopMsgType()};
  message.data = false;
  this->publisher_->publish(message);
}

rclcpp::TimerBase::SharedPtr Node::SetupTimer()
{
  this->declare_parameter(kTriggerTimeName, 10'000);
  rclcpp::Parameter time_param = this->get_parameter(kTriggerTimeName);
  std::chrono::milliseconds duration_ms{time_param.as_int()};
  return this->create_wall_timer(duration_ms, std::bind(&Node::TimerCallback, this));
}

rclcpp::Publisher<Node::EstopMsgType>::SharedPtr Node::SetupPublisher()
{
  rclcpp::QoS estop_qos{rclcpp::SystemDefaultsQoS()};
  estop_qos.keep_last(kEstopDepth).transient_local().reliable();
  return this->create_publisher<EstopMsgType>(
    "analog/estop_triggered",
    estop_qos);
}

void Node::TimerCallback()
{
  this->timer_->cancel();
  auto message {EstopMsgType()};
  message.data = true;
  this->publisher_->publish(message);
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<analog::estop_monitor::Node>());
  rclcpp::shutdown();
  return 0;
}
