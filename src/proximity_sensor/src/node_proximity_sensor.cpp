#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32.hpp>
#include "list_distance_source.hpp"
#include "node_proximity_sensor.hpp"

namespace analog::proximity_sensor
{

Node::Node()
: rclcpp::Node("proximity_sensor"),
  publisher_(this->create_publisher<std_msgs::msg::Float32>(
      "analog/proximity_data",
      rclcpp::SensorDataQoS()))
{
  this->timer_ = this->SetupTimer();
  this->distance_source_ = this->SetupDistanceSource();
}

rclcpp::TimerBase::SharedPtr Node::SetupTimer()
{
  this->declare_parameter(kSampleTimeParamName, 500);
  rclcpp::Parameter time_param = this->get_parameter(kSampleTimeParamName);
  std::chrono::milliseconds duration_ms{time_param.as_int()};
  return this->create_wall_timer(duration_ms, std::bind(&Node::TimerCallback, this));
}

std::unique_ptr<ListDistanceSource> Node::SetupDistanceSource()
{
  // Set up distance series
  this->declare_parameter(
    kDistanceSeriesParamName,
    std::vector<float>({850, 450, 200}));
  rclcpp::Parameter series_param = this->get_parameter(kDistanceSeriesParamName);
  std::vector<double> distance_series = series_param.as_double_array();
  return std::make_unique<ListDistanceSource>(distance_series);
}

void Node::TimerCallback()
{
  auto message {ProximityMsgType()};
  auto distance{distance_source_->GetDistance()};

  if (distance.has_value()) {
    message.data = distance.value();
    this->publisher_->publish(message);
  }
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<analog::proximity_sensor::Node>());
  rclcpp::shutdown();
  return 0;
}
