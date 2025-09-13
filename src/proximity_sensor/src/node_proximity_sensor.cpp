#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32.hpp>
#include "list_distance_source.hpp"

namespace analog::proximity_sensor
{

class Node : public rclcpp::Node 
{
    public:
        Node()
          : rclcpp::Node("proximity_sensor"),
            publisher_(this->create_publisher<std_msgs::msg::Float32>("analog/proximity_data", kQueueDepth))
        {
          // Set up timer
          this->declare_parameter(kSampleTimeParamName, 500);
          rclcpp::Parameter time_param = this->get_parameter(kSampleTimeParamName);
          std::chrono::milliseconds duration_ms{time_param.as_int()};
          this->timer_ = this->create_wall_timer(duration_ms, std::bind(&Node::TimerCallback, this)),

          // Set up distance series
          this->declare_parameter(kDistanceSeriesParamName, 
            std::vector<float>({850, 450, 200}));
          rclcpp::Parameter series_param = this->get_parameter(kDistanceSeriesParamName);
          std::vector<double> distance_series = series_param.as_double_array();
          distance_source_ = std::make_unique<ListDistanceSource>(distance_series);
        }

    private:
        static constexpr int kQueueDepth{10}; // TODO: choose this
        static constexpr const char * kSampleTimeParamName{"sample_time_ms"};
        static constexpr const char * kDistanceSeriesParamName{"distance_series"};
        std::unique_ptr<ListDistanceSource> distance_source_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

        void TimerCallback()
        {
            auto message {std_msgs::msg::Float32()};
            auto distance{distance_source_->GetDistance()};

            if (distance.has_value())
            {
              message.data = distance.value();
              this->publisher_->publish(message);
            }
        }
};

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<analog::proximity_sensor::Node>());
    rclcpp::shutdown();
    return 0;
}
