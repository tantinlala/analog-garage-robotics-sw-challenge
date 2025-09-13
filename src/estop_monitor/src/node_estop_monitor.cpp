#include <cstdio>
#include <rclcpp/parameter.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rmw/types.h>
#include <std_msgs/msg/bool.hpp>

namespace analog::estop_monitor
{

class Node : public rclcpp::Node 
{
    public:
        Node()
          : rclcpp::Node("estop_monitor")
        {
            // Set up timer
            this->declare_parameter(kTriggerTimeName, 10'000);
            rclcpp::Parameter time_param = this->get_parameter(kTriggerTimeName);
            std::chrono::milliseconds duration_ms{time_param.as_int()};
            this->timer_ = this->create_wall_timer(duration_ms, std::bind(&Node::TimerCallback, this));

            rclcpp::QoS estop_qos{rclcpp::SystemDefaultsQoS()};
            estop_qos.keep_last(1).transient_local().reliable();
            this->publisher_ = this->create_publisher<std_msgs::msg::Bool>("analog/estop_triggered", estop_qos);

            auto message {std_msgs::msg::Bool()};
            message.data = false;
            this->publisher_->publish(message);
        }

    private:
        static constexpr const char * kTriggerTimeName{"trigger_time_ms"};
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

        void TimerCallback()
        {
            this->timer_->cancel();
            auto message {std_msgs::msg::Bool()};
            message.data = true;
            this->publisher_->publish(message);
        }
};

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<analog::estop_monitor::Node>());
    rclcpp::shutdown();
    return 0;
}
