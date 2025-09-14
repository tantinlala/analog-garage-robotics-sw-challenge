#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "i_publisher.hpp"
#include "speed_limiter_states.hpp"

namespace analog::speed_limiter 
{

class SpeedStatePublisher : public IPublisher<StateId>
{
    public:
        SpeedStatePublisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher,
            rclcpp::Logger& logger);

        void Publish(const StateId state);

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Logger* logger_;
        std::optional<StateId> last_state_;

        const char* ToString(const StateId state);
};

}