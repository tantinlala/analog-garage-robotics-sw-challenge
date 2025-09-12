#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "speed_limiter/i_publisher.hpp"
#include "speed_limiter/speed_limiter_states.hpp"

namespace analog::speed_limiter 
{

class StatePublisher : public IPublisher<StateId>
{
    public:
        explicit StatePublisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
            : publisher_(publisher) {}

        void Publish(const StateId state) override
        {
            if (publisher_)
            {
                auto message = std_msgs::msg::String();
                switch (state)
                {
                    case StateId::FULL_SPEED:
                        message.data = "FULL_SPEED";
                        break;
                    case StateId::SLOW:
                        message.data = "SLOW";
                        break;
                    case StateId::STOP:
                        message.data = "STOP";
                        break;
                    case StateId::ESTOPPED:
                        message.data = "ESTOPPED";
                        break;
                    default:
                        message.data = "UNKNOWN_STATE";
                        break;
                }
                publisher_->publish(message);
            }
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

}