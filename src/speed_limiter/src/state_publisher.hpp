#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "i_publisher.hpp"
#include "speed_limiter_states.hpp"

namespace analog::speed_limiter 
{

class StatePublisher : public IPublisher<StateId>
{
    public:
        StatePublisher() {}

        void SetPublisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
        {
            this->publisher_ = publisher;
            if (this->last_state_.has_value())
            {
                this->HandleState(this->last_state_.value());
            }
        }

        void SetLogger(rclcpp::Logger& logger)
        {
            this->logger_ = &logger;
        }

        void Publish(const StateId state) override
        {
            if (publisher_)
            {
                this->HandleState(state);
            }
            else
            {
                last_state_ = state;
            }
        }

    private:
        rclcpp::Logger* logger_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std::optional<StateId> last_state_;

        const char* ToString(const StateId state)
        {
            switch (state)
            {
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

        void HandleState(const StateId state)
        {
            auto message = std_msgs::msg::String();
            const char* state_string(this->ToString(state));

            if (this->logger_ != nullptr)
            {
                RCLCPP_INFO(*this->logger_, "Publishing %s", state_string);
            }

            message.data = state_string;
            publisher_->publish(message);
        }
};

}