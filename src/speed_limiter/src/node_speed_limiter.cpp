#include <cstdio>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "state_machine/state_machine.hpp"
#include "speed_limiter_states.hpp"
#include "state_publisher.hpp"

namespace analog::speed_limiter 
{

class Node : public rclcpp::Node 
{
    public:
        Node() : rclcpp::Node("speed_limiter"),
            publisher_(this->create_publisher<std_msgs::msg::String>("speed_state", kQueueDepth)),
            state_publisher_(std::make_shared<StatePublisher>(publisher_)),
            estopped_state_(state_publisher_),
            stop_state_(
                StateId::STOP, 
                std::nullopt,
                NotEstoppedState::ProximityBoundary{StateId::SLOW, kDefaultStopThreshold + kDefaultHysteresis},
                state_publisher_
            ),
            slow_state_{
                StateId::SLOW,
                NotEstoppedState::ProximityBoundary{StateId::STOP, kDefaultStopThreshold},
                NotEstoppedState::ProximityBoundary{StateId::FULL_SPEED, kDefaultFullSpeedThreshold + kDefaultHysteresis},
                state_publisher_,
            },
            full_speed_state_{
                StateId::FULL_SPEED,
                NotEstoppedState::ProximityBoundary{StateId::STOP, kDefaultFullSpeedThreshold},
                std::nullopt,
                state_publisher_,
            },
            state_machine_{{&estopped_state_, &stop_state_, &slow_state_, &full_speed_state_}}
        {
            this->estop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "estop_cleared", 
                kQueueDepth, 
                std::bind(&Node::EstopCallback, this, std::placeholders::_1)
            );

            this->proximity_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
                "proximity_data", 
                kQueueDepth, 
                std::bind(&Node::ProximityDataCallback, this, std::placeholders::_1)
            );
        }

    private:
        static constexpr int kQueueDepth{10}; // TODO: choose this

        void EstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            if (msg->data)
            {
                state_machine_.ProcessEvent(EstopSet{});
            }
            else
            {
                state_machine_.ProcessEvent(EstopCleared{});
            }
        }

        void ProximityDataCallback(const std_msgs::msg::Float32::SharedPtr msg)
        {
            state_machine_.ProcessEvent(ProximityData{msg->data});
        }

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr proximity_subscription_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std::shared_ptr<StatePublisher> state_publisher_;

        static constexpr float kDefaultStopThreshold{400.0f};
        static constexpr float kDefaultFullSpeedThreshold{800.0f};
        static constexpr float kDefaultHysteresis{50.0f};

        EstoppedState estopped_state_;
        NotEstoppedState stop_state_;
        NotEstoppedState slow_state_;
        NotEstoppedState full_speed_state_;
        sm::StateMachine<StateId, Events> state_machine_;
};

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<analog::speed_limiter::Node>());
    rclcpp::shutdown();
    return 0;
}
