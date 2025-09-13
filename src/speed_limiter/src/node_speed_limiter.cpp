#include <cstdio>
#include <optional>
#include <rclcpp/qos.hpp>
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
            publisher_(this->create_publisher<std_msgs::msg::String>("analog/speed_state", kQueueDepth)),
            state_publisher_(std::make_shared<StatePublisher>(publisher_)),
            estopped_state_(state_publisher_),
            stop_state_(
                StateId::STOP, 
                params_,
                state_publisher_
            ),
            slow_state_{
                StateId::SLOW,
                params_,
                state_publisher_,
            },
            full_speed_state_{
                StateId::FULL_SPEED,
                params_,
                state_publisher_,
            },
            state_machine_{{&estopped_state_, &stop_state_, &slow_state_, &full_speed_state_}}
        {

            rclcpp::QoS estop_qos{rclcpp::SystemDefaultsQoS()};
            estop_qos.keep_last(1).transient_local().reliable();

            this->estop_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
                "analog/estop_triggered", 
                estop_qos, 
                std::bind(&Node::EstopCallback, this, std::placeholders::_1)
            );

            this->proximity_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
                "analog/proximity_data", 
                rclcpp::SensorDataQoS(), 
                std::bind(&Node::ProximityDataCallback, this, std::placeholders::_1)
            );
        }

    private:
        static constexpr int kQueueDepth{10}; // TODO: choose this

        NotEstoppedState::Params params_ = { 
            { { StateId::STOP, 400.0}, {StateId::SLOW, 800.0}, {StateId::FULL_SPEED, std::numeric_limits<float>::infinity()} },
            50.0,
        };

        void EstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            const bool estop_triggered{msg->data};
            if (estop_triggered)
            {
                state_machine_.ProcessEvent(EstopTriggered{});
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
