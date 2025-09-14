#include <cstdio>
#include <optional>
#include <rclcpp/logger.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "state_machine/state_machine.hpp"
#include "speed_limiter_states.hpp"
#include "speed_state_publisher.hpp"

namespace analog::speed_limiter 
{

class Node : public rclcpp::Node 
{
    public:
        Node() : rclcpp::Node("speed_limiter"),
            logger_(this->get_logger())
        {
            rclcpp::QoS speed_state_qos{rclcpp::SystemDefaultsQoS()};
            speed_state_qos.keep_last(2).transient_local().reliable();
            auto publisher = this->create_publisher<std_msgs::msg::String>("analog/speed_state", speed_state_qos);
            auto state_publisher {std::make_shared<SpeedStatePublisher>(publisher, this->logger_)};

            auto estopped_state = std::make_shared<EstoppedState>(state_publisher);
            auto stop_state = std::make_shared<NotEstoppedState>(
                StateId::STOP, 
                params_,
                state_publisher
            );

            auto slow_state = std::make_shared<NotEstoppedState>(
                StateId::SLOW,
                params_,
                state_publisher
            );
            
            auto full_speed_state = std::make_shared<NotEstoppedState>(
                StateId::FULL_SPEED,
                params_,
                state_publisher
            );

            this->state_machine_ = std::make_unique<SpeedStateMachine>(
                SpeedStateMachine::StateArray{
                    estopped_state,
                    stop_state,
                    slow_state,
                    full_speed_state
                }
            );

            rclcpp::QoS estop_qos{rclcpp::SystemDefaultsQoS()};
            estop_qos.keep_last(2).transient_local().reliable();

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

            this->DeclareBoundaryParameter(kStopBoundaryName, StateId::STOP, 400.0);
            this->DeclareBoundaryParameter(kSlowBoundaryName, StateId::SLOW, 800.0);

            this->declare_parameter(kHysteresisName, 50.0);
            rclcpp::Parameter hysteresis = this->get_parameter(kHysteresisName);
            this->params_.hysteresis = hysteresis.as_double();
        }

    private:
        using SpeedStateMachine = sm::StateMachine<StateId, Events>;

        static constexpr const char * kStopBoundaryName{"stop_boundary"};
        static constexpr const char * kSlowBoundaryName{"slow_boundary"};
        static constexpr const char * kHysteresisName{"hysteresis"};

        NotEstoppedState::Params params_ = { 
            { { StateId::STOP, 0.0},
                {StateId::SLOW, 0.0},
                {StateId::FULL_SPEED, std::numeric_limits<float>::infinity()} },
            0.0,
        };

        void EstopCallback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            const bool estop_triggered{msg->data};
            if (estop_triggered)
            {
                state_machine_->ProcessEvent(EstopTriggered{});
            }
            else
            {
                state_machine_->ProcessEvent(EstopCleared{});
            }
        }

        void ProximityDataCallback(const std_msgs::msg::Float32::SharedPtr msg)
        {
            state_machine_->ProcessEvent(ProximityData{msg->data});
        }

        void DeclareBoundaryParameter(const char * name, const StateId state, const float default_value)
        {
            this->declare_parameter(name, default_value);
            rclcpp::Parameter boundary_distance = this->get_parameter(name);
            auto boundary{std::find_if(this->params_.boundaries.begin(), 
                this->params_.boundaries.end(),
                [state](auto & boundary) {return boundary.state == state;})};
            boundary->distance = boundary_distance.as_double();
        }

        rclcpp::Logger logger_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr proximity_subscription_;
        std::unique_ptr<SpeedStateMachine> state_machine_;
};

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<analog::speed_limiter::Node>());
    rclcpp::shutdown();
    return 0;
}
