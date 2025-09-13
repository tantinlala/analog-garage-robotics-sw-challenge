#pragma once

#include <memory>
#include <variant>
#include <optional>
#include <vector>
#include "state_machine/i_state.hpp"
#include "i_publisher.hpp"

namespace analog::speed_limiter
{

/**
 * @brief Identifiers for the different states in the speed limiter state machine.
 */
enum class StateId 
{
    ESTOPPED,
    STOP,
    SLOW,
    FULL_SPEED,
    num_state_ids
};

/**
 * @brief Estop clear event
 */
struct EstopCleared {};

/**
 * @brief Estop set event
 */
struct EstopTriggered {};
/**
 * @brief Proximity data event
 */
struct ProximityData
{
    float distance;
};

using Events = std::variant<EstopCleared, EstopTriggered, ProximityData>;

/**
 * @brief Base class for all states in the speed limiter state machine.
 */
class BaseState : public sm::IState<StateId, Events>
{
    public:
        using PublisherPtr = std::shared_ptr<IPublisher<StateId>>;
        BaseState(StateId id, PublisherPtr publisher);

    protected:
        StateId GetStateId() override;

    private:
        void Enter() override;
        StateId Process(const Events& event) override;
        StateId Handle(const EstopTriggered event);
        virtual StateId Handle(const EstopCleared event) = 0;
        virtual StateId Handle(const ProximityData event) = 0;

        StateId state_id_;
        PublisherPtr publisher_;
};

/**
 * @brief State entered when emergency stop is pressed.
 */
class EstoppedState : public BaseState
{
    public:
        EstoppedState(PublisherPtr publisher);

    private:
        StateId Handle(const EstopCleared event) override;
        StateId Handle(const ProximityData event) override;
};

/**
 * @brief Class for handling states when not estopped (FULL_SPEED, SLOW, STOP).
 */
class NotEstoppedState : public BaseState 
{
    public:
        struct ProximityBoundary
        {
            StateId boundary_state;
            float distance;
        };

        struct Params 
        {
            std::vector<ProximityBoundary> boundaries;
            float hysteresis;
        };

        NotEstoppedState(StateId id, const Params& params, PublisherPtr publisher);

    private:
        const Params* params_;

        StateId Handle(const EstopCleared event) override;
        StateId Handle(const ProximityData event) override;
};

}