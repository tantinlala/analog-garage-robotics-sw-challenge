#pragma once

#include <memory>
#include <variant>
#include <optional>
#include "state_machine/i_state.hpp"
#include "i_publisher.hpp"

namespace analog::speed_limiter
{

/**
 * @brief Identifiers for the different states in the speed limiter state machine.
 */
enum class StateId 
{
    FULL_SPEED,
    SLOW,
    STOP,
    ESTOPPED,
    num_state_ids
};

/**
 * @brief Estop clear event
 */
struct EstopCleared {};

/**
 * @brief Estop set event
 */
struct EstopSet {};
/**
 * @brief Proximity data event
 */
struct ProximityData
{
    float distance;
};

using Events = std::variant<EstopCleared, EstopSet, ProximityData>;

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
        StateId Handle(const EstopSet event);
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

        NotEstoppedState(StateId id, std::optional<ProximityBoundary> min,
            std::optional<ProximityBoundary> max, PublisherPtr publisher);

    private:
        std::optional<ProximityBoundary> min_boundary_;
        std::optional<ProximityBoundary> max_boundary_;

        StateId Handle(const EstopCleared event) override;
        StateId Handle(const ProximityData event) override;
};

}