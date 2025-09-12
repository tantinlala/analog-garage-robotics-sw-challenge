#pragma once

#include <type_traits>

namespace analog::sm
{

/**
 * @brief State base class for event-driven state machine
 * @tparam StateIdT Type of state identifier (e.g., enum class)
 * @tparam EventContainerT Container representing event types that the state
 * machine can process (e.g., std::variant, union)
 */
template <class StateIdT, class EventContainerT>
class IState
{
    static_assert(std::is_enum_v<StateIdT>, "StateIdT must be an enum");

    public:

    /**
     * @brief Destroy the State object
     */
    virtual ~IState() = default;

    /**
     * @brief Get the ID of this State
     * @return StateIdT The state identifier
     */
    virtual StateIdT GetStateId() = 0;

    /**
     * @brief Called when the parent state machine enters this state
     */
    virtual void Enter() = 0;

    /**
     * @brief Called when the parent state machine has an event ready for
     processing
     * @details Derived classes must implement this method. Handling a generic
     event container guarantees that each State can handle every event type.
     Implementations may choose to ignore certain event types within the
     container.
     *
     * @param event_container The event to be handled
     * @return StateIdT the state that the state machine should transition to
     */
    [[nodiscard]] virtual StateIdT Process(
        const EventContainerT& event_container) = 0;
};

}