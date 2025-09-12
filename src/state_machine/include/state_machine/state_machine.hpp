#pragma once

#include "i_state.hpp"
#include <array>

namespace analog::sm
{

/**
 * @brief Implements reusable logic for event-driven state machines.
 * @tparam StateIdT Type of state identifier (e.g., enum class)
 * @tparam EventContainerT Container representing event types that the state
 * machine can process (e.g., std::variant, union)
 */

template <class StateIdT, class EventContainerT>
class StateMachine final
{
    public:
        using StateT = IState<StateIdT, EventContainerT>;
        using StateArray = std::array<StateT *const, 
            static_cast<std::size_t>(StateIdT::num_state_ids)>;

    /**
     * @brief Construct a new state machine
     * @param states Array of pointers to state instances representing all
     * possible states for this state machine
     */
    StateMachine(StateArray&& /*states*/)
    {

    }

    /**
     * @brief Process an event. This may trigger entry into a new state.
     * @param event_container The event to process
     */
    void ProcessEvent(const EventContainerT& /*event_container*/)
    {

    }
};
}