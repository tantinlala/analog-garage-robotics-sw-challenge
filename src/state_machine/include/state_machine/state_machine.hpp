#pragma once

#include <memory>
#include <array>
#include <optional>
#include <stdexcept>
#include "i_state.hpp"

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
    using StateArray = std::array<std::shared_ptr<StateT>, 
        static_cast<std::size_t>(StateIdT::num_state_ids)>;

    /**
     * @brief Construct a new state machine
     * @details The constructor will throw an exception if any state element is nullptr
     * or if the state IDs are not consecutive starting from zero. This also verifies
     * that each element has a unique state ID.
     * @param states Array of pointers to state instances representing all
     * possible states for this state machine
     */
    StateMachine(StateArray&& states) : states_(states)
    {
        std::optional<std::size_t> last_index;
        for (auto & state : this->states_)
        {
            if (state == nullptr)
            {
                throw std::runtime_error("StateMachine: State element is nullptr");
            }

            const auto index{static_cast<std::size_t>(state->GetStateId())};

            if (!last_index.has_value() &&
                (index != 0))
            {
                throw std::runtime_error("StateMachine: State IDs do not start with 0");
            }

            if (last_index.has_value() &&
                (index != (last_index.value() + 1)))
            {
                throw std::runtime_error("StateMachine: State IDs are not consecutive");
            }

            last_index = index;
        }

        this->current_index_ = 0;
        this->states_.at(0).get()->Enter();
    }

    /**
     * @brief Process an event. This may trigger entry into a new state.
     * @details This is non-reentrant and is meant to be processed to completion on a single
     * thread.
     * @param event_container The event to process
     */
    void ProcessEvent(const EventContainerT& event_container)
    {
        StateT* current_state{this->GetState(this->current_index_)};
        const auto next_index{static_cast<std::size_t>(current_state->Process(event_container))};
        if (next_index != this->current_index_)
        {
            this->current_index_ = next_index;
            StateT* next_state{this->GetState(this->current_index_)};
            next_state->Enter();
        }
    }

    private:

    StateArray states_;
    std::size_t current_index_{0};

    [[nodiscard]] StateT* GetState(const std::size_t index)
    {
        if (index >= this->states_.size())
        {
            throw std::out_of_range("StateMachine: Invalid state index");
        }

        return this->states_.at(index).get();
    }
};
}