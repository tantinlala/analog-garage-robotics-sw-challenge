#include "i_publisher.hpp"
#include "speed_limiter_states.hpp"

namespace analog::speed_limiter 
{

// BaseState Implementation

BaseState::BaseState(StateId id, PublisherPtr publisher)
    : state_id_(id), publisher_(publisher) {}


StateId BaseState::GetStateId()
{
    return state_id_;
}

void BaseState::Enter()
{
    if (publisher_)
    {
        publisher_->Publish(this->GetStateId());
    }
}

StateId BaseState::Process(const Events& event)
{
    return std::visit([this](auto& event) {
        return this->Handle(event); }, event);
}

StateId BaseState::Handle(const EstopTriggered /*event*/)
{
    return StateId::ESTOPPED;
}

// EstoppedState Implementation

EstoppedState::EstoppedState(PublisherPtr publisher) 
    : BaseState(StateId::ESTOPPED, publisher) {}

StateId EstoppedState::Handle(const EstopCleared /*event*/)
{
    return StateId::STOP;
}

StateId EstoppedState::Handle(const ProximityData /*event*/)
{
    return StateId::ESTOPPED;
}

// NotEstoppedState Implementation

NotEstoppedState::NotEstoppedState(StateId id, const Params& params, PublisherPtr publisher) : 
    BaseState(id, publisher), params_(&params)
{
    // TODO: check whether boundaries are ordered as required

}

StateId NotEstoppedState::Handle(const EstopCleared /*event*/)
{
    return this->GetStateId();
}

StateId NotEstoppedState::Handle(const ProximityData event)
{
    float distance{event.distance};
    float hysteresis{this->params_->hysteresis};
    StateId current_state{this->GetStateId()};
    bool passed_current_state{false};
    for (auto &boundary : this->params_->boundaries)
    {
        passed_current_state |= (boundary.boundary_state == current_state);
        if (passed_current_state)
        {
            if (distance < boundary.distance + hysteresis)
            {
                return boundary.boundary_state;
            }
        }
        else if (distance <= boundary.distance)
        {
            return boundary.boundary_state;
        }
    }

    return this->GetStateId();
}

}
