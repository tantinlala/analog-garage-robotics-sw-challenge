#include "i_publisher.hpp"
#include "speed_limiter_states.hpp"

namespace analog::speed_limiter 
{

// BaseState Implementation

BaseState::BaseState(StateId id, PublisherPtr publisher)
    : State(id), publisher_(publisher) {}

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

StateId BaseState::Handle(const EstopSet /*event*/)
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

NotEstoppedState::NotEstoppedState(StateId id, std::optional<ProximityBoundary> min,
    std::optional<ProximityBoundary> max, PublisherPtr publisher) : 
    BaseState(id, publisher), min_boundary_(min), max_boundary_(max) {}

StateId NotEstoppedState::Handle(const EstopCleared /*event*/)
{
    return this->GetStateId();
}

StateId NotEstoppedState::Handle(const ProximityData event)
{
    if (min_boundary_.has_value() && 
        (event.distance < min_boundary_.value().distance))
    {
        return min_boundary_.value().boundary_state;
    }

    if (max_boundary_.has_value() && 
        (event.distance > max_boundary_.value().distance))
    {
        return max_boundary_.value().boundary_state;
    }

    return this->GetStateId();
}

}
