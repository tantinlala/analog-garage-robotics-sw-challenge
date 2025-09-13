#include "i_publisher.hpp"
#include "speed_limiter_states.hpp"
#include <algorithm>
#include <stdexcept>
#include <set>

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
    BaseState(id, publisher)
{
    this->CheckParams(params);
    this->params_ = &params;
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

void NotEstoppedState::CheckParams(const Params& params)
{
    // Check whether sorted
    if (!std::is_sorted(params.boundaries.begin(), 
        params.boundaries.end(), 
        [](ProximityBoundary a, ProximityBoundary b) {
        return a.distance < b.distance;
    }))
    {
        throw std::runtime_error("NotEstoppedState: proximity boundaries should be in ascending order.");
    }

    // Check for allowable hysteresis value
    if (params.hysteresis < 0)
    {
        throw std::runtime_error("NotEstoppedState: hysteresis can't be negative.");
    }

    // Check for duplicates
    std::set<StateId> state_set;
    for (auto & boundary : params.boundaries)
    {
        state_set.insert(boundary.boundary_state);
    }
    if (state_set.size() != params.boundaries.size())
    {
        throw std::runtime_error("NotEstoppedState: duplicate states found in boundaries.");
    }

    // Check for allowed state values
    if (state_set.find(StateId::ESTOPPED) != state_set.end())
    {
        throw std::runtime_error("NotEstoppedState: ESTOPPED cannot be used as boundary.");
    }

    if (state_set.find(StateId::num_state_ids) != state_set.end())
    {
        throw std::runtime_error("NotEstoppedState: num_state_ids cannot be used as boundary.");
    }
}

}
