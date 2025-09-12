#pragma once

#include <gmock/gmock.h>
#include <variant>

#include "state_machine/state.hpp"

namespace analog::sm 
{

enum class TestStateId 
{
    STATE_A,
    STATE_B,
    num_state_ids
};

using TestEventX = int;
using TestEventY = double;

using TestEventContainer = std::variant<TestEventX, TestEventY>;

class MockState : public State<TestStateId, TestEventContainer>
{
    public:
        MockState(TestStateId state_id) : State(state_id) {};
        MOCK_METHOD(void, Enter, (), (override));
        MOCK_METHOD(TestStateId, Process, 
            (const TestEventContainer& event_container), (override));
};

}