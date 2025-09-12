#include <gtest/gtest.h>

#include "mocks/mock_state.hpp"
#include "state_machine/state_machine.hpp"

using testing::Test;
using testing::Return;

namespace analog::sm
{

class StateMachineTest : public Test
{
    protected:
        MockState state_a{TestStateId::STATE_A};
        MockState state_b{TestStateId::STATE_B};
};  

TEST_F(StateMachineTest, When_Constructed_Expect_FirstStateIsEntered)
{
    EXPECT_CALL(this->state_a, Enter()).Times(1);
    StateMachine<TestStateId, TestEventContainer> state_machine
        {{&this->state_a, &this->state_b}};
}

TEST_F(StateMachineTest, When_ProcessReturnsCurrentState_Expect_DoNothing)
{
    StateMachine<TestStateId, TestEventContainer> state_machine
        {{&this->state_a, &this->state_b}};

    const TestEventY event{1.0};
    const TestEventContainer event_container{event};
    const auto current_state_id{this->state_a.GetStateId()};

    EXPECT_CALL(this->state_a, Process(event_container))
        .Times(1)
        .WillOnce(Return(current_state_id));

    EXPECT_CALL(this->state_a, Enter()).Times(0);
    EXPECT_CALL(this->state_b, Enter()).Times(0);

    state_machine.ProcessEvent(event_container);
}

TEST_F(StateMachineTest, When_ProcessReturnsNewState_Expect_EnterNewState)
{
    StateMachine<TestStateId, TestEventContainer> state_machine
        {{&this->state_a, &this->state_b}};

    const TestEventX event{-1};
    const TestEventContainer event_container{event};
    const auto new_state_id{this->state_b.GetStateId()};

    EXPECT_CALL(this->state_b, Process(event_container))
        .Times(1)
        .WillOnce(Return(new_state_id));

    EXPECT_CALL(this->state_a, Enter()).Times(0);
    EXPECT_CALL(this->state_b, Enter()).Times(1);

    state_machine.ProcessEvent(event_container);
}

}