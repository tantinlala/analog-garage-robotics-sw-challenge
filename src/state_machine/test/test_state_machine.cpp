#include <gtest/gtest.h>
#include <stdexcept>
#include <memory>

#include "mocks/mock_state.hpp"
#include "state_machine/state_machine.hpp"

using testing::Test;
using testing::Return;

namespace analog::sm
{

class StateMachineTest : public Test
{
protected:
  using FakeStateMachine = StateMachine<TestStateId, TestEventContainer>;
  std::shared_ptr<MockState> state_a;
  std::shared_ptr<MockState> state_b;

  void SetUp() override
  {
    state_a = std::make_shared<MockState>();
    state_b = std::make_shared<MockState>();
  }
};

TEST_F(StateMachineTest, When_ConstructedProperly_Expect_FirstStateIsEntered)
{
  EXPECT_CALL(*this->state_a, Enter()).Times(1);
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->state_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  EXPECT_NO_THROW(FakeStateMachine({this->state_a, this->state_b}));
}

TEST_F(StateMachineTest, When_StateIdNonConsecutive_Expect_Throw)
{
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_b));
  EXPECT_THROW(FakeStateMachine({this->state_a, this->state_b}), std::runtime_error);
}

TEST_F(StateMachineTest, When_StateElementIsNullptr_Expect_Throw)
{
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_THROW(FakeStateMachine({this->state_a, nullptr}), std::runtime_error);
}

TEST_F(StateMachineTest, When_StateIdDuplicate_Expect_Throw)
{
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->state_b, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_THROW(FakeStateMachine({this->state_a, this->state_b}), std::runtime_error);
}

TEST_F(StateMachineTest, When_ProcessReturnsCurrentState_Expect_DoNothing)
{
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->state_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{this->state_a, this->state_b}};

  const TestEventY event{1.0};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->state_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_a));

  EXPECT_CALL(*this->state_a, Enter()).Times(0);
  EXPECT_CALL(*this->state_b, Enter()).Times(0);

  state_machine.ProcessEvent(event_container);
}

TEST_F(StateMachineTest, When_ProcessReturnsInvalidState_Expect_Throw)
{
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->state_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{this->state_a, this->state_b}};

  const TestEventY event{1.0};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->state_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(static_cast<TestStateId>(999)));

  EXPECT_THROW(state_machine.ProcessEvent(event_container), std::out_of_range);
}

TEST_F(StateMachineTest, When_ProcessReturnsNewState_Expect_EnterNewState)
{
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->state_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{this->state_a, this->state_b}};

  const TestEventX event{-1};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->state_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_b));

  EXPECT_CALL(*this->state_a, Enter()).Times(0);
  EXPECT_CALL(*this->state_b, Enter()).Times(1);

  state_machine.ProcessEvent(event_container);
}

TEST_F(StateMachineTest, Given_PreviousStateChange_When_ProcessReturnsNewState_Expect_EnterNewState)
{
  EXPECT_CALL(*this->state_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->state_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{this->state_a, this->state_b}};

  const TestEventX event{-1};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->state_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_b));

  state_machine.ProcessEvent(event_container);

  EXPECT_CALL(*this->state_b, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_a));

  EXPECT_CALL(*this->state_a, Enter()).Times(1);
  EXPECT_CALL(*this->state_b, Enter()).Times(0);
  state_machine.ProcessEvent(event_container);
}

}
