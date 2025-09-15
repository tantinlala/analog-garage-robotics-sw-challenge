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
  std::unique_ptr<MockState> state_a;
  std::unique_ptr<MockState> state_b;

  MockState * raw_ptr_a;
  MockState * raw_ptr_b;

  void SetUp() override
  {
    state_a = std::make_unique<MockState>();
    state_b = std::make_unique<MockState>();

    raw_ptr_a = state_a.get();
    raw_ptr_b = state_b.get();
  }
};

TEST_F(StateMachineTest, When_ConstructedProperly_Expect_FirstStateIsEntered)
{
  EXPECT_CALL(*this->raw_ptr_a, Enter()).Times(1);
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->raw_ptr_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  EXPECT_NO_THROW(FakeStateMachine({std::move(this->state_a), std::move(this->state_b)}));
}

TEST_F(StateMachineTest, When_StateIdNonConsecutive_Expect_Throw)
{
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_b));
  EXPECT_THROW(
    FakeStateMachine(
      {std::move(this->state_a), std::move(
          this->state_b)}), std::runtime_error);
}

TEST_F(StateMachineTest, When_StateElementIsNullptr_Expect_Throw)
{
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_THROW(FakeStateMachine({std::move(this->state_a), nullptr}), std::runtime_error);
}

TEST_F(StateMachineTest, When_StateIdDuplicate_Expect_Throw)
{
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->raw_ptr_b, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_THROW(
    FakeStateMachine(
      {std::move(this->state_a), std::move(
          this->state_b)}), std::runtime_error);
}

TEST_F(StateMachineTest, When_ProcessReturnsCurrentState_Expect_DoNothing)
{
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->raw_ptr_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{std::move(this->state_a), std::move(this->state_b)}};

  const TestEventY event{1.0};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->raw_ptr_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_a));

  EXPECT_CALL(*this->raw_ptr_a, Enter()).Times(0);
  EXPECT_CALL(*this->raw_ptr_b, Enter()).Times(0);

  state_machine.ProcessEvent(event_container);
}

TEST_F(StateMachineTest, When_ProcessReturnsInvalidState_Expect_Throw)
{
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->raw_ptr_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{std::move(this->state_a), std::move(this->state_b)}};

  const TestEventY event{1.0};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->raw_ptr_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(static_cast<TestStateId>(999)));

  EXPECT_THROW(state_machine.ProcessEvent(event_container), std::out_of_range);
}

TEST_F(StateMachineTest, When_ProcessReturnsNewState_Expect_EnterNewState)
{
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->raw_ptr_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{std::move(this->state_a), std::move(this->state_b)}};

  const TestEventX event{-1};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->raw_ptr_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_b));

  EXPECT_CALL(*this->raw_ptr_a, Enter()).Times(0);
  EXPECT_CALL(*this->raw_ptr_b, Enter()).Times(1);

  state_machine.ProcessEvent(event_container);
}

TEST_F(StateMachineTest, Given_PreviousStateChange_When_ProcessReturnsNewState_Expect_EnterNewState)
{
  EXPECT_CALL(*this->raw_ptr_a, GetStateId()).WillOnce(Return(TestStateId::state_a));
  EXPECT_CALL(*this->raw_ptr_b, GetStateId()).WillOnce(Return(TestStateId::state_b));
  FakeStateMachine state_machine {{std::move(this->state_a), std::move(this->state_b)}};

  const TestEventX event{-1};
  const TestEventContainer event_container{event};

  EXPECT_CALL(*this->raw_ptr_a, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_b));

  state_machine.ProcessEvent(event_container);

  EXPECT_CALL(*this->raw_ptr_b, Process(event_container))
  .Times(1)
  .WillOnce(Return(TestStateId::state_a));

  EXPECT_CALL(*this->raw_ptr_a, Enter()).Times(1);
  EXPECT_CALL(*this->raw_ptr_b, Enter()).Times(0);
  state_machine.ProcessEvent(event_container);
}

}
