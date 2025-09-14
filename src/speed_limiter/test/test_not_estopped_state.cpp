#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>
#include <stdexcept>
#include "speed_limiter_states.hpp"
#include "mocks/mock_publisher.hpp"

using testing::TestWithParam;
using testing::Values;

namespace analog::speed_limiter
{

static const NotEstoppedState::ProximityBoundary kStopBoundary{StateId::STOP, 400.0};
static const NotEstoppedState::ProximityBoundary kSlowBoundary{StateId::SLOW, 800.0};
static const NotEstoppedState::ProximityBoundary kFullSpeedBoundary{StateId::FULL_SPEED,
  std::numeric_limits<float>::infinity()};

static const NotEstoppedState::Params kGoodParams{
  {kStopBoundary, kSlowBoundary, kFullSpeedBoundary},
  50.0,
};

struct TestParams
{
  // Given
  StateId initial_state;
  float distance_mm;

  // Expect
  StateId expected_state;
};

class NotEstoppedTest : public TestWithParam<TestParams>
{
protected:
  std::shared_ptr<MockPublisher<StateId>> publisher_ptr{std::make_shared<MockPublisher<StateId>>()};
  std::shared_ptr<NotEstoppedState::Params> params_ptr{std::make_shared<NotEstoppedState::Params>(
      kGoodParams)};
};

TEST_F(NotEstoppedTest, When_BoundariesOutOfOrder_ExpectThrow)
{
  params_ptr->boundaries.at(0).distance_mm = 800.0;
  params_ptr->boundaries.at(1).distance_mm = 400.0;
  EXPECT_THROW(NotEstoppedState(StateId::SLOW, params_ptr, publisher_ptr), std::runtime_error);
}

TEST_F(NotEstoppedTest, When_DuplicateStates_ExpectThrow)
{
  params_ptr->boundaries.at(0).state = StateId::SLOW;
  params_ptr->boundaries.at(1).state = StateId::SLOW;
  EXPECT_THROW(NotEstoppedState(StateId::SLOW, params_ptr, publisher_ptr), std::runtime_error);
}

TEST_F(NotEstoppedTest, When_EstoppedStateStatesInParams_ExpectThrow)
{
  params_ptr->boundaries.at(2).state = StateId::ESTOPPED;
  EXPECT_THROW(NotEstoppedState(StateId::SLOW, params_ptr, publisher_ptr), std::runtime_error);
}

TEST_F(NotEstoppedTest, When_NumStatesInParams_ExpectThrow)
{
  params_ptr->boundaries.at(2).state = StateId::ESTOPPED;
  EXPECT_THROW(NotEstoppedState(StateId::SLOW, params_ptr, publisher_ptr), std::runtime_error);
}

TEST_F(NotEstoppedTest, When_HysteresisNegative_ExpectThrow)
{
  params_ptr->hysteresis = -50.0;
  EXPECT_THROW(NotEstoppedState(StateId::SLOW, params_ptr, publisher_ptr), std::runtime_error);
}

TEST_F(NotEstoppedTest, When_Enter_Expect_PublishState)
{
  NotEstoppedState state{StateId::SLOW, params_ptr, publisher_ptr};
  sm::IState<StateId, Events> * state_ptr{&state};

  EXPECT_CALL(*publisher_ptr, Publish(StateId::SLOW)).Times(1);
  state_ptr->Enter();
}

TEST_F(NotEstoppedTest, When_EstopCleared_Expect_StayInState)
{
  NotEstoppedState state{StateId::SLOW, params_ptr, publisher_ptr};
  sm::IState<StateId, Events> * state_ptr{&state};

  const auto new_state_id{state_ptr->Process(EstopCleared{})};
  EXPECT_EQ(state_ptr->GetStateId(), new_state_id);
}

TEST_F(NotEstoppedTest, When_EstopSet_Expect_ReturnEstopped)
{
  NotEstoppedState state{StateId::SLOW, params_ptr, publisher_ptr};
  sm::IState<StateId, Events> * state_ptr{&state};

  const auto new_state_id{state_ptr->Process(EstopTriggered{})};
  EXPECT_EQ(StateId::ESTOPPED, new_state_id);
}

TEST_P(NotEstoppedTest, Given_Distance_Expect_CorrectState)
{
  auto [initial_state, distance, expected_state] = GetParam();
  NotEstoppedState state{initial_state, params_ptr, publisher_ptr};
  sm::IState<StateId, Events> * state_ptr{&state};

  const auto new_state_id{state_ptr->Process(ProximityData{distance})};
  EXPECT_EQ(expected_state, new_state_id);
}

INSTANTIATE_TEST_SUITE_P(
  Transition, NotEstoppedTest,
  Values(
    // Given in STOP State
    TestParams{StateId::STOP, -1.0, StateId::STOP},
    TestParams{StateId::STOP, 49.0, StateId::STOP},
    TestParams{StateId::STOP, 50.0, StateId::STOP},
    TestParams{StateId::STOP, 51.0, StateId::STOP},
    TestParams{StateId::STOP, 399.0, StateId::STOP},
    TestParams{StateId::STOP, 400.0, StateId::STOP},
    TestParams{StateId::STOP, 401.0, StateId::STOP},
    TestParams{StateId::STOP, 449.0, StateId::STOP},
    TestParams{StateId::STOP, 450.0, StateId::SLOW},
    TestParams{StateId::STOP, 451.0, StateId::SLOW},
    TestParams{StateId::STOP, 799.0, StateId::SLOW},
    TestParams{StateId::STOP, 800.0, StateId::SLOW},
    TestParams{StateId::STOP, 801.0, StateId::SLOW},
    TestParams{StateId::STOP, 849.0, StateId::SLOW},
    TestParams{StateId::STOP, 850.0, StateId::FULL_SPEED},
    TestParams{StateId::STOP, 851.0, StateId::FULL_SPEED},
    TestParams{StateId::STOP, 900.0, StateId::FULL_SPEED},

    // Given in SLOW State
    TestParams{StateId::SLOW, -1.0, StateId::STOP},
    TestParams{StateId::SLOW, 49.0, StateId::STOP},
    TestParams{StateId::SLOW, 50.0, StateId::STOP},
    TestParams{StateId::SLOW, 51.0, StateId::STOP},
    TestParams{StateId::SLOW, 399.0, StateId::STOP},
    TestParams{StateId::SLOW, 400.0, StateId::STOP},
    TestParams{StateId::SLOW, 401.0, StateId::SLOW},
    TestParams{StateId::SLOW, 449.0, StateId::SLOW},
    TestParams{StateId::SLOW, 450.0, StateId::SLOW},
    TestParams{StateId::SLOW, 451.0, StateId::SLOW},
    TestParams{StateId::SLOW, 799.0, StateId::SLOW},
    TestParams{StateId::SLOW, 800.0, StateId::SLOW},
    TestParams{StateId::SLOW, 801.0, StateId::SLOW},
    TestParams{StateId::SLOW, 849.0, StateId::SLOW},
    TestParams{StateId::SLOW, 850.0, StateId::FULL_SPEED},
    TestParams{StateId::SLOW, 851.0, StateId::FULL_SPEED},
    TestParams{StateId::SLOW, 900.0, StateId::FULL_SPEED},

    // Give in FULL_SPEED State
    TestParams{StateId::FULL_SPEED, -1.0, StateId::STOP},
    TestParams{StateId::FULL_SPEED, 49.0, StateId::STOP},
    TestParams{StateId::FULL_SPEED, 50.0, StateId::STOP},
    TestParams{StateId::FULL_SPEED, 51.0, StateId::STOP},
    TestParams{StateId::FULL_SPEED, 399.0, StateId::STOP},
    TestParams{StateId::FULL_SPEED, 400.0, StateId::STOP},
    TestParams{StateId::FULL_SPEED, 401.0, StateId::SLOW},
    TestParams{StateId::FULL_SPEED, 449.0, StateId::SLOW},
    TestParams{StateId::FULL_SPEED, 450.0, StateId::SLOW},
    TestParams{StateId::FULL_SPEED, 451.0, StateId::SLOW},
    TestParams{StateId::FULL_SPEED, 799.0, StateId::SLOW},
    TestParams{StateId::FULL_SPEED, 800.0, StateId::SLOW},
    TestParams{StateId::FULL_SPEED, 801.0, StateId::FULL_SPEED},
    TestParams{StateId::FULL_SPEED, 849.0, StateId::FULL_SPEED},
    TestParams{StateId::FULL_SPEED, 850.0, StateId::FULL_SPEED},
    TestParams{StateId::FULL_SPEED, 851.0, StateId::FULL_SPEED},
    TestParams{StateId::FULL_SPEED, 900.0, StateId::FULL_SPEED}
));

}
