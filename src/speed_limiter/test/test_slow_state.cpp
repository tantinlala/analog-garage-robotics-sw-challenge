#include <gtest/gtest.h>
#include "speed_limiter/speed_limiter_states.hpp"
#include "mocks/mock_publisher.hpp"

using testing::Test;

namespace analog::speed_limiter 
{

class SlowStateTest: public Test 
{
    protected:
        static constexpr float kMinBoundary{1.0f};
        static constexpr float kMaxBoundary{2.0f};
        std::shared_ptr<MockPublisher<StateId>> publisher_ptr_{new MockPublisher<StateId>()};
        NotEstoppedState slow_state_{
            StateId::SLOW,
            NotEstoppedState::ProximityBoundary{StateId::STOP, kMinBoundary},
            NotEstoppedState::ProximityBoundary{StateId::FULL_SPEED, kMaxBoundary},
            publisher_ptr_
        };

        sm::State<StateId, Events>* state_ptr_{&slow_state_};
};

TEST_F(SlowStateTest, When_Enter_Expect_PublishSlow)
{
    EXPECT_CALL(*publisher_ptr_, Publish(StateId::SLOW)).Times(1);
    state_ptr_->Enter();
}

TEST_F(SlowStateTest, When_BetweenMinAndMax_Expect_StayInState)
{
    const float distance{(kMinBoundary + kMaxBoundary) / 2.0f};
    const StateId old_state_id{state_ptr_->GetStateId()};
    const auto new_state_id{state_ptr_->Process(ProximityData{distance})};
    EXPECT_EQ(new_state_id, old_state_id);
}

TEST_F(SlowStateTest, When_DistanceMoreThanMax_Expect_TransitionToFullSpeed)
{
    const auto new_state_id{state_ptr_->Process(ProximityData{kMaxBoundary + 0.1f})};
    EXPECT_EQ(new_state_id, StateId::FULL_SPEED);
}

TEST_F(SlowStateTest, When_DistanceLessThanMin_Expect_TransitionToStop)
{
    const auto new_state_id{state_ptr_->Process(ProximityData{kMinBoundary - 0.1f})};
    EXPECT_EQ(new_state_id, StateId::STOP);
}

}