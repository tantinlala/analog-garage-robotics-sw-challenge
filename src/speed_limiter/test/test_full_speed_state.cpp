#include <gtest/gtest.h>
#include "speed_limiter_states.hpp"
#include "mocks/mock_publisher.hpp"

using testing::Test;

namespace analog::speed_limiter 
{

class FullSpeedStateTest: public Test 
{
    protected:
        static constexpr float kMinBoundary{2.0f};
        std::shared_ptr<MockPublisher<StateId>> publisher_ptr_{new MockPublisher<StateId>()};
        NotEstoppedState full_speed_state_{
            StateId::FULL_SPEED, 
            NotEstoppedState::ProximityBoundary{StateId::SLOW, kMinBoundary},
            std::nullopt,
            publisher_ptr_
        };
        sm::IState<StateId, Events>* state_ptr_{&full_speed_state_};
};

TEST_F(FullSpeedStateTest, When_Enter_Expect_PublishFullSpeed)
{
    EXPECT_CALL(*publisher_ptr_, Publish(StateId::FULL_SPEED)).Times(1);
    state_ptr_->Enter();
}

TEST_F(FullSpeedStateTest, When_DistanceMoreThanMin_Expect_StayInState)
{
    const StateId old_state_id{state_ptr_->GetStateId()};
    const auto new_state_id{state_ptr_->Process(ProximityData{kMinBoundary + 0.1f})};
    EXPECT_EQ(new_state_id, old_state_id);
}

TEST_F(FullSpeedStateTest, When_DistanceLessThanMin_Expect_TransitionToSlow)
{
    const auto new_state_id{state_ptr_->Process(ProximityData{kMinBoundary - 0.1f})};
    EXPECT_EQ(new_state_id, StateId::SLOW);
}

}