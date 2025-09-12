#include <gtest/gtest.h>
#include "speed_limiter_states.hpp"
#include "mocks/mock_publisher.hpp"

using testing::Test;

namespace analog::speed_limiter 
{

class StopStateTest: public Test 
{
    protected:
        static constexpr float kMaxBoundary{1.0f};
        std::shared_ptr<MockPublisher<StateId>> publisher_ptr_{new MockPublisher<StateId>()};
        NotEstoppedState full_speed_state_{
            StateId::STOP, 
            std::nullopt,
            NotEstoppedState::ProximityBoundary{StateId::SLOW, kMaxBoundary},
            publisher_ptr_
        };
        sm::IState<StateId, Events>* state_ptr_{&full_speed_state_};
};

TEST_F(StopStateTest, When_Enter_Expect_PublishStop)
{
    EXPECT_CALL(*publisher_ptr_, Publish(StateId::STOP)).Times(1);
    state_ptr_->Enter();
}

TEST_F(StopStateTest, When_EstopCleared_Expect_StayInState)
{
    const StateId old_state_id{state_ptr_->GetStateId()};
    const auto new_state_id{state_ptr_->Process(EstopCleared{})};
    EXPECT_EQ(new_state_id, old_state_id);
}

TEST_F(StopStateTest, When_EstopSet_Expect_ReturnEstopped)
{
    const auto new_state_id{state_ptr_->Process(EstopSet{})};
    EXPECT_EQ(new_state_id, StateId::ESTOPPED);
}

TEST_F(StopStateTest, When_DistanceLessThanMax_Expect_StayInState)
{
    const StateId old_state_id{state_ptr_->GetStateId()};
    const auto new_state_id{state_ptr_->Process(ProximityData{kMaxBoundary - 0.1f})};
    EXPECT_EQ(new_state_id, old_state_id);
}

TEST_F(StopStateTest, When_DistanceMoreThanMax_Expect_TransitionToSlow)
{
    const auto new_state_id{state_ptr_->Process(ProximityData{kMaxBoundary + 0.1f})};
    EXPECT_EQ(new_state_id, StateId::SLOW);
}

}