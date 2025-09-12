#include <gtest/gtest.h>
#include "speed_limiter_states.hpp"
#include "mocks/mock_publisher.hpp"

using testing::Test;

namespace analog::speed_limiter 
{

class EstoppedStateTest: public Test 
{
    protected:
        std::shared_ptr<MockPublisher<StateId>> publisher_ptr_{new MockPublisher<StateId>()};
        EstoppedState estopped_state_{publisher_ptr_};
        sm::IState<StateId, Events>* state_ptr_{&estopped_state_};
};

TEST_F(EstoppedStateTest, When_Entered_Expect_PublishEstopped)
{
    EXPECT_CALL(*publisher_ptr_, Publish(StateId::ESTOPPED)).Times(1);
    state_ptr_->Enter();
}

TEST_F(EstoppedStateTest, When_EstopCleared_Expect_TransitionToStop)
{
    const auto new_state_id{state_ptr_->Process(EstopCleared{})};
    EXPECT_EQ(new_state_id, StateId::STOP);
}

TEST_F(EstoppedStateTest, When_EstopSet_Expect_StayEstopped)
{
    const auto new_state_id{state_ptr_->Process(EstopSet{})};
    EXPECT_EQ(new_state_id, StateId::ESTOPPED);
}

TEST_F(EstoppedStateTest, When_ProximityData_Expect_StayEstopped)
{
    const auto new_state_id{state_ptr_->Process(ProximityData{0.5f})};
    EXPECT_EQ(new_state_id, StateId::ESTOPPED);
}

}