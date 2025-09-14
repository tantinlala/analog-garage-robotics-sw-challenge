#pragma once

#include <gmock/gmock.h>
#include <variant>

#include "state_machine/i_state.hpp"

namespace analog::sm
{

enum class TestStateId
{
  state_a,
  state_b,
  num_state_ids
};

using TestEventX = int;
using TestEventY = double;

using TestEventContainer = std::variant<TestEventX, TestEventY>;

class MockState : public IState<TestStateId, TestEventContainer>
{
public:
  MOCK_METHOD(TestStateId, GetStateId, (), (override));
  MOCK_METHOD(void, Enter, (), (override));
  MOCK_METHOD(
    TestStateId, Process,
    (const TestEventContainer & event_container), (override));
};

}
