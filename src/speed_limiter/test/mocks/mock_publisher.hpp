#pragma once

#include <gmock/gmock.h>
#include "speed_limiter/i_publisher.hpp"

namespace analog::speed_limiter 
{

template <typename StateT>
class MockPublisher : public IPublisher<StateT>
{
    public:
        MOCK_METHOD(void, Publish, (StateT), (override));
};


}