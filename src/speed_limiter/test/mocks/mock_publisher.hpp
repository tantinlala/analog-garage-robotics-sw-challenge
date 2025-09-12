#pragma once

#include <gmock/gmock.h>
#include "i_publisher.hpp"

namespace analog::speed_limiter 
{

template <typename StateT>
class MockPublisher : public IPublisher<StateT>
{
    public:
        MOCK_METHOD(void, Publish, (StateT), (override));
};


}