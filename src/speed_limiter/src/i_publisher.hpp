#pragma once

namespace analog::speed_limiter
{

template<typename T>
class IPublisher
{
public:
  virtual ~IPublisher() = default;
  virtual void Publish(const T data) = 0;
};

}
