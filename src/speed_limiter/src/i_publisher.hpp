#pragma once

namespace analog::speed_limiter
{

/**
  * @brief Interface for publishing data
  * @tparam T Type of data to publish
 */
template<typename T>
class IPublisher
{
public:
  virtual ~IPublisher() = default;

  /**
   * @brief Publish data
   * @param data Data to publish
   */
  virtual void Publish(const T data) = 0;
};

}
