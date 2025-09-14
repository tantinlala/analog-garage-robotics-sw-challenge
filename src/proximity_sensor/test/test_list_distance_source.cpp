#include <gtest/gtest.h>

#include "list_distance_source.hpp"

using testing::Test;

namespace analog::proximity_sensor
{

TEST(ListDistanceSourceTest, When_GetDistanceCalled_Expect_CorrectValues)
{
  ListDistanceSource::Series series{100, 200, 300};
  ListDistanceSource distance_source{series};
  EXPECT_FLOAT_EQ(100, distance_source.GetDistance().value());
  EXPECT_FLOAT_EQ(200, distance_source.GetDistance().value());
  EXPECT_FLOAT_EQ(300, distance_source.GetDistance().value());
  EXPECT_EQ(std::nullopt, distance_source.GetDistance());
}

}
