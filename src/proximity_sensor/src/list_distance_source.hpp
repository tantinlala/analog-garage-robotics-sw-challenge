#pragma once

#include <vector>
#include <string>
#include <optional>

namespace analog::proximity_sensor
{

class ListDistanceSource 
{
    public:
        using Series = std::vector<double>;

        ListDistanceSource(Series& series);

        std::optional<float> GetDistance();

    private:
        Series series_;
        std::vector<double>::iterator iterator_;
};

}