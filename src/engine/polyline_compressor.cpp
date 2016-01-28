#include "engine/polyline_compressor.hpp"

#include <boost/assert.hpp>
#include <cstddef>
#include <algorithm>

namespace osrm
{
namespace engine
{
namespace /*detail*/ // anonymous to keep TU local
{

std::string encode(int number_to_encode)
{
    std::string output;
    while (number_to_encode >= 0x20)
    {
        const int next_value = (0x20 | (number_to_encode & 0x1f)) + 63;
        output += static_cast<char>(next_value);
        number_to_encode >>= 5;
    }

    number_to_encode += 63;
    output += static_cast<char>(number_to_encode);
    return output;
}

std::string encode(std::vector<int> &numbers)
{
    std::string output;
    const auto end = numbers.size();
    for (std::size_t i = 0; i < end; ++i)
    {
        numbers[i] <<= 1;
        if (numbers[i] < 0)
        {
            numbers[i] = ~(numbers[i]);
        }
    }
    for (const int number : numbers)
    {
        output += encode(number);
    }
    return output;
}
} // anonymous ns


std::string encodePolyline(CoordVectorForwardIter begin, CoordVectorForwardIter end)
{
    if (std::distance(begin, end) == 0)
    {
        return {};
    }

    std::vector<int> delta_numbers;
    BOOST_ASSERT(locations.size() > 0);
    delta_numbers.reserve((std::distance(begin, end) - 1) * 2);
    util::FixedPointCoordinate previous_coordinate = {0, 0};
    std::for_each(begin, end, [&delta_numbers, &previous_coordinate](const FixedPointCoordinate loc)
    {
            const int lat_diff = loc.lat - previous_coordinate.lat;
            const int lon_diff = loc.lon - previous_coordinate.lon;
            delta_numbers.emplace_back(lat_diff);
            delta_numbers.emplace_back(lon_diff);
            previous_coordinate = loc;
    });
    return encode(delta_numbers);
}
std::vector<util::FixedPointCoordinate> decodePolyline(const std::string &geometry_string)
{
    std::vector<util::FixedPointCoordinate> new_coordinates;
    int index = 0, len = geometry_string.size();
    int lat = 0, lng = 0;

    while (index < len)
    {
        int b, shift = 0, result = 0;
        do
        {
            b = geometry_string.at(index++) - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int dlat = ((result & 1) != 0 ? ~(result >> 1) : (result >> 1));
        lat += dlat;

        shift = 0;
        result = 0;
        do
        {
            b = geometry_string.at(index++) - 63;
            result |= (b & 0x1f) << shift;
            shift += 5;
        } while (b >= 0x20);
        int dlng = ((result & 1) != 0 ? ~(result >> 1) : (result >> 1));
        lng += dlng;

        util::FixedPointCoordinate p;
        p.lat = COORDINATE_PRECISION * (((double)lat / 1E6));
        p.lon = COORDINATE_PRECISION * (((double)lng / 1E6));
        new_coordinates.push_back(p);
    }

    return new_coordinates;
}
}
}
