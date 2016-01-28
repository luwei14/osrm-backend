#ifndef POLYLINECOMPRESSOR_H_
#define POLYLINECOMPRESSOR_H_

#include "osrm/coordinate.hpp"

#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
using CoordVectorForwardIter = std::vector<FixedPointCoordinate>::const_iterator;
// Encodes geometry into polyline format.
// See: https://developers.google.com/maps/documentation/utilities/polylinealgorithm
std::string encodePolyline(CoordVectorForwardIter begin, CoordVectorForwardIter end);

// Decodes geometry from polyline format
// See: https://developers.google.com/maps/documentation/utilities/polylinealgorithm
std::vector<util::FixedPointCoordinate> decodePolyline(const std::string &polyline);
}
}

#endif /* POLYLINECOMPRESSOR_H_ */
