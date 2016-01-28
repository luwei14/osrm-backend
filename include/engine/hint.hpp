#ifndef ENGINE_HINT_HPP
#define ENGINE_HINT_HPP

#include "engine/object_encoder.hpp"
#include "engine/phantom_node.hpp"

namespace osrm
{
namespace engine
{

// Is returned as a temporary identifier for snapped coodinates
struct Hint
{
    FixedPointCoordinate input_coordinate;
    PhantomNode phantom;
    std::uint32_t data_checksum;

    template<typename DataFacadeT>
    bool IsValid(const FixedPointCoordinate new_input_coordinates, DataFacadeT *facade) const
    {
        auto is_same_input_coordinate = new_input_coordinates.lat == input_coordinate.lat && new_input_coordinates.lon == input_coordinate.lon;
        return is_same_input_coordinate && phantom.IsValid(facade->GetNumberOfNodes()) && facade->GetChecksum() == data_checksum;
    }

    std::string ToBase64() const
    {
        std::string encoded;
        ObjectEncoder::EncodeToBase64(*this, encoded);
        return encoded;
    }

    static Hint FromBase64(std::string base64Hint)
    {
        Hint decoded;
        ObjectEncoder::DecodeFromBase64(base64Hint, decoded);
        return decoded;
    }
};

}
}

#endif

