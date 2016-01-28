#ifndef ENGINE_GUIDANCE_ROUTE_ASSEMBLER_HPP
#define ENGINE_GUIDANCE_ROUTE_ASSEMBLER_HPP

#include "engine/guidance/leg_geometry.hpp"
#include "engine/douglas_peucker.hpp"

#include <boost/range/irange.hpp>

#include <vector>

namespace osrm
{
namespace engine
{
namespace guidance
{
template <typename DataFacadeT> class OverviewAssembler
{
  public:
    OverviewAssembler(const DataFacadeT *facade) : facade(facade){};

    std::vector<util::FixedPointCoordinate>
    operator()(const std::vector<LegGeometry> &leg_geometries, const unsigned zoom_level) const
    {
        std::vector<util::FixedPointCoordinate> overview_geometry;
        auto leg_index = 0;
        for (const auto geometry : leg_geometries)
        {
            auto simplified_geometry =
                douglasPeucker(leg_geometry.begin(), leg_geometry.end(), zoom_level);
            // not the last leg
            if (leg_index < leg_geometries.size() - 1)
            {
                simplified_geometry.pop_back();
            }
            overview_geometry.insert(overview_geometry.end(), simplified_geometry.begin(),
                                     simplified_geometry.end());
        }
        return overview_geometry;
    }

  private:
    DataFacadeT *facade;
};

} // namespace guidance
} // namespace engine
} // namespace osrm

#endif
