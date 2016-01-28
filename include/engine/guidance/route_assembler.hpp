#ifndef ENGINE_GUIDANCE_ROUTE_ASSEMBLER_HPP
#define ENGINE_GUIDANCE_ROUTE_ASSEMBLER_HPP

#include "engine/guidance/route_leg.hpp"
#include "engine/guidance/route.hpp"

#include <boost/range/irange.hpp>

#include <vector>

namespace osrm
{
namespace engine
{
namespace guidance
{
template <typename DataFacadeT> class RouteAssembler
{
  public:
    RouteAssembler(const DataFacadeT *facade) : facade(facade) {};

    Route operator()(const std::vector<RouteLeg> &route_legs) const
    {
        auto distance = std::accumulate(route_legs.begin(),
                                        route_legs.end(), 0.,
                                        [](const RouteLeg &leg)
                                        {
                                            return leg.distance;
                                        });
        auto duration = std::accumulate(route_legs.begin(), route_legs.end(),
                                        [](const RouteLeg &leg)
                                        {
                                            return leg.duration;
                                        });

        return Route{duration, distance};
    }

  private:
    DataFacadeT *facade;
};

} // namespace guidance
} // namespace engine
} // namespace osrm

#endif
