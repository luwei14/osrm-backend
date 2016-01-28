#ifndef ROUTE_LEG_HPP
#define ROUTE_LEG_HPP

#include <boost/optional.hpp>

#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace guidance
{

struct RouteLeg
{
    double duration;
    double distance;
    std::string summary;
    boost::optional<std::vector<RouteStep>> steps;
};

}
}
}

#endif
