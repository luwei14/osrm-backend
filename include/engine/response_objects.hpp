#ifndef ENGINE_RESPONSE_OBJECTS_HPP_
#define ENGINE_RESPONSE_OBJECTS_HPP_

#include "engine/guidance/route_step.hpp"
#include "engine/guidance/step_maneuver.hpp"
#include "engine/guidance/route_leg.hpp"
#include "engine/guidance/route.hpp"
#include "engine/guidance/leg_geometry.hpp"
#include "engine/hint.hpp"
#include "engine/polyline_compressor.hpp"
#include "extractor/turn_instructions.hpp"
#include "extractor/travel_mode.hpp"
#include "util/coordinate.hpp"
#include "util/json_container.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>
#include <boost/optional.hpp>

#include <string>
#include <utility>
#include <algorithm>
#include <vector>

namespace osrm
{
namespace engine
{
namespace detail
{

std::string instructionToString(extractor::TurnInstruction instruction)
{
    std::string token;
    switch (instruction)
    {
    case extractor::TurnInstruction::Continue:
        token = "continue";
        break;
    case extractor::TurnInstruction::BearRight:
        token = "bear right";
        break;
    case extractor::TurnInstruction::TurnRight:
        token = "turn right";
        break;
    case extractor::TurnInstruction::SharpRight:
        token = "sharp right";
        break;
    case extractor::TurnInstruction::UTurn:
        token = "uturn";
        break;
    case extractor::TurnInstruction::SharpLeft:
        token = "sharp left";
        break;
    case extractor::TurnInstruction::TurnLeft:
        token = "turn left";
        break;
    case extractor::TurnInstruction::BearLeft:
        token = "bear left";
        break;
    case extractor::TurnInstruction::ReachedWaypointLocation:
        token = "waypoint";
        break;
    case extractor::TurnInstruction::EnterRoundAbout:
        token = "enter roundabout";
        break;
    case extractor::TurnInstruction::LeaveRoundAbout:
        token = "leave roundabout";
        break;
    case extractor::TurnInstruction::StayOnRoundAbout:
        token = "roundabout";
        break;
    case extractor::TurnInstruction::Depart:
        token = "depart";
        break;
    case extractor::TurnInstruction::Arrive:
        token = "arrive";
        break;
    default:
        BOOST_ASSERT("Unsupported instruction reached");
        break;
    }
    return token;
}

util::json::Array coordinateToLonLat(const FixedPointCoordinate &coordinate)
{
    util::json::Array array;
    array.values.push_back(coordinate.lon / COORDINATE_PRECISION);
    array.values.push_back(coordinate.lat / COORDINATE_PRECISION);
    return array;
}

util::json::Array coordinateToLatLon(const FixedPointCoordinate &coordinate)
{
    util::json::Array array;
    array.values.push_back(coordinate.lat / COORDINATE_PRECISION);
    array.values.push_back(coordinate.lon / COORDINATE_PRECISION);
    return array;
}

// FIXME this actually needs to be configurable from the profiles
std::string modeToString(const extractor::TravelMode mode) {
  std::string token;
  switch (mode)
  {
    case TRAVEL_MODE_DEFAULT:
      token = "default";
      break;
    case TRAVEL_MODE_INACCESSIBLE:
      token = "inaccessible";
      break;
    default:
      token = "other";
      break;
  }
  return token;
}

} // namespace detail

util::json::Object makeStepManeuver(const guidance::StepManeuver& maneuver)
{
    util::json::Object step_maneuver;
    step_maneuver.values["type"] = detail::instructionToString(maneuver.instruction);
    step_maneuver.values["location"] = detail::coordinateToLatLon(maneuver.location);
    step_maneuver.values["heading_before"] = maneuver.heading_before;
    step_maneuver.values["heading_after"] = maneuver.heading_after;
    return step_maneuver;
}

util::json::Object makeRouteStep(guidance::RouteStep &&step, boost::optional<util::json::Value> geometry)
{
    util::json::Object route_step;
    route_step.values["distance"] = step.distance;
    route_step.values["duration"] = step.duration;
    route_step.values["way_name"] = std::move(step.way_name);
    route_step.values["mode"] = detail::modeToString(step.mode);
    route_step.values["maneuver"] = makeStepManeuver(step.maneuver);
    if (geometry)
    {
        route_step.values["geometry"] = std::move(*geometry);
    }
    return route_step;
}

template<typename ForwardIter>
util::json::String makePolyline(ForwardIter begin, ForwardIter end)
{
    util::json::String polyline;
    polyline.value = encodePolyline(begin, end);
    return polyline;
}

template<typename ForwardIter>
util::json::Array makeCoordinateArray(ForwardIter begin, ForwardIter end)
{
    util::json::Array coordinates;
    std::transform(begin, end, std::back_inserter(coordinates.values), [](const util::FixedPointCoordinate loc) {
          return detail::coordinateToLatLon(loc);
        });
    return coordinates;
}

template<typename ForwardIter>
util::json::Object makeGeoJSONLineString(ForwardIter begin, ForwardIter end)
{
    util::json::Object geojson;
    geojson.values["type"] = "LineString";
    util::json::Array coordinates;
    std::transform(begin, end, std::back_inserter(coordinates.values), [](const util::FixedPointCoordinate loc) {
          return detail::coordinateToLonLat(loc);
        });
    geojson.values["coordinates"] = std::move(coordinates);
    return geojson;
}

util::json::Object makeRoute(const guidance::Route& route,
                             util::json::Array&& legs,
                             boost::optional<util::json::Value> geometry)
{
    util::json::Object json_route;
    json_route.values["distance"] = route.distance;
    json_route.values["duration"] = route.duration;
    json_route.values["legs"] = std::move(legs);
    if (geometry)
    {
        json_route.values["geometry"] = std::move(*geometry);
    }
    return json_route;
}

util::json::Object makeWaypoint(const FixedPointCoordinate location, std::string &&way_name, const Hint& hint)
{
    util::json::Object waypoint;
    waypoint.values["location"] = detail::coordinateToLatLon(location);
    waypoint.values["way_name"] = std::move(way_name);
    waypoint.values["hint"] = hint.ToBase64();
    return waypoint;
}

util::json::Object makeRouteLeg(guidance::RouteLeg &&leg,
                                boost::optional<util::json::Array> steps)
{
    util::json::Object route_leg;
    route_leg.values["distance"] = leg.distance;
    route_leg.values["duration"] = leg.duration;
    route_leg.values["summary"] = std::move(leg.summary);
    if (steps)
    {
        route_leg.values["steps"] = std::move(steps);
    }
    return route_leg;
}

util::json::Array makeRouteLegs(std::vector<guidance::RouteLeg>&& legs, const std::vector<guidance::LegGeometry> &leg_geometries)
{
    util::json::Array json_legs;
    for (auto&& leg : legs)
    {
        boost::optional<util::json::Array> json_steps;
        if (leg.steps)
        {
            json_steps = boost::make_optional(util::json::Array {});
            BOOST_ASSERT(json_steps);
            json_steps->values.resize(leg.steps->size());
            std::transform(leg.steps->begin(), leg.steps->end(), json_steps->values.begin(), [&leg_geometries](guidance::RouteStep&& step) {
                  // FIXME we only support polyline here
                  auto geometry = boost::make_optional<util::json::Value>(makePolyline(leg_geometries.begin() + step.geometry_begin, leg_geometries.begin() + step.geometry_end));
                  return makeRouteStep(std::move(step), std::move(geometry));
                });
        }
        json_legs.values.push_back(makeRouteLeg(std::move(leg), json_steps));
    }

    return json_legs;
}

} // namespace engine
} // namespace osrm

#endif // ENGINE_GUIDANCE_API_RESPONSE_GENERATOR_HPP_
