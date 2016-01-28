#ifndef VIA_ROUTE_HPP
#define VIA_ROUTE_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/response_objects.hpp"
#include "engine/guidance/step_assembler.hpp"
#include "engine/guidance/geometry_assembler.hpp"
#include "engine/guidance/leg_assembler.hpp"
#include "engine/guidance/route_assembler.hpp"

#include "engine/object_encoder.hpp"
#include "engine/search_engine.hpp"
#include "util/for_each_pair.hpp"
#include "util/integer_range.hpp"
#include "util/json_renderer.hpp"
#include "util/make_unique.hpp"
#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

template <class DataFacadeT> class ViaRoutePlugin final : public BasePlugin
{
  private:
    std::string descriptor_string;
    std::unique_ptr<SearchEngine<DataFacadeT>> search_engine_ptr;
    DataFacadeT *facade;
    int max_locations_viaroute;
    guidance::StepAssembler<DataFacadeT> step_assembler;
    guidance::LegAssembler<DataFacadeT> leg_asssembler;
    guidance::GeometryAssembler<DataFacadeT> geometry_assembler;
    guidance::RouteAssembler<DataFacadeT> route_assembler;
    guidance::OverviewAssembler<DataFacadeT> overview_assembler;

    util::json::Array MakeWaypoints(const RouteParameters &parameters,
                                    const InternalRouteResult &raw_route) const
    {
        util::json::Array waypoints;
        auto coordinate_iter = parameters.coordinates.begin();

        const auto &first_phantoms = raw_route.segment_end_coordinates.front();
        waypoints.values.push_back(makeWaypoint(
            first_phantoms.source_phantom.location,
            facade->get_name_for_id(first_phantoms.source_phantom.name_id),
            Hint{*coordinate_iter++, first_phantoms.source_phantom, facade->GetChecksum()}));
        for (auto phantoms : raw_route.segment_end_coordinates)
        {
            waypoints.values.push_back(makeWaypoint(
                phantoms.target_phantom.location,
                facade->get_name_for_id(phantoms.target_phantom.name_id),
                Hint{*coordinate_iter++, phantoms.target_phantom, facade->GetChecksum()}));
        }

        return waypoints;
    }

    util::json::Array MakeRoutes(const RouteParameters &parameters,
                                 const InternalRouteResult &raw_route) const
    {
        util::json::Array json_routes;
        // currently the raw_route has a weird format that boundles routes
        auto number_of_routes = raw_route.has_alterntaive() ? 2 : 1;
        json_routes.reserve(number_of_routes);

        std::vector<RouteLeg> legs;
        std::vector<LegGeometry> leg_geometries;
        legs.reserve(raw_route.segment_end_coordinates.size());
        leg_geometries.reserve(raw_route.segment_end_coordinates.size());
        for (auto idx : boost::irange(0UL, raw_route.segment_end_coordinates.size()))
        {
            leg_geometries.push_back(
                geometry_assembler(leg, phantoms.source_phantom, phantoms.target_phantom));
            legs.push_back(leg_asssembler(
                raw_route.unpacked_path_segments[idx], raw_route.segment_end_coordinates[idx],
                raw_route.source_traversed_in_reverse[idx],
                raw_route.alt_target_traversed_in_reverse[idx], leg_geometries.back()));

            if (parameters.print_instructions)
            {
                legs.back().steps = boost::make_option(
                    step_assembler(legs.back(), phantoms.source_phantom, phantoms.target_phantom,
                                   source_traversed_in_reverse, target_traversed_in_reverse,
                                   leg_geometries.back()));
            }
        }
        auto route = route_assembler(legs);
        boost::optional<util::json::Value> json_overview;
        if(parameters.geometry)
        {
            auto overview = overview_assembler(leg_geometries);
            if (parameters.compression)
            {
                json_overview = makePolyline(overview.begin(), overview.end());
            }
            else
            {
                json_overview = makeCoordinateArray(overview.begin(), overview.end());
            }
        }
        auto shortest_route = makeRoute(route, makeRouteLegs(std::move(legs), leg_geometries), std::move(json_overview));
        json_routes.values.push_back(std::move(shortest_route));
        if (raw_route.has_alternative())
        {
            BOOST_ASSERT(raw_route.segment_end_coodinates.size() == 1);
            std::vector<RouteLeg> alternative_legs(1);
            std::vector<LegGeometry> alternative_geometries(1);

            const auto& phantoms = raw_route.segment_end_coordinates.front();
            const auto& path_data = raw_route.unpacked_alternative;

            alternative_geometries.front() = geometry_assembler(path_data, phantoms.source_phantom, phantoms.target_phantom);
            alternative_legs.front() = leg_asssembler(
                path_data, phantoms,
                raw_route.source_traversed_in_reverse.front(),
                raw_route.alt_target_traversed_in_reverse.front(), alternative_geometries.front());
            auto alternative_route = route_assembler(alternative_legs);
            boost::optional<util::json::Value> json_alternative_overview;
            if(parameters.geometry)
            {
                auto overview = overview_assembler(alternative_geometries);
                if (parameters.compression)
                {
                    json_overview = makePolyline(overview.begin(), overview.end());
                }
                else
                {
                    json_overview = makeCoordinateArray(overview.begin(), overview.end());
                }
            }
            auto alternate_route = makeRoute(route, makeRouteLegs(std::move(alternative_legs), alternative_geometries), std::move(json_overview));
            json_routes.values.push_back(std::move(alternative_route));
        }

        return json_routes;
    }

    void MakeResponse(const RouteParameters &parameters,
                      const InternalRouteResult &raw_route,
                      util::json::Object &response) const
    {
        response.values["waypoints"] = MakeWaypoints(parameters, raw_route);
        response.values["routes"] = MakeRoutes(parameters, raw_route);
    }

  public:
    explicit ViaRoutePlugin(DataFacadeT *facade, int max_locations_viaroute)
        : descriptor_string("viaroute"), facade(facade), step_assembler(facade),
          leg_asssembler(facade), geometry_assembler(facade), route_assembler(facade),
          max_locations_viaroute(max_locations_viaroute)
    {
        search_engine_ptr = util::make_unique<SearchEngine<DataFacadeT>>(facade);
    }

    virtual ~ViaRoutePlugin() {}

    const std::string GetDescriptor() const override final { return descriptor_string; }

    Status HandleRequest(const RouteParameters &route_parameters,
                         util::json::Object &json_result) override final
    {
        if (max_locations_viaroute > 0 &&
            (static_cast<int>(route_parameters.coordinates.size()) > max_locations_viaroute))
        {
            return Error("too-big", "Number of entries " +
                                        std::to_string(route_parameters.coordinates.size()) +
                                        " is higher than current maximum (" +
                                        std::to_string(max_locations_viaroute) + ")",
                         json_result);
        }

        if (!check_all_coordinates(route_parameters.coordinates))
        {
            return Error("invalid-value", "Invalid coordinate value.", json_result);
        }

        const auto &input_bearings = route_parameters.bearings;
        if (input_bearings.size() > 0 &&
            route_parameters.coordinates.size() != input_bearings.size())
        {
            return Error("invalid-parameter",
                         "Number of bearings does not match number of coordinate", json_result);
        }

        std::vector<PhantomNodePair> phantom_node_pair_list(route_parameters.coordinates.size());
        const bool checksum_OK = (route_parameters.check_sum == facade->GetCheckSum());

        for (const auto i : util::irange<std::size_t>(0, route_parameters.coordinates.size()))
        {
            if (checksum_OK && i < route_parameters.hints.size() &&
                !route_parameters.hints[i].empty())
            {
                ObjectEncoder::DecodeFromBase64(route_parameters.hints[i],
                                                phantom_node_pair_list[i].first);
                if (phantom_node_pair_list[i].first.IsValid(facade->GetNumberOfNodes()))
                {
                    continue;
                }
            }
            const int bearing = input_bearings.size() > 0 ? input_bearings[i].first : 0;
            const int range = input_bearings.size() > 0
                                  ? (input_bearings[i].second ? *input_bearings[i].second : 10)
                                  : 180;
            phantom_node_pair_list[i] = facade->NearestPhantomNodeWithAlternativeFromBigComponent(
                route_parameters.coordinates[i], bearing, range);
            // we didn't found a fitting node, return error
            if (!phantom_node_pair_list[i].first.IsValid(facade->GetNumberOfNodes()))
            {
                return Error("no-segment",
                             std::string("Could not find a matching segment for coordinate ") +
                                 std::to_string(i),
                             json_result);
            }
            BOOST_ASSERT(phantom_node_pair_list[i].first.IsValid(facade->GetNumberOfNodes()));
            BOOST_ASSERT(phantom_node_pair_list[i].second.IsValid(facade->GetNumberOfNodes()));
        }

        auto snapped_phantoms = snapPhantomNodes(phantom_node_pair_list);

        InternalRouteResult raw_route;
        auto build_phantom_pairs = [&raw_route](const PhantomNode &first_node,
                                                const PhantomNode &second_node)
        {
            raw_route.segment_end_coordinates.push_back(PhantomNodes{first_node, second_node});
        };
        util::for_each_pair(snapped_phantoms, build_phantom_pairs);

        if (1 == raw_route.segment_end_coordinates.size())
        {
            if (route_parameters.alternate_route)
            {
                search_engine_ptr->alternative_path(raw_route.segment_end_coordinates.front(),
                                                    raw_route);
            }
            else
            {
                search_engine_ptr->direct_shortest_path(raw_route.segment_end_coordinates,
                                                        route_parameters.uturns, raw_route);
            }
        }
        else
        {
            search_engine_ptr->shortest_path(raw_route.segment_end_coordinates,
                                             route_parameters.uturns, raw_route);
        }

        // we can only know this after the fact, different SCC ids still
        // allow for connection in one direction.
        if (raw_route.is_valid())
        {
            auto generator = MakeApiResponseGenerator(facade);
            generator.DescribeRoute(route_parameters, raw_route, json_result);
            json_result.values["code"] = "ok";
        }
        else
        {
            auto first_component_id = snapped_phantoms.front().component.id;
            auto not_in_same_component =
                std::any_of(snapped_phantoms.begin(), snapped_phantoms.end(),
                            [first_component_id](const PhantomNode &node)
                            {
                                return node.component.id != first_component_id;
                            });

            if (not_in_same_component)
            {
                return Error("no-route", "Impossible route between points", json_result);
            }
            else
            {
                return Error("no-route", "No route found between points", json_result);
            }
        }

        return Status::Ok;
    }
};
}
}
}

#endif // VIA_ROUTE_HPP
