#ifndef ENGINE_GUIDANCE_LEG_ASSEMBLER_HPP_
#define ENGINE_GUIDANCE_LEG_ASSEMBLER_HPP_

#include "engine/guidance/route_leg.hpp"
#include "engine/guidance/route_step.hpp"
#include "engine/guidance/leg_geometry.hpp"
#include "engine/internal_route_result.hpp"

#include <boost/range/irange.hpp>

#include <vector>

namespace osrm
{
namespace engine
{
namespace guidance
{
template <typename DataFacadeT> class LegAssembler
{
  public:
    LegAssembler(const DataFacadeT *facade) : facade(facade){};

    RouteLeg operator()(const std::vector<PathData> &route_data,
                        const PhantomNode &source_node,
                        const PhantomNode &target_node,
                        const bool source_traversed_in_reverse,
                        const bool target_traversed_in_reverse,
                        const LegGeometry &leg_geometry) const
    {
        const auto source_duration =
            (source_traversed_in_reverse ? source_node.GetReverseWeightPlusOffset() : source_node.GetForwardWeightPlusOffset()) / 10.;
        const auto target_duration =
            (target_traversed_in_reverse ? target_node.GetReverseWeightPlusOffset() : target_node.GetForwardWeightPlusOffset()) / 10.;

        auto distance = std::accumulate(leg_geometry.segment_distances.begin(),
                                        leg_geometry.segment_distances.end(), 0.);
        auto duration = std::accumulate(route_data.begin(), route_data.end(),
                                        [](const PathData &data)
                                        {
                                            return data.duration_until_turn;
                                        }) / 10.;
        //                 s
        //                 |
        // Given a route a---b---c  where there is a right turn at b.
        //                       |
        //                       d
        //                       |--t
        //                       e
        // (a, b, c) gets compressed to (a,c)
        // (c, d, e) gets compressed to (c,e)
        // The duration of the turn (a,c) -> (c,e) will be the duration of (a,c) (e.g. the duration of (a,b,c)).
        // The phantom node of s will contain:
        // `forward_weight`: duration of (a,s)
        // `forward_offset`: 0 (its the first segment)
        // The phantom node of t will contain:
        // `forward_weight`: duration of (d,t)
        // `forward_offset`: duration of (c, d)
        //
        // The PathData will contain entries of b, c and d. But only c will contain
        // a duration value since its the only point associated with a turn.
        // As such we want to slice of the duration for (a,s) and add the duration for
        // (c,d,t)
        duration = duration - source_duration + target_duration;
        auto summary = SummarizeRoute(route_data);

        return RouteLeg{duration, distance, summary};
    }

  private:
    struct NamedSegment
    {
        double duration;
        std::uint32_t position;
        std::uint32_t name_id;
    };

    std::string SummarizeRoute(std::vector<PathData> &route_data) const
    {
        // merges segments with same name id
        const auto collapse_segments = [](std::vector<NamedSegment> &segments)
        {
            auto out = segments.begin();
            auto end = segments.end();
            for (auto in = segments.begin(); in != end; ++in)
            {
                if (in->name_id == out->name_id)
                {
                    out->duration += in->duration;
                }
                else
                {
                    ++out;
                    BOOST_ASSERT(out != end);
                    *out = *in;
                }
            }
            return out;
        };

        std::vector<NamedSegment> segments(route_data.size());
        std::uint32_t index = 0;
        std::transform(route_data.begin(), route_data.end(), segments.begin(),
                       [&index](const PathData &point)
                       {
                           return NamedSegment{point.duration_until_turn, index++, point.name_id};
                       });
        // this makes sure that the segment with the lowest position comes first
        std::sort(segments.begin(), segments.end(),
                  [](const NamedSegment &lhs, const NamedSegment &rhs)
                  {
                      return lhs.name_id < rhs.name_id ||
                             (lhs.name_id == rhs.name_id && lhs.position < rhs.position);
                  });
        auto new_end = collapse_segments(segments);
        segments.resize(new_end - segments.begin());
        // sort descending
        std::sort(segments.begin(), segments.end(),
                  [](const NamedSegment &lhs, const NamedSegment &rhs)
                  {
                      return lhs.duration > rhs.duration;
                  });

        // make sure the segments are sorted by position
        const constexpr std::size_t MAX_USED_SEGMENTS = 2;
        segments.resize(std::min(segments.size(), MAX_USED_SEGMENTS));
        std::sort(segments.begin(), segments.end(),
                  [](const NamedSegment &lhs, const NamedSegment &rhs)
                  {
                      return lhs.position < rhs.position;
                  });

        std::string summary;
        auto end_segment = segments.end();
        for (auto iter = segments.begin(); iter != end_segment; ++iter)
        {
            summary += facade->get_name_for_id(iter->name_id);
            if (std::next(iter) != end_segment)
            {
                summary += ", ";
            }
        }
        return summary;
    }

    DataFacadeT *facade;
};

} // namespace guidance
} // namespace engine
} // namespace osrm

#endif // ENGINE_GUIDANCE_SEGMENT_LIST_HPP_
