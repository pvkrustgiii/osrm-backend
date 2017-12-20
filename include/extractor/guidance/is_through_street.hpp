#include "extractor/guidance/sliproad_handler.hpp"
#include "extractor/guidance/constants.hpp"
#include "util/assert.hpp"
#include "util/bearing.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/guidance/name_announcements.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include <boost/assert.hpp>

using osrm::extractor::guidance::getTurnDirection;
using osrm::util::angularDeviation;

namespace osrm
{
namespace extractor
{
namespace guidance
{

// template <typename restriction_type> auto asDuplicatedNode(const restriction_type &restriction)
// {
//     auto &way = restriction.AsWayRestriction();
//     // group restrictions by the via-way. On same via-ways group by from
//     return std::tie(way.in_restriction.via, way.out_restriction.via, way.in_restriction.from);
// }

bool isThroughStreet(const EdgeID from, const IntersectionView &intersection, const util::NodeBasedDynamicGraph &node_based_graph, const EdgeBasedNodeDataContainer &node_data_container, const util::NameTable &name_table, const SuffixTable &street_name_suffix_table)
{
    BOOST_ASSERT(from != SPECIAL_EDGEID);
    BOOST_ASSERT(!intersection.empty());

    const auto from_annotation_id = node_based_graph.GetEdgeData(from).annotation_data;
    const auto &edge_name_id = node_data_container.GetAnnotation(from_annotation_id).name_id;

    auto first = begin(intersection) + 1; // Skip UTurn road
    auto last = end(intersection);

    auto same_name = [&](const auto &road) {
        const auto annotation_id = node_based_graph.GetEdgeData(road.eid).annotation_data;
        const auto &road_name_id = node_data_container.GetAnnotation(annotation_id).name_id;

        return edge_name_id != EMPTY_NAMEID && //
               road_name_id != EMPTY_NAMEID && //
               !util::guidance::requiresNameAnnounced(edge_name_id,
                                                      road_name_id,
                                                      name_table,
                                                      street_name_suffix_table); //
    };

    return std::find_if(first, last, same_name) != last;
}


} // namespace guidance
} // namespace extractor
} // namespace osrm