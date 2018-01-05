#include "guidance/guidance_runner.hpp"

// TODO: to guidance
#include "extractor/scripting_environment_lua.hpp"
#include "extractor/suffix_table.hpp"
#include "guidance/turn_data_container.hpp"

// #include "customizer/cell_customizer.hpp"
// #include "customizer/customizer.hpp"
// #include "customizer/edge_based_graph.hpp"
// #include "customizer/files.hpp"

// #include "partition/cell_storage.hpp"
// #include "partition/edge_based_graph_reader.hpp"
// #include "partition/files.hpp"
// #include "partition/multi_level_partition.hpp"

// #include "storage/shared_memory_ownership.hpp"

// #include "updater/updater.hpp"

// #include "util/exclude_flag.hpp"
// #include "util/log.hpp"
// #include "util/timing_util.hpp"

namespace osrm
{
namespace guidance
{

int GuidanceRunner::Run(const GuidanceConfig &config)
{

    extractor::Sol2ScriptingEnvironment scripting_environment(config.profile_path.string(), {});
    extractor::SuffixTable street_name_suffix_table(scripting_environment);

    TurnDataExternalContainer turn_data_container;

    /*
    const auto &turn_lanes_data = transformTurnLaneMapIntoArrays(lane_description_map);
    guidance::MergableRoadDetector mergable_road_detector(m_node_based_graph,
                                                          m_edge_based_node_container,
                                                          m_coordinates,
                                                          m_compressed_edge_container,
                                                          node_restriction_map,
                                                          m_barrier_nodes,
                                                          turn_lanes_data,
                                                          name_table,
                                                          street_name_suffix_table);

    // Loop over all turns and generate new set of edges.
    // Three nested loop look super-linear, but we are dealing with a (kind of)
    // linear number of turns only.
    guidance::TurnAnalysis turn_analysis(m_node_based_graph,
                                         m_edge_based_node_container,
                                         m_coordinates,
                                         m_compressed_edge_container,
                                         node_restriction_map,
                                         m_barrier_nodes,
                                         turn_lanes_data,
                                         name_table,
                                         street_name_suffix_table);

    util::guidance::LaneDataIdMap lane_data_map;
    guidance::lanes::TurnLaneHandler turn_lane_handler(m_node_based_graph,
                                                       m_edge_based_node_container,
                                                       m_coordinates,
                                                       m_compressed_edge_container,
                                                       node_restriction_map,
                                                       m_barrier_nodes,
                                                       turn_lanes_data,
                                                       lane_description_map,
                                                       turn_analysis,
                                                       lane_data_map);

    bearing_class_by_node_based_node.resize(m_node_based_graph.GetNumberOfNodes(),
                                            std::numeric_limits<std::uint32_t>::max());

    // The following block generates the edge-based-edges using a parallel processing
    // pipeline.  Sets of intersection IDs are batched in groups of GRAINSIZE (100)
    // `generator_stage`,
    // then those groups are processed in parallel `processor_stage`.  Finally, results are
    // appended to the various buffer vectors by the `output_stage` in the same order
    // that the `generator_stage` created them in (tbb::filter::serial_in_order creates this
    // guarantee).  The order needs to be maintained because we depend on it later in the
    // processing pipeline.
    {
        util::UnbufferedLog log;

        const NodeID node_count = m_node_based_graph.GetNumberOfNodes();

        // Because we write TurnIndexBlock data as we go, we'll
        // buffer them into groups of 1000 to reduce the syscall
        // count by 1000x.  This doesn't need much memory, but
        // greatly reduces the syscall overhead of writing lots
        // of small objects
        std::vector<lookup::TurnIndexBlock> turn_indexes_write_buffer;
        turn_indexes_write_buffer.reserve(TURN_INDEX_WRITE_BUFFER_SIZE);

        struct TurnsPipelineBuffer
        {
            std::size_t nodes_processed = 0;

            std::vector<TurnData> continuous_turn_data; // populate answers from guidance
            std::vector<TurnData> delayed_turn_data;    // populate answers from guidance
        };
        using TurnsPipelineBufferPtr = std::shared_ptr<TurnsPipelineBuffer>;

        // going over all nodes (which form the center of an intersection), we compute all
        // possible turns along these intersections.

        NodeID current_node = 0;

        // Handle intersections in sets of 100.  The pipeline below has a serial bottleneck
        // during the writing phase, so we want to make the parallel workers do more work
        // to give the serial final stage time to complete its tasks.
        const constexpr unsigned GRAINSIZE = 100;

        // First part of the pipeline generates iterator ranges of IDs in sets of GRAINSIZE
        tbb::filter_t<void, tbb::blocked_range<NodeID>> generator_stage(
            tbb::filter::serial_in_order, [&](tbb::flow_control &fc) {
                if (current_node < node_count)
                {
                    auto next_node = std::min(current_node + GRAINSIZE, node_count);
                    auto result = tbb::blocked_range<NodeID>(current_node, next_node);
                    current_node = next_node;
                    return result;
                }
                else
                {
                    fc.stop();
                    return tbb::blocked_range<NodeID>(node_count, node_count);
                }
            });

        //
        // Guidance stage
        //
        tbb::filter_t<tbb::blocked_range<NodeID>, TurnsPipelineBufferPtr> guidance_stage(
            tbb::filter::parallel, [&](const tbb::blocked_range<NodeID> &intersection_node_range) {

                auto buffer = std::make_shared<TurnsPipelineBuffer>();
                buffer->nodes_processed = intersection_node_range.size();

                for (auto intersection_node = intersection_node_range.begin(),
                          end = intersection_node_range.end();
                     intersection_node < end;
                     ++intersection_node)
                {
                    // We capture the thread-local work in these objects, then flush
                    // them in a controlled manner at the end of the parallel range
                    const auto &incoming_edges =
                        intersection::getIncomingEdges(m_node_based_graph, intersection_node);
                    const auto &outgoing_edges =
                        intersection::getOutgoingEdges(m_node_based_graph, intersection_node);
                    const auto &edge_geometries_and_merged_edges =
                        intersection::getIntersectionGeometries(m_node_based_graph,
                                                                m_compressed_edge_container,
                                                                m_coordinates,
                                                                mergable_road_detector,
                                                                intersection_node);
                    const auto &edge_geometries = edge_geometries_and_merged_edges.first;
                    const auto &merged_edge_ids = edge_geometries_and_merged_edges.second;

                    // all nodes in the graph are connected in both directions. We check all
                    // outgoing nodes to find the incoming edge. This is a larger search overhead,
                    // but the cost we need to pay to generate edges here is worth the additional
                    // search overhead.
                    //
                    // a -> b <-> c
                    //      |
                    //      v
                    //      d
                    //
                    // will have:
                    // a: b,rev=0
                    // b: a,rev=1 c,rev=0 d,rev=0
                    // c: b,rev=0
                    //
                    // From the flags alone, we cannot determine which nodes are connected to
                    // `b` by an outgoing edge. Therefore, we have to search all connected edges for
                    // edges entering `b`

                    for (const auto &incoming_edge : incoming_edges)
                    {
                        const auto intersection_view =
                            convertToIntersectionView(m_node_based_graph,
                                                      m_edge_based_node_container,
                                                      node_restriction_map,
                                                      m_barrier_nodes,
                                                      edge_geometries,
                                                      turn_lanes_data,
                                                      incoming_edge,
                                                      outgoing_edges,
                                                      merged_edge_ids);

                        auto intersection = turn_analysis.AssignTurnTypes(
                            incoming_edge.node, incoming_edge.edge, intersection_view);

                        OSRM_ASSERT(intersection.valid(), m_coordinates[intersection_node]);
                        intersection = turn_lane_handler.assignTurnLanes(
                            incoming_edge.node, incoming_edge.edge, std::move(intersection));

                        // the entry class depends on the turn, so we have to classify the
                        // interesction for every edge
                        const auto turn_classification =
                            classifyIntersection(intersection, m_coordinates[intersection_node]);

                        const auto entry_class_id =
                            entry_class_hash.ConcurrentFindOrAdd(turn_classification.first);

                        const auto bearing_class_id =
                            bearing_class_hash.ConcurrentFindOrAdd(turn_classification.second);

                        // Note - this is strictly speaking not thread safe, but we know we
                        // should never be touching the same element twice, so we should
                        // be fine.
                        bearing_class_by_node_based_node[intersection_node] = bearing_class_id;

                        // check if we are turning off a via way
                        const auto turning_off_via_way =
                            way_restriction_map.IsViaWay(incoming_edge.node, intersection_node);

                        for (const auto &outgoing_edge : outgoing_edges)
                        {
                            if (!intersection::isTurnAllowed(m_node_based_graph,
                                                             m_edge_based_node_container,
                                                             node_restriction_map,
                                                             m_barrier_nodes,
                                                             edge_geometries,
                                                             turn_lanes_data,
                                                             incoming_edge,
                                                             outgoing_edge))
                                continue;

                            const auto turn =
                                std::find_if(intersection.begin(),
                                             intersection.end(),
                                             [edge = outgoing_edge.edge](const auto &road) {
                                                 return road.eid == edge;
                                             });

                            OSRM_ASSERT(turn != intersection.end(),
                                        m_coordinates[intersection_node]);

                            buffer->continuous_turn_data.push_back(
                                TurnData{turn->instruction,
                                         turn->lane_data_id,
                                         entry_class_id,
                                         util::guidance::TurnBearing(intersection[0].bearing),
                                         util::guidance::TurnBearing(turn->bearing)});

                            // when turning off a a via-way turn restriction, we need to not only
                            // handle the normal edges for the way, but also add turns for every
                            // duplicated node. This process is integrated here to avoid doing the
                            // turn analysis multiple times.
                            if (turning_off_via_way)
                            {
                                const auto duplicated_nodes = way_restriction_map.DuplicatedNodeIDs(
                                    incoming_edge.node, intersection_node);

                                // next to the normal restrictions tracked in `entry_allowed`, via
                                // ways might introduce additional restrictions. These are handled
                                // here when turning off a via-way
                                for (auto duplicated_node_id : duplicated_nodes)
                                {
                                    auto const node_at_end_of_turn =
                                        m_node_based_graph.GetTarget(outgoing_edge.edge);

                                    const auto is_way_restricted = way_restriction_map.IsRestricted(
                                        duplicated_node_id, node_at_end_of_turn);

                                    if (is_way_restricted)
                                    {
                                        auto const restriction = way_restriction_map.GetRestriction(
                                            duplicated_node_id, node_at_end_of_turn);

                                        if (restriction.condition.empty())
                                            continue;

                                        buffer->delayed_turn_data.push_back(TurnData{
                                            turn->instruction,
                                            turn->lane_data_id,
                                            entry_class_id,
                                            util::guidance::TurnBearing(intersection[0].bearing),
                                            util::guidance::TurnBearing(turn->bearing)});
                                    }
                                    else
                                    {
                                        buffer->delayed_turn_data.push_back(TurnData{
                                            turn->instruction,
                                            turn->lane_data_id,
                                            entry_class_id,
                                            util::guidance::TurnBearing(intersection[0].bearing),
                                            util::guidance::TurnBearing(turn->bearing)});
                                    }
                                }
                            }
                        }
                    }
                }

                return buffer;
            });

        // Last part of the pipeline puts all the calculated data into the serial buffers
        util::Percent guidance_progress(log, node_count);
        std::vector<TurnData> delayed_turn_data;

        tbb::filter_t<TurnsPipelineBufferPtr, void> guidance_output_stage(
            tbb::filter::serial_in_order, [&](auto buffer) {

                guidance_progress.PrintAddition(buffer->nodes_processed);

                // Guidance data
                std::for_each(buffer->continuous_turn_data.begin(),
                              buffer->continuous_turn_data.end(),
                              [&turn_data_container](const auto &turn_data) {
                                  turn_data_container.push_back(turn_data);
                              });

                // Copy via-way restrictions delayed data
                delayed_turn_data.insert(delayed_turn_data.end(),
                                         buffer->delayed_turn_data.begin(),
                                         buffer->delayed_turn_data.end());
            });

        // Now, execute the pipeline.  The value of "5" here was chosen by experimentation
        // on a 16-CPU machine and seemed to give the best performance.  This value needs
        // to be balanced with the GRAINSIZE above - ideally, the pipeline puts as much work
        // as possible in the `intersection_handler` step so that those parallel workers don't
        // get blocked too much by the slower (io-performing) `buffer_storage`
        util::Log() << "Generating guidance turns ";
        current_node = 0;
        tbb::parallel_pipeline(tbb::task_scheduler_init::default_num_threads() * 5,
                               generator_stage & guidance_stage & guidance_output_stage);

        // NOTE: buffer.delayed_data and buffer.delayed_turn_data have the same index
        std::for_each(delayed_data.begin(), delayed_data.end(), transfer_data);
        std::for_each(delayed_turn_data.begin(),
                      delayed_turn_data.end(),
                      [&turn_data_container](const auto &turn_data) {
                          turn_data_container.push_back(turn_data);
                      });
    }

    util::Log() << "Created " << entry_class_hash.data.size() << " entry classes and "
                << bearing_class_hash.data.size() << " Bearing Classes";

    // TODO: move entry_class_hash and bearing_class_hash writing

    util::Log() << "Writing Turn Lane Data to File...";
    {
        storage::io::FileWriter writer(turn_lane_data_filename,
                                       storage::io::FileWriter::GenerateFingerprint);

        std::vector<util::guidance::LaneTupleIdPair> lane_data(lane_data_map.data.size());
        // extract lane data sorted by ID
        for (auto itr : lane_data_map.data)
            lane_data[itr.second] = itr.first;

        storage::serialization::write(writer, lane_data);
    }
    util::Log() << "done.";

    files::writeTurnData(turn_data_filename, turn_data_container);

    // TODO: add turns logging and timing output
    // util::Log() << "Generated " << m_edge_based_node_segments.size() << " edge based node
    segments";
    */
    return 0;
}

} // namespace customizer$
} // namespace osrm
