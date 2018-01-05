#ifndef OSRM_GUIDANCE_GUIDANCE_CONFIG_HPP
#define OSRM_GUIDANCE_GUIDANCE_CONFIG_HPP

#include <boost/filesystem/path.hpp>

#include "storage/io_config.hpp"

namespace osrm
{
namespace guidance
{

struct GuidanceConfig final : storage::IOConfig
{
    GuidanceConfig()
        : IOConfig({".osrm.ebg",
                    ".osrm.partition",
                    ".osrm.cells",
                    ".osrm.ebg_nodes",
                    ".osrm.properties"},
                   {},
                   {".osrm.cell_metrics", ".osrm.mldgr"}),
          requested_num_threads(0)
    {
    }

    void UseDefaultOutputNames(const boost::filesystem::path &base)
    {
        IOConfig::UseDefaultOutputNames(base);
    }

    boost::filesystem::path profile_path;

    unsigned requested_num_threads;
};
}
}

#endif // OSRM_GUIDANCE_GUIDANCE_CONFIG_HPP
