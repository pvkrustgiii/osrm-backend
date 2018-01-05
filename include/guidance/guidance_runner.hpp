#ifndef OSRM_GUIDANCE_GUIDANCE_RUNNER_HPP
#define OSRM_GUIDANCE_GUIDANCE_RUNNER_HPP

#include "guidance/guidance_config.hpp"

namespace osrm
{
namespace guidance
{

class GuidanceRunner
{
  public:
    int Run(const GuidanceConfig &config);
};

} // namespace customizer
} // namespace osrm

#endif // OSRM_GUIDANCE_GUIDANCE_RUNNER_HPP
