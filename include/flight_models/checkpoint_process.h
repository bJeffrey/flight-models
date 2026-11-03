/** @file Simulation-only native checkpoint/control process binding. */
#pragma once

#include <nlohmann/json.hpp>

namespace flight_models {
/**
 * Serve one finite scenario with explicitly configured native checkpoint support.
 * @param config Validated process configuration containing a checkpoint profile.
 * @return Zero on finite completion or signal termination; failures throw to main.
 * This host serializes authoritative ticks and checkpoint requests. It owns no
 * pacing, distributed catalog or input replay and never enables other services.
 */
int run_checkpoint_process(const nlohmann::json& config);
}  // namespace flight_models
