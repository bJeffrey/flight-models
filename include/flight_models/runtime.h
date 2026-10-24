/** @file Fresh-run owner joining configured motion, physical samples and adapters. */
#pragma once

#include "flight_models/dis_entity_state.h"
#include "flight_models/motion_driver.h"
#include "flight_models/motion_profile.h"
#include "flight_models/position_report.h"

#include <nlohmann/json.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace flight_models {

/** One owning adapter result at a driver sample boundary. */
struct RuntimeOutput {
  std::string entity_id; /**< Configured entity identity. */
  SampleKind kind{}; /**< Physical DIS or reported navigation PRD. */
  std::vector<std::uint8_t> dis; /**< Encoded DIS PDU; empty for navigation rows. */
  nlohmann::json prd; /**< Encoded PRD; null when this entity has no PRD config. */
};

/** Bounded, single-owner simulation runtime for one fresh configured cohort. */
class Runtime {
 public:
  /**
   * Load the explicit profile and initialize all configured entities at time zero.
   * @param config Explicit engine inputs described in the runtime planning
   *        checkpoint. Deployment schema/transport admission remains pending.
   * @throws std::invalid_argument for invalid configuration or entity state.
   * @throws std::runtime_error when configured profile/schema files cannot be loaded.
   */
  explicit Runtime(const nlohmann::json& config);
  ~Runtime();
  Runtime(Runtime&&) noexcept;
  Runtime& operator=(Runtime&&) noexcept;
  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  /**
   * Advance through all due events and return owning encoded outputs.
   * @param time_ns Monotonic elapsed scenario nanoseconds; first call must be zero.
   * @return Physical samples for every entity and navigation samples for PRD entities.
   * An adapter encoding failure after the state advance makes this runtime unusable;
   * callers must fail the run and cannot retry the same boundary as atomic output.
   * @throws std::invalid_argument for invalid/nonmonotonic time or driver failure.
   * @throws std::length_error when configured event/output bounds are exceeded.
   */
  std::vector<RuntimeOutput> advance_to(std::int64_t time_ns);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flight_models
