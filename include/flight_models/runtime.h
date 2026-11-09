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

/** Typed in-memory view of a runtime continuation; encode_checkpoint defines its JSON form. */
struct RuntimeCheckpoint {
  nlohmann::json configuration; /**< Exact immutable runtime input document. */
  MotionProfile profile; /**< Admitted profile snapshot, independent of later file edits. */
  MotionDriverCheckpoint driver; /**< Committed physical state and program progress. */
  bool started{}; /**< Whether the initial time-zero boundary was processed. */
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
  /** Advance to a per-call processing boundary while retaining announced creation time.
   * This API supports explicit bounded intervals; it does not set a run duration.
   * @param time_ns Physical processing boundary for this call.
   * @param creation_time_ns Announced simulation time, at or after time_ns.
   * @return Same scheduled samples as advance_to(time_ns), with PRD creation
   *         stamped at creation_time_ns. Validity timestamps remain unchanged.
   */
  std::vector<RuntimeOutput> advance_to(std::int64_t time_ns, std::int64_t creation_time_ns);

  /**
   * Capture a process-local continuation, including the admitted configuration.
   * @return Owning state without transport queues or clock/epoch control.
   * @throws std::logic_error if the runtime is failed or moved-from.
   */
  RuntimeCheckpoint checkpoint() const;
  /**
   * Atomically replace this runtime from a compatible continuation using a fresh candidate.
   * @param checkpoint Process-local capture with the same admitted configuration and profile.
   * Failed validation leaves this runtime's state unchanged; no outputs are published.
   * @throws std::invalid_argument if immutable configuration/profile or payload is incompatible.
   * @throws std::logic_error if this runtime is moved-from.
   */
  void restore(const RuntimeCheckpoint& checkpoint);

  /**
   * Encode the current continuation as versioned UTF-8 JSON.
   * Format is `flight-models.runtime-checkpoint`, version 1. Compatibility is
   * limited to the same software/toolchain and exact admitted configuration.
   * Callers own durable storage, payload integrity, restore scheduling, and clock/epoch
   * advancement; this API defines no transport or distributed-coordination semantics.
   * @return Owning UTF-8 bytes without process-local pointers or output buffers.
   * @throws std::logic_error if the runtime is failed or moved-from.
   * @throws std::invalid_argument if the admitted state cannot be represented.
   */
  std::string encode_checkpoint() const;

  /**
   * Validate and prepare a candidate runtime from a versioned JSON continuation.
   * The current runtime is unchanged; install the returned candidate with move assignment.
   * @param payload UTF-8 JSON emitted by encode_checkpoint().
   * Configuration and every typed admitted profile value must match this runtime.
   * The candidate publishes no outputs until the caller advances it.
   * @return A validated, uninstalled runtime with no retained captured-boundary outputs.
   * @throws std::invalid_argument for malformed, unsupported, or incompatible payloads.
   * @throws std::logic_error if this runtime is moved-from.
   */
  Runtime prepare_restore(const std::string& payload) const;

 private:
  struct Impl;
  explicit Runtime(std::unique_ptr<Impl> candidate) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flight_models
