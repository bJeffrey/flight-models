/** @file Checkpoint-runtime adapter for the owning flight-models continuation. */
#pragma once

#include "checkpoint_runtime/participant.hpp"
#include "flight_models/runtime.h"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace flight_models {

/** Binds Runtime continuation and output publication to a checkpoint participant.
 * This adapter has no clock, transport, or coordinator policy. Its host must
 * publish the precomputed time-zero output only after authoritative tick zero
 * has been accepted, and must fence processing before restore/apply operations.
 * A cold owner may prepare and apply recovery before publishing that initial output.
 */
class CheckpointStateOwner final : public checkpoint_runtime::StateOwner {
 public:
  /**
   * @param config Exact runtime configuration, including nonnegative end_time_ns.
   * @param publish Synchronous output boundary; it receives owning samples from
   *        one processed runtime boundary. It must not retain borrowed objects.
   * Construction processes time zero internally but publishes nothing.
   * @throws std::invalid_argument for invalid end time or an empty publisher.
   */
  CheckpointStateOwner(const nlohmann::json& config,
                       std::function<void(const std::vector<RuntimeOutput>&)> publish);
  CheckpointStateOwner(const CheckpointStateOwner&) = delete;
  CheckpointStateOwner& operator=(const CheckpointStateOwner&) = delete;
  CheckpointStateOwner(CheckpointStateOwner&&) = delete;
  CheckpointStateOwner& operator=(CheckpointStateOwner&&) = delete;

  /** Publish the precomputed initial sample exactly once after authority admission.
   * A throwing callback leaves initialization incomplete and permanently prevents
   * retrying that publication because delivery may have been partial.
   * @throws std::logic_error if publication was already attempted.
   */
  void publish_initial();

  /** @return Whether initial publication or coordinated restore established usable state. */
  bool initialized() const noexcept;

  /** @return Last fully published elapsed time in microseconds. */
  std::uint64_t current_t_us() const override;
  /** @return Current delivery epoch; starts at zero. */
  std::uint32_t current_timeline_epoch() const override;
  /** @return Versioned owning Runtime continuation; requires initialization. */
  std::string capture_state() const override;
  /**
   * Validate a continuation without mutating live state or publishing outputs.
   * @param payload Versioned Runtime checkpoint JSON.
   * @param target_epoch Strictly newer destination delivery epoch.
   * @return Uninstalled candidate with whole-microsecond elapsed time within
   *         the configured scenario endpoint.
   * @throws std::invalid_argument for incompatible time, epoch, or payload.
   */
  std::unique_ptr<checkpoint_runtime::RestoreCandidate> prepare_restore(
      const std::string& payload, std::uint32_t target_epoch) const override;
  /** Install a prepared candidate while fenced; emits no output. */
  void apply_restore(std::unique_ptr<checkpoint_runtime::RestoreCandidate> candidate) override;
  /**
   * Advance one whole-microsecond elapsed boundary and publish its outputs.
   * @param t_us Monotonic elapsed scenario time in microseconds.
   * @throws std::logic_error before initial publication.
   * @throws std::invalid_argument for backward, unrepresentable, or out-of-range time.
   * Callback or Runtime failures may leave domain state partially advanced; the
   * participant must fence and require recovery before any further processing.
   */
  void advance_to(std::uint64_t t_us) override;

 private:
  Runtime runtime_;
  std::function<void(const std::vector<RuntimeOutput>&)> publish_;
  std::vector<RuntimeOutput> held_initial_;
  std::int64_t end_time_ns_{};
  std::uint32_t epoch_{};
  std::uint64_t processed_t_us_{};
  bool initialization_attempted_{};
  bool initialized_{};
};

}  // namespace flight_models
