/** @file Fresh-run authoritative clock admission; independent of pacing. */
#pragma once
#include "sim_time.pb.h"
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace flight_models {
/** Admit ordered authoritative intervals without an additional tick ACK protocol.
 * Pause/heartbeats finish no new domain work. Accepted advancing duplicates are
 * idempotent within the configured bounded replay window. Unknown older ticks,
 * gaps, changed runs/origins/epochs and conflicting intervals fail explicitly;
 * restoring a service is outside this initial fresh-run process.
 */
class ClockAdmission {
 public:
  /** @param run_id Expected scenario run identity.
   * @param epoch_utc Expected UTC scenario origin (not the timeline epoch counter).
   * @param replay_window Maximum retained advancing fingerprints; must be positive.
   * @param maximum_elapsed_ns Largest elapsed time whose creation timestamp is
   *        representable for the configured absolute scenario origin.
   */
  ClockAdmission(std::string run_id, std::string epoch_utc, std::size_t replay_window,
                 std::int64_t maximum_elapsed_ns);
  /** @return Elapsed source ns when work is due, nullopt for heartbeat/duplicate.
   * First admission must be nonadvancing time/tick zero; any initial timeline
   * epoch is retained and frozen for this process lifetime.
   * @throws std::invalid_argument for inconsistent/gapped/fenced clock input.
   * @throws std::overflow_error for nonrepresentable elapsed nanoseconds.
   */
  std::optional<std::int64_t> accept(const rlbm::proto::SimTimeTick& tick);
 private:
  std::string run_id_, epoch_utc_;
  std::size_t replay_window_;
  std::int64_t maximum_elapsed_ns_;
  bool started_{};
  std::uint32_t timeline_epoch_{};
  std::uint64_t last_tick_{}, last_us_{};
  struct Fingerprint { std::uint64_t t_us, dt_us; };
  std::map<std::uint64_t, Fingerprint> history_;
};
}  // namespace flight_models
