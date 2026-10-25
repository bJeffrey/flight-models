#include "flight_models/clock_admission.h"
#include "flight_models/process_config.h"
#include <limits>
#include <stdexcept>
#include <utility>

namespace flight_models {
ClockAdmission::ClockAdmission(std::string run_id, std::string epoch_utc, std::size_t replay_window,
                               std::int64_t maximum_elapsed_ns)
    : run_id_(std::move(run_id)), epoch_utc_(canonical_scenario_epoch(epoch_utc)),
      replay_window_(replay_window), maximum_elapsed_ns_(maximum_elapsed_ns) {
  if (run_id_.empty() || replay_window_ == 0 || maximum_elapsed_ns_ < 0) {
    throw std::invalid_argument("clock admission requires run identity, replay and timestamp bounds");
  }
}

std::optional<std::int64_t> ClockAdmission::accept(const rlbm::proto::SimTimeTick& tick) {
  if (tick.run_id() != run_id_ || canonical_scenario_epoch(tick.scenario_epoch_utc()) != epoch_utc_) {
    throw std::invalid_argument("clock run/origin does not match configured scenario");
  }
  constexpr auto maximum_us = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000);
  if (tick.t_us() > maximum_us || tick.dt_us() > maximum_us) {
    throw std::overflow_error("clock microseconds exceed signed nanosecond range");
  }
  if (tick.t_us() > static_cast<std::uint64_t>(maximum_elapsed_ns_ / 1000)) {
    throw std::overflow_error("clock creation time exceeds configured UTC origin's representable range");
  }
  if (!started_) {
    if (tick.t_us() != 0 || tick.dt_us() != 0 || tick.tick_id() != 0 || tick.advances_time()) {
      throw std::invalid_argument("fresh runtime requires clock time/tick zero; late join needs restoration");
    }
    timeline_epoch_ = tick.timeline_epoch();
    started_ = true;
    return 0;
  }
  if (tick.timeline_epoch() != timeline_epoch_) {
    throw std::invalid_argument("timeline epoch changed; restore is not implemented for this fresh-run process");
  }
  if (!tick.advances_time()) {
    if (tick.dt_us() != 0 || tick.t_us() != last_us_ || tick.tick_id() != last_tick_) {
      throw std::invalid_argument("nonadvancing clock input changed the processed boundary");
    }
    return std::nullopt;
  }
  if (tick.paused() || tick.dt_us() == 0) throw std::invalid_argument("advancing clock interval cannot be paused or empty");

  const auto replay = history_.find(tick.tick_id());
  if (replay != history_.end()) {
    if (replay->second.t_us != tick.t_us() || replay->second.dt_us != tick.dt_us()) {
      throw std::invalid_argument("conflicting authoritative clock replay");
    }
    return std::nullopt;
  }
  if (last_tick_ == std::numeric_limits<std::uint64_t>::max() || tick.tick_id() != last_tick_ + 1 ||
      tick.t_us() <= last_us_ || tick.t_us() - last_us_ != tick.dt_us()) {
    throw std::invalid_argument("clock interval is missing, reordered or outside the retained replay window");
  }
  history_.emplace(tick.tick_id(), Fingerprint{tick.t_us(), tick.dt_us()});
  if (history_.size() > replay_window_) history_.erase(history_.begin());
  last_tick_ = tick.tick_id();
  last_us_ = tick.t_us();
  return static_cast<std::int64_t>(tick.t_us() * 1000);
}
}  // namespace flight_models
