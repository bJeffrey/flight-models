#include "runtime_support/simulation_clock.hpp"
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition) { if (!condition) throw std::runtime_error("clock admission check failed"); }
template<typename Error, typename Operation> void rejects(Operation operation) {
  try { operation(); } catch (const Error&) { return; }
  throw std::runtime_error("invalid clock input accepted");
}
rlbm::proto::SimTimeTick initial() {
  rlbm::proto::SimTimeTick tick;
  tick.set_run_id("run"); tick.set_scenario_epoch_utc("2026-10-24T00:00:00Z");
  tick.set_paused(true); tick.set_timeline_epoch(3);
  return tick;
}
}
int main() {
  using runtime_support::SimulationClock;
  const auto origin = runtime_support::parse_uci_timestamp("2026-10-24T00:00:00.000000000Z");
  SimulationClock clock("run", origin, 2, std::numeric_limits<std::int64_t>::max());
  rejects<std::logic_error>([&] { (void)clock.now(); });
  auto tick = initial();
  require(clock.accept(tick) == 0);
  require(clock.now().unix_ns == origin.unix_ns);
  require(!clock.accept(tick)); // Initial/paused heartbeat is idempotent.
  tick.set_paused(false); require(!clock.accept(tick)); // Resume itself advances no time.
  tick.set_advances_time(true); tick.set_tick_id(1); tick.set_t_us(100); tick.set_dt_us(100);
  require(clock.accept(tick) == 100'000);
  require(clock.now().unix_ns == origin.unix_ns + 100'000);
  require(!clock.accept(tick));
  const auto first = tick;
  tick.set_tick_id(2); tick.set_t_us(200); require(clock.accept(tick) == 200'000);
  require(!clock.accept(first)); // Replayed earlier advancing interval changes nothing.
  auto conflict = first; conflict.set_dt_us(50);
  rejects<std::invalid_argument>([&] { (void)clock.accept(conflict); });
  tick.set_tick_id(3); tick.set_t_us(300); require(clock.accept(tick) == 300'000);
  rejects<std::invalid_argument>([&] { (void)clock.accept(first); }); // Evicted replay fails explicitly.
  auto pause = tick; pause.set_advances_time(false); pause.set_dt_us(0); pause.set_paused(true);
  require(!clock.accept(pause));
  auto bad = pause; bad.set_t_us(301);
  rejects<std::invalid_argument>([&] { (void)clock.accept(bad); });
  bad = tick; bad.set_tick_id(5); bad.set_t_us(500); bad.set_dt_us(200);
  rejects<std::invalid_argument>([&] { (void)clock.accept(bad); });
  bad = tick; bad.set_timeline_epoch(4);
  rejects<std::invalid_argument>([&] { (void)clock.accept(bad); });
  bad = tick; bad.set_scenario_epoch_utc("2026-10-24T00:00:01Z");
  rejects<std::invalid_argument>([&] { (void)clock.accept(bad); });
  bad = tick; bad.set_run_id("other");
  rejects<std::invalid_argument>([&] { (void)clock.accept(bad); });
  bad = tick; bad.set_t_us(std::numeric_limits<std::uint64_t>::max());
  rejects<std::overflow_error>([&] { (void)clock.accept(bad); });
  SimulationClock joining("run", origin, 2, std::numeric_limits<std::int64_t>::max());
  rejects<std::invalid_argument>([&] { (void)joining.accept(tick); });
  SimulationClock utc_limited("run", origin, 2, 500);
  require(utc_limited.accept(initial()) == 0);
  auto too_late = initial();
  too_late.set_paused(false); too_late.set_advances_time(true);
  too_late.set_tick_id(1); too_late.set_t_us(1); too_late.set_dt_us(1);
  rejects<std::overflow_error>([&] { (void)utc_limited.accept(too_late); });
  require(!utc_limited.accept(initial())); // Rejection did not mutate processing state.

  auto invalid_origin = initial();
  invalid_origin.set_scenario_epoch_utc("2026-02-30T00:00:00Z");
  rejects<std::invalid_argument>([&] { (void)clock.accept(invalid_origin); });

  const auto near_limit = runtime_support::Instant{std::numeric_limits<std::int64_t>::max() - 500};
  SimulationClock overflow_clock("run", near_limit, 2, std::numeric_limits<std::int64_t>::max());
  auto near_initial = initial();
  near_initial.set_scenario_epoch_utc(runtime_support::format_uci_timestamp(near_limit));
  require(overflow_clock.accept(near_initial) == 0);
  auto overflowing_tick = near_initial;
  overflowing_tick.set_paused(false); overflowing_tick.set_advances_time(true);
  overflowing_tick.set_tick_id(1); overflowing_tick.set_t_us(1); overflowing_tick.set_dt_us(1);
  rejects<std::overflow_error>([&] { (void)overflow_clock.accept(overflowing_tick); });
  require(!overflow_clock.accept(near_initial)); // Overflow rejection leaves the clock at time zero.
  require(overflow_clock.now().unix_ns == near_limit.unix_ns);
}
