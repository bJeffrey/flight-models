#include "flight_models/navigation.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace flight_models;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Error, typename Function>
void rejects(Function operation, const char* message) {
  try {
    operation();
  } catch (const Error&) {
    return;
  }
  throw std::runtime_error(message);
}

PhysicalState state(double x, std::int64_t time) {
  PhysicalState value;
  value.position_ecef_m = {x, 2.0, 3.0};
  value.velocity_ecef_mps = {4.0, 5.0, 6.0};
  value.body_frd_to_ecef = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  value.source_time_ns = time;
  return value;
}

DesiredMotion desired() { return {0.0, 0.0, 0.0}; }

void explicit_selection_and_copy() {
  rejects<std::invalid_argument>([] { IdealNavigationBatch batch(2, ""); }, "implicit model accepted");
  rejects<std::invalid_argument>([] { IdealNavigationBatch batch(2, "nonideal"); }, "unimplemented model accepted");
  StateStore store(3, 1);
  store.spawn("zeta", 0, EntityMode::moving, state(10, 100), desired());
  const Handle alpha = store.spawn("alpha", 0, EntityMode::fixed, state(20, 200), desired());
  store.spawn("hidden", 0, EntityMode::disabled, state(30, 300), desired());
  Snapshot snapshot(3);
  store.snapshot(snapshot);

  IdealNavigationBatch batch(3, "ideal_test");
  batch.generate(snapshot);
  require(batch.size() == 2, "disabled row was published");
  require(batch.entity_id(0) == "alpha" && batch.entity_id(1) == "zeta", "snapshot order was not retained");
  const NavigationSolution first = batch.solution(0);
  require(first.position_ecef_m.x == 20 && first.velocity_ecef_mps.y == 5 &&
              first.body_frd_to_ecef[0] == 1 && first.source_time_ns == 200,
          "physical state was not copied into navigation solution");
  for (double value : first.covariance_ecef) require(value == 0.0, "ideal covariance is not zero");

  store.set_mode(alpha, EntityMode::disabled);
  require(batch.size() == 2 && batch.solution(0).position_ecef_m.x == 20,
          "published solution depended on later physical state");
  NavigationSolution returned = batch.solution(0);
  returned.position_ecef_m.x = -1;
  require(returned.position_ecef_m.x == -1 && batch.solution(0).position_ecef_m.x == 20,
          "solution accessor did not return an owning value");
  rejects<std::out_of_range>([&] { batch.solution(batch.size()); }, "out-of-range row accepted");
}

void rollback_and_capacity() {
  StateStore store(2, 1);
  store.spawn("one", 0, EntityMode::moving, state(1, 1), desired());
  Snapshot snapshot(2);
  store.snapshot(snapshot);
  IdealNavigationBatch batch(1, "ideal_test");
  batch.generate(snapshot);
  store.spawn("two", 0, EntityMode::fixed, state(2, 2), desired());
  store.snapshot(snapshot);
  rejects<std::length_error>([&] { batch.generate(snapshot); }, "over-capacity snapshot accepted");
  require(batch.size() == 1 && batch.entity_id(0) == "one" && batch.solution(0).source_time_ns == 1,
          "failed capacity update replaced published batch");
}

void state_store_rejects_malformed_physical_input() {
  StateStore store(2, 1);
  store.spawn("good", 0, EntityMode::moving, state(1, 1), desired());
  Snapshot snapshot(2);
  store.snapshot(snapshot);
  IdealNavigationBatch batch(2, "ideal_test");
  batch.generate(snapshot);
  auto bad_dcm = state(1, 1);
  bad_dcm.body_frd_to_ecef[0] = 2;
  store.spawn("bad-dcm", 0, EntityMode::moving, bad_dcm, desired());
  store.snapshot(snapshot);
  rejects<std::invalid_argument>([&] { batch.generate(snapshot); }, "navigation accepted malformed DCM");
  require(batch.size() == 1 && batch.solution(0).source_time_ns == 1,
          "malformed DCM update replaced published batch");
  auto bad_value = state(1, 1);
  bad_value.position_ecef_m.x = std::numeric_limits<double>::infinity();
  rejects<std::invalid_argument>([&] { store.spawn("bad-value", 0, EntityMode::moving, bad_value, desired()); },
                                 "state store accepted nonfinite state");
  auto bad_time = state(1, -1);
  rejects<std::invalid_argument>([&] { store.spawn("bad-time", 0, EntityMode::moving, bad_time, desired()); },
                                 "state store accepted negative time");
}

}  // namespace

int main() {
  try {
    explicit_selection_and_copy();
    rollback_and_capacity();
    state_store_rejects_malformed_physical_input();
    std::cout << "Navigation tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
