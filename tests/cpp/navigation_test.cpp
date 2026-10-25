#include "flight_models/navigation.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <numeric>

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
  rejects<std::invalid_argument>([] { NavigationBatch batch(2, NavigationConfig{"", {}, {}, 0, 0, 0}); }, "implicit model accepted");
  rejects<std::invalid_argument>([] { NavigationBatch batch(2, NavigationConfig{"nonideal", {}, {}, 0, 0, 0}); }, "unimplemented model accepted");
  StateStore store(3, 1);
  store.spawn("zeta", 0, EntityMode::moving, state(10, 100), desired());
  const Handle alpha = store.spawn("alpha", 0, EntityMode::fixed, state(20, 200), desired());
  store.spawn("hidden", 0, EntityMode::disabled, state(30, 300), desired());
  Snapshot snapshot(3);
  store.snapshot(snapshot);

  NavigationBatch batch(3, NavigationConfig{"ideal_test", {}, {}, 0, 0, 0});
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
  NavigationBatch batch(1, NavigationConfig{"ideal_test", {}, {}, 0, 0, 0});
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
  NavigationBatch batch(2, NavigationConfig{"ideal_test", {}, {}, 0, 0, 0});
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

void configured_navigation_is_identity_time_keyed() {
  NavigationConfig config{"bias_gaussian", {1, 0, 0}, {0, 2, 0}, 3.0, 0.5, 42};
  NavigationModel model(config);
  const auto source = state(100, 1234);
  const auto first = model.solution(source, "aircraft-a");
  const auto repeated = model.solution(source, "aircraft-a");
  require(first.position_ecef_m.x == repeated.position_ecef_m.x &&
          first.position_ecef_m.y == repeated.position_ecef_m.y &&
          first.velocity_ecef_mps.z == repeated.velocity_ecef_mps.z,
          "same identity and time did not reproduce the sample");
  require(first.position_ecef_m.x != source.position_ecef_m.x + 1,
          "configured Gaussian noise was not applied");
  require(first.covariance_ecef[0] == 9 && first.covariance_ecef[7] == 9 &&
          first.covariance_ecef[14] == 9 && first.covariance_ecef[21] == .25 &&
          first.covariance_ecef[28] == .25 && first.covariance_ecef[35] == .25,
          "ECEF covariance diagonal does not match configured sigmas");
  for (int r=0;r<6;++r) for(int c=0;c<6;++c) if(r!=c)
    require(first.covariance_ecef[r*6+c] == 0, "navigation covariance has cross terms");
  NavigationConfig invalid{"ideal_test", {1,0,0}, {}, 0, 0, 0};
  rejects<std::invalid_argument>([&] { NavigationModel rejected(invalid); }, "ideal model accepted bias");
  invalid = {"bias_gaussian", {}, {}, 1e308, 0, 0};
  rejects<std::invalid_argument>([&] { NavigationModel rejected(invalid); }, "variance overflow accepted");
  PhysicalState huge = source;
  huge.position_ecef_m.x = std::numeric_limits<double>::max();
  NavigationConfig overflowing{"bias_gaussian", {std::numeric_limits<double>::max(),0,0}, {}, 0, 0, 0};
  rejects<std::overflow_error>([&] { (void)NavigationModel(overflowing).solution(huge, "edge"); },
                               "reported output overflow was not rejected");
  NavigationConfig exact_a{"bias_gaussian", {}, {}, 0, 0, 3};
  NavigationConfig exact_b = exact_a;
  exact_b.seed = 987654321;
  const auto exact_solution = NavigationModel(exact_a).solution(source, "exact");
  const auto changed_seed_solution = NavigationModel(exact_b).solution(source, "exact");
  require(exact_solution.position_ecef_m.x == changed_seed_solution.position_ecef_m.x &&
          exact_solution.velocity_ecef_mps.z == changed_seed_solution.velocity_ecef_mps.z,
          "all-zero error path changed with seed");
  NavigationConfig bias_only{"bias_gaussian", {1,-2,3}, {0.1,0.2,0.3}, 0, 0, 8};
  const auto biased = NavigationModel(bias_only).solution(source, "biased");
  require(biased.position_ecef_m.x == source.position_ecef_m.x + 1 &&
          biased.velocity_ecef_mps.y == source.velocity_ecef_mps.y + 0.2,
          "bias-only configuration did not apply exact offsets");

  NavigationConfig position_only{"bias_gaussian", {}, {}, 2.0, 0.0, 991};
  NavigationConfig both = position_only;
  both.velocity_sigma_mps = 0.75;
  NavigationConfig velocity_only{"bias_gaussian", {}, {}, 0.0, 0.75, 991};
  const auto p = NavigationModel(position_only).solution(source, "aircraft-a");
  const auto b = NavigationModel(both).solution(source, "aircraft-a");
  const auto v = NavigationModel(velocity_only).solution(source, "aircraft-a");
  require(p.position_ecef_m.x == b.position_ecef_m.x && p.position_ecef_m.y == b.position_ecef_m.y &&
          p.position_ecef_m.z == b.position_ecef_m.z, "velocity stream changed position samples");
  require(b.velocity_ecef_mps.x == v.velocity_ecef_mps.x && b.velocity_ecef_mps.y == v.velocity_ecef_mps.y &&
          b.velocity_ecef_mps.z == v.velocity_ecef_mps.z, "position stream changed velocity samples");
  const auto different_id = model.solution(source, "aircraft-b");
  auto later = source;
  later.source_time_ns++;
  const auto different_time = model.solution(later, "aircraft-a");
  NavigationConfig other_seed = config;
  ++other_seed.seed;
  const auto different_seed = NavigationModel(other_seed).solution(source, "aircraft-a");
  require(different_id.position_ecef_m.x != first.position_ecef_m.x &&
          different_time.position_ecef_m.x != first.position_ecef_m.x &&
          different_seed.position_ecef_m.x != first.position_ecef_m.x,
          "identity, time, or seed did not affect keyed samples");

  // Across 12,000 independently keyed samples, the mean should be within six
  // standard errors and sample variance within six Gaussian standard errors.
  constexpr int samples = 12000;
  NavigationModel statistics({"bias_gaussian", {}, {}, 4.0, 0, 8675309});
  std::vector<double> draws;
  draws.reserve(samples);
  for (int i = 0; i < samples; ++i) {
    auto sample_state = source;
    sample_state.source_time_ns = i;
    draws.push_back(statistics.solution(sample_state, "stats").position_ecef_m.x - source.position_ecef_m.x);
  }
  const double mean = std::accumulate(draws.begin(), draws.end(), 0.0) / samples;
  double squared = 0;
  for (double draw : draws) squared += (draw - mean) * (draw - mean);
  const double variance = squared / (samples - 1);
  require(std::abs(mean) < 6.0 * 4.0 / std::sqrt(samples), "fixed-seed Gaussian mean outside six-sigma tolerance");
  require(std::abs(variance - 16.0) < 6.0 * 16.0 * std::sqrt(2.0 / (samples - 1)),
          "fixed-seed Gaussian variance outside six-sigma tolerance");

  NavigationBatch batch(2, config);
  StateStore store(3, 1);
  store.spawn("aircraft-a", 0, EntityMode::moving, source, desired());
  store.spawn("aircraft-b", 0, EntityMode::fixed, state(20, 200), desired());
  Snapshot snapshot(3);
  store.snapshot(snapshot);
  batch.generate(snapshot);
  require(batch.solution(0).position_ecef_m.x == model.solution(source, "aircraft-a").position_ecef_m.x &&
          batch.solution(1).position_ecef_m.x == model.solution(state(20, 200), "aircraft-b").position_ecef_m.x,
          "batch samples differ from scalar keyed samples");
  StateStore reversed_store(3, 1);
  reversed_store.spawn("aircraft-b", 0, EntityMode::fixed, state(20, 200), desired());
  reversed_store.spawn("aircraft-a", 0, EntityMode::moving, source, desired());
  Snapshot reversed_snapshot(3);
  reversed_store.snapshot(reversed_snapshot);
  NavigationBatch reversed_batch(3, config);
  reversed_batch.generate(reversed_snapshot);
  for (std::size_t row = 0; row < batch.size(); ++row) {
    const std::string& id = batch.entity_id(row);
    std::size_t reverse_row = 0;
    while (reverse_row < reversed_batch.size() && reversed_batch.entity_id(reverse_row) != id) ++reverse_row;
    require(reverse_row < reversed_batch.size() &&
            reversed_batch.solution(reverse_row).position_ecef_m.x == batch.solution(row).position_ecef_m.x,
            "batch membership/order changed identity-keyed samples");
  }
  const double published = batch.solution(0).position_ecef_m.x;
  store.spawn("overflow", 0, EntityMode::moving, state(30, 300), desired());
  store.snapshot(snapshot);
  rejects<std::length_error>([&] { batch.generate(snapshot); }, "over-capacity batch accepted");
  require(batch.solution(0).position_ecef_m.x == published && batch.entity_id(0) == "aircraft-a",
          "failed batch replaced published data");
}

}  // namespace

int main() {
  try {
    explicit_selection_and_copy();
    rollback_and_capacity();
    state_store_rejects_malformed_physical_input();
    configured_navigation_is_identity_time_keyed();
    std::cout << "Navigation tests passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
