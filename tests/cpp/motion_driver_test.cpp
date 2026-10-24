#include "flight_models/motion_driver.h"
#include "flight_models/bounded_motion.h"
#include "math/coordinates/geodetic.hpp"

#include <nlohmann/json.hpp>
#include <valijson/adapters/nlohmann_json_adapter.hpp>
#include <valijson/schema.hpp>
#include <valijson/schema_parser.hpp>
#include <valijson/validator.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace test_allocation {
void forbid_heap_allocations(bool forbid) noexcept;
}

namespace {
using namespace flight_models;
namespace geo = math::coordinates;
constexpr std::int64_t second = 1000000000;
constexpr double pi = 3.14159265358979323846;

class NoAllocation {
 public:
  NoAllocation() { test_allocation::forbid_heap_allocations(true); }
  ~NoAllocation() { test_allocation::forbid_heap_allocations(false); }
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance, const char* message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}
template <typename Function>
void rejects(Function function, const char* message) {
  try { function(); }
  catch (const std::invalid_argument&) { return; }
  catch (const std::length_error&) { return; }
  throw std::runtime_error(message);
}

/** Explicit fixture capacities; the driver does not load deployment settings. */
DriverLimits limits_from(const std::string& root) {
  std::ifstream config(root + "/config/event_policy.json");
  std::ifstream definition(root + "/config/event_policy.schema.json");
  const auto document = nlohmann::json::parse(config);
  const auto schema_json = nlohmann::json::parse(definition);
  valijson::Schema schema;
  valijson::SchemaParser parser;
  valijson::adapters::NlohmannJsonAdapter schema_adapter(schema_json);
  parser.populateSchema(schema_adapter, schema);
  valijson::Validator validator;
  valijson::adapters::NlohmannJsonAdapter adapter(document);
  require(validator.validate(schema, adapter, nullptr), "invalid fixture event policy");
  return {document.at("max_commands"), document.at("max_waypoints"), document.at("max_samples"),
          document.at("max_transitions"), document.at("max_events")};
}

PhysicalState initial(bool fixed = false) {
  PhysicalState state;
  state.position_ecef_m = {6379137, 0, 0};
  state.velocity_ecef_mps = {0, fixed ? 0.0 : 100.0, 0};
  state.body_frd_to_ecef = {0, 0, -1, 1, 0, 0, 0, -1, 0};
  return state;
}
Handle spawn(StateStore& store, const std::string& id, EntityMode mode = EntityMode::moving) {
  return store.spawn(id, 0, mode, initial(mode == EntityMode::fixed), {pi / 2, 100, 1000});
}
PhysicalState read(const StateStore& store, Handle entity) {
  PhysicalState state;
  DesiredMotion desired;
  require(store.read_state(entity, state, desired), "fixture handle became stale");
  return state;
}
Waypoint east(double distance_m) {
  return {geo::geodetic_to_ecef({0, distance_m / 6379137.0, 1000})};
}
double distance(math::Vector3 a, math::Vector3 b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

void cadences_and_coincident_command(MotionProfile profile, DriverLimits limits) {
  profile.publication = {70000000, 110000000};
  StateStore store(4, 1);
  const auto zulu = spawn(store, "zulu");
  const auto alpha = spawn(store, "alpha");
  const auto fixed = spawn(store, "fixed", EntityMode::fixed);
  const auto disabled = spawn(store, "disabled", EntityMode::disabled);
  EntityProgram program{alpha, true, {}, {
      {110000000, 1, {pi, 100, 1000}}, {110000000, 2, {0, 100, 1000}}}};
  MotionDriver driver(store, 0, profile, limits, {program, {zulu, false, {}, {}}, {fixed, false, {}, {}}});
  driver.advance_to(250000000);
  require(driver.samples().size() == 15, "independent sample deadlines were lost or coalesced");
  std::size_t physical = 0, navigation = 0;
  std::int64_t previous_time = -1;
  bool saw_command_sample = false;
  for (const auto& sample : driver.samples()) {
    require(sample.state.source_time_ns >= previous_time, "sample ordering went backward");
    previous_time = sample.state.source_time_ns;
    if (sample.kind == SampleKind::physical) {
      ++physical;
      require(sample.state.source_time_ns % 70000000 == 0, "physical deadline rounded");
    } else {
      ++navigation;
      require(sample.entity == alpha && sample.state.source_time_ns % 110000000 == 0,
              "navigation membership or deadline incorrect");
      if (sample.state.source_time_ns == 110000000) {
        saw_command_sample = true;
        near(sample.desired.heading_rad, 0, 0, "same-time commands ignored sequence order");
        require(sample.state.angular_velocity_body_radps.z < 0,
                "sample retained pre-command turn derivative");
        near(std::hypot(sample.state.velocity_ecef_mps.x, sample.state.velocity_ecef_mps.y),
             100, 1e-8, "heading command changed speed instantly");
      }
    }
  }
  require(physical == 12 && navigation == 3 && saw_command_sample, "wrong publication counts");
  require(driver.samples()[0].entity == alpha && driver.samples()[1].entity == fixed &&
          driver.samples()[2].entity == zulu, "physical samples are not canonical ID order");
  require(read(store, disabled).source_time_ns == 0, "disabled state was updated");
  require(read(store, fixed).source_time_ns == 250000000 &&
          distance(read(store, fixed).position_ecef_m, initial(true).position_ecef_m) == 0,
          "fixed entity did not retain its position and advance validity");
  const auto previous_count = driver.samples().size();
  driver.advance_to(250000000);
  require(driver.samples().size() == previous_count, "same-time call regenerated output");
}

void exact_thirty_second_turn(const MotionProfile& profile, DriverLimits limits) {
  StateStore store(1, 1);
  const auto entity = spawn(store, "turn-at-thirty");
  MotionDriver driver(store, 0, profile, limits,
      {{entity, true, {}, {{30 * second, 1, {0, 100, 1000}}}}});
  for (std::int64_t time = 2 * second; time <= 30 * second; time += 2 * second) driver.advance_to(time);
  const auto expected = geo::geodetic_to_ecef({0, 3000 / 6379137.0, 1000});
  near(distance(read(store, entity).position_ecef_m, expected), 0, 1e-4,
       "thirty-second command influenced preceding motion");
  bool saw = false;
  for (const auto& sample : driver.samples()) {
    if (sample.state.source_time_ns == 30 * second) {
      saw = true;
      near(sample.desired.heading_rad, 0, 0, "command missing from boundary snapshot");
      require(sample.state.angular_velocity_body_radps.z < 0, "boundary derivative did not apply new command");
    }
  }
  require(saw, "coincident publication at thirty seconds lost");
  driver.advance_to(31 * second);
  require(read(store, entity).position_ecef_m.z > 1, "turn did not begin after command time");
}

void swept_waypoints(MotionProfile profile, DriverLimits limits) {
  profile.waypoint_capture_radius_m = 0.25;
  profile.integration.max_substep_ns = 20000000;
  StateStore store(1, 1);
  const auto entity = spawn(store, "route");
  MotionDriver driver(store, 0, profile, limits,
      {{entity, false, {east(0), east(10), east(20), east(30)}, {}}});
  driver.advance_to(second / 2);
  require(driver.waypoint_progress(entity) == 4 && driver.transitions().size() == 4,
          "swept path skipped a narrow waypoint");
  for (std::size_t index = 0; index < 4; ++index) {
    const auto& transition = driver.transitions()[index];
    require(transition.entity == entity && transition.waypoint_index == index,
            "waypoint transitions are out of order");
    const double expected = index == 0 ? 0 : (10.0 * index - 0.25) / 100 * second;
    near(static_cast<double>(transition.time_ns), expected, 10,
         "capture was rounded to the enclosing numerical step");
  }
  // Named-ENU conversion belongs at input admission, not in the route loop.
  const geo::GeodeticPosition origin{0, 0, 1000};
  const auto enu_point = geo::enu_to_geodetic({10, 0, 0}, origin);
  require(distance(geo::geodetic_to_ecef(enu_point), east(10).position_ecef_m) < 1e-5,
          "explicit-origin ENU fixture did not map to the same ECEF waypoint");
}

void atomic_failure_and_retry(const MotionProfile& profile, DriverLimits limits) {
  limits.max_samples = 1;
  StateStore store(1, 1);
  const auto entity = spawn(store, "bounded-output");
  MotionDriver driver(store, 0, profile, limits, {{entity, false, {}, {}}});
  driver.advance_to(0);
  require(driver.samples().size() == 1, "initial snapshot missing");
  const auto before = read(store, entity);
  rejects([&] { driver.advance_to(2 * profile.publication.dis_period_ns); }, "sample overflow silently grew");
  require(driver.time_ns() == 0 && read(store, entity).source_time_ns == before.source_time_ns &&
          driver.samples().size() == 1 && driver.samples()[0].state.source_time_ns == 0,
          "failed interval leaked state, time, or output");
  driver.advance_to(profile.publication.dis_period_ns);
  require(driver.time_ns() == profile.publication.dis_period_ns && driver.samples().size() == 1,
          "failed attempt consumed successful-retry deadlines");
}

void guidance_turn_and_progress_rollback(MotionProfile profile, DriverLimits limits) {
  StateStore turning(1, 1);
  const auto aircraft = spawn(turning, "guidance-turn");
  const auto north = geo::geodetic_to_ecef(geo::enu_to_geodetic({200, 5000, 0}, {0, 0, 1000}));
  MotionDriver guidance(turning, 0, profile, limits, {{aircraft, false, {east(200), {north}}, {}}});
  guidance.advance_to(2 * second);
  require(guidance.waypoint_progress(aircraft) == 1 &&
          read(turning, aircraft).position_ecef_m.z > 1,
          "guidance did not turn toward the next waypoint");
  auto finer = profile;
  finer.integration.max_substep_ns = 1000000;
  StateStore reference(1, 1);
  const auto reference_aircraft = spawn(reference, "guidance-turn");
  MotionDriver fine_guidance(reference, 0, finer, limits,
      {{reference_aircraft, false, {east(200), {north}}, {}}});
  fine_guidance.advance_to(2 * second);
  require(fine_guidance.waypoint_progress(reference_aircraft) == 1,
          "smaller substeps changed route progression");
  near(distance(read(turning, aircraft).position_ecef_m,
                read(reference, reference_aircraft).position_ecef_m), 0, 0.001,
       "guided turn did not converge within the fixture position tolerance");

  profile.waypoint_capture_radius_m = 0.25;
  limits.max_transitions = 1;
  StateStore store(1, 1);
  const auto entity = spawn(store, "route-rollback");
  MotionDriver driver(store, 0, profile, limits,
      {{entity, false, {east(0), east(10), east(20)}, {}}});
  driver.advance_to(0);
  require(driver.waypoint_progress(entity) == 1, "initial inside-sphere waypoint was not captured");
  rejects([&] { driver.advance_to(300000000); }, "transition capacity silently grew");
  require(driver.waypoint_progress(entity) == 1 && driver.time_ns() == 0 &&
          read(store, entity).source_time_ns == 0 && driver.transitions().size() == 1 &&
          driver.transitions()[0].waypoint_index == 0,
          "failed interval consumed route progress or published provisional transitions");
  driver.advance_to(150000000);
  require(driver.waypoint_progress(entity) == 2 && driver.transitions().size() == 1,
          "retry did not resume from the committed waypoint cursor");
}

void interval_wide_step_budget(MotionProfile profile, DriverLimits limits) {
  profile.integration = {10000000, 100000000, 10};
  profile.publication = {7000000, 11000000};
  StateStore store(1, 1);
  const auto entity = spawn(store, "step-budget");
  MotionDriver driver(store, 0, profile, limits,
      {{entity, true, {}, {{5000000, 1, {0, 100, 1000}}}}});
  driver.advance_to(0);
  rejects([&] { driver.advance_to(100000000); }, "event splits reset numerical work budget");
  require(driver.time_ns() == 0 && read(store, entity).source_time_ns == 0,
          "exhausted numerical work committed a partial interval");
}

void admission_and_lifecycle(const MotionProfile& profile, DriverLimits limits) {
  StateStore store(1, 1);
  const auto entity = spawn(store, "admission");
  rejects([&] { MotionDriver driver(store, 0, profile, limits,
      {{entity, false, {east(10)}, {{second, 1, {0, 100, 1000}}}}}); }, "mixed guidance silently overrode commands");
  rejects([&] { MotionDriver driver(store, 0, profile, limits,
      {{entity, false, {}, {{second, 1, {0, 100, 1000}}, {second, 1, {1, 100, 1000}}}}}); },
      "conflicting command identity accepted");
  rejects([&] { MotionDriver driver(store, 0, profile, limits,
      {{entity, false, {}, {{second, 1, {0, 1000, 1000}}}}}); }, "out-of-envelope command admitted");
  rejects([&] { MotionDriver driver(store, 0, profile, limits,
      {{entity, false, {}, {}}, {entity, false, {}, {}}}); }, "duplicate entity programs admitted");
  MotionDriver driver(store, 0, profile, limits, {{entity, false, {}, {}}});
  store.erase(entity);
  spawn(store, "replacement");
  rejects([&] { driver.advance_to(second); }, "driver accepted changed cohort generations");
  require(driver.time_ns() == 0, "failed lifecycle admission advanced driver");

  StateStore growing(2, 1);
  const auto first = spawn(growing, "first");
  MotionDriver membership(growing, 0, profile, limits, {{first, false, {}, {}}});
  spawn(growing, "new-member");
  rejects([&] { membership.advance_to(second); }, "driver silently omitted a new cohort member");
}

void zero_time_cascade_and_failed_initialization(const MotionProfile& profile, DriverLimits limits) {
  StateStore store(1, 1);
  const auto entity = spawn(store, "coincident-captures");
  MotionDriver driver(store, 0, profile, limits,
      {{entity, false, {east(0), east(0), east(0)}, {}}});
  require(driver.waypoint_progress(entity) == 0 && driver.samples().empty() && driver.transitions().empty(),
          "constructor published provisional initial events");
  driver.advance_to(0);
  require(driver.waypoint_progress(entity) == 3 && driver.transitions().size() == 3,
          "coincident initial captures did not drain in one boundary");
  for (const auto& event : driver.transitions())
    require(event.time_ns == 0, "initial capture time advanced");

  StateStore failing(1, 1);
  const auto other = spawn(failing, "failed-initial");
  limits.max_transitions = 1;
  MotionDriver bounded(failing, 0, profile, limits,
      {{other, false, {east(0), east(0)}, {}}});
  rejects([&] { bounded.advance_to(0); }, "initial transition budget was not enforced");
  require(bounded.waypoint_progress(other) == 0 && bounded.samples().empty() &&
          bounded.transitions().empty(), "failed initial boundary leaked cursor or outputs");
}

void invalid_initial_state_and_warmed_work(const MotionProfile& profile, DriverLimits limits) {
  for (int defect = 0; defect < 3; ++defect) {
    StateStore invalid(1, 1);
    auto state = initial();
    if (defect == 0) state.body_frd_to_ecef[0] = 0.5;
    if (defect == 1) state.velocity_ecef_mps = {0, 0, 0};
    if (defect == 2) state.position_ecef_m = geo::geodetic_to_ecef({0, 0, 20000});
    const auto entity = invalid.spawn("invalid-physical", 0, EntityMode::moving,
                                     state, {pi / 2, 100, 1000});
    MotionDriver driver(invalid, 0, profile, limits, {{entity, false, {}, {}}});
    rejects([&] { driver.advance_to(0); }, "invalid physical state normalized into an initial sample");
    require(driver.samples().empty(), "invalid initial state published output");
  }

  StateStore store(1, 1), legacy(1, 1);
  const auto entity = spawn(store, "warmed-driver");
  spawn(legacy, "warmed-legacy");
  StateBatch scratch(1);
  MotionDriver driver(store, 0, profile, limits, {{entity, true, {}, {}}});
  driver.advance_to(0);
  advance_motion(legacy, scratch, 0, profile, 10000000);
  // Buffers are reserved at admission. Event processing and the existing scalar
  // entry point must reuse them rather than allocating a counter vector per call.
  {
    NoAllocation guard;
    driver.advance_to(100000000);
    driver.advance_to(200000000);
    advance_motion(legacy, scratch, 0, profile, 20000000);
  }
}

void partition_agreement(const MotionProfile& profile, DriverLimits limits) {
  StateStore whole(1, 1), partitioned(1, 1);
  const auto a = spawn(whole, "partition");
  const auto b = spawn(partitioned, "partition");
  MotionDriver first(whole, 0, profile, limits,
      {{a, true, {}, {{300000000, 1, {0, 105, 1010}}}}});
  MotionDriver second_driver(partitioned, 0, profile, limits,
      {{b, true, {}, {{300000000, 1, {0, 105, 1010}}}}});
  first.advance_to(second);
  std::vector<std::int64_t> times;
  for (const auto deadline : {137000000LL, 499000000LL, 1000000000LL}) {
    second_driver.advance_to(deadline);
    for (const auto& sample : second_driver.samples()) times.push_back(sample.state.source_time_ns);
  }
  require(times.size() == first.samples().size(), "partitioning lost or duplicated deadlines");
  for (std::size_t index = 0; index < times.size(); ++index)
    require(times[index] == first.samples()[index].state.source_time_ns, "partitioning shifted a deadline");
  near(distance(read(whole, a).position_ecef_m, read(partitioned, b).position_ecef_m), 0, 0.001,
       "interval partitioning changed motion beyond fixture tolerance");
}

template <typename Function>
bool run_case(const char* name, Function test, const MotionProfile& profile, DriverLimits limits) {
  try { test(profile, limits); return true; }
  catch (const std::exception& error) {
    std::cerr << name << ": " << error.what() << '\n';
    return false;
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "expected repository root");
    const std::string root = argv[1];
    const auto profile = load_motion_profile(root + "/config/profiles/synthetic_fixed_wing.json",
                                             root + "/config/motion_profile.schema.json");
    const auto limits = limits_from(root);
    int failures = 0;
    failures += !run_case("cadences and command boundary", cadences_and_coincident_command, profile, limits);
    failures += !run_case("thirty-second command", exact_thirty_second_turn, profile, limits);
    failures += !run_case("swept waypoints", swept_waypoints, profile, limits);
    failures += !run_case("atomic failure and retry", atomic_failure_and_retry, profile, limits);
    failures += !run_case("guidance turn and progress rollback", guidance_turn_and_progress_rollback, profile, limits);
    failures += !run_case("whole-interval step budget", interval_wide_step_budget, profile, limits);
    failures += !run_case("admission and lifecycle", admission_and_lifecycle, profile, limits);
    failures += !run_case("interval partition", partition_agreement, profile, limits);
    failures += !run_case("zero-time initialization", zero_time_cascade_and_failed_initialization, profile, limits);
    failures += !run_case("initial validation and warmed work", invalid_initial_state_and_warmed_work, profile, limits);
    if (failures != 0) return 1;
    std::cout << "motion driver tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "motion driver failed: " << error.what() << '\n';
    return 1;
  }
}
