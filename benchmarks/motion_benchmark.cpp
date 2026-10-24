/** @file Deterministic throughput comparison for transactional motion updates.
 * Timed regions include command gather/commit and motion gather/integration/commit;
 * entity creation and CSV output are outside the timer. Wall durations are ns.
 */
#include "flight_models/bounded_motion.h"
#include "flight_models/motion_profile.h"
#include "math/coordinates/geodetic.hpp"

#include <nlohmann/json.hpp>
#include <valijson/adapters/nlohmann_json_adapter.hpp>
#include <valijson/schema.hpp>
#include <valijson/schema_parser.hpp>
#include <valijson/validator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

/** Require JSON integer tokens so durations and counts never pass through floating point. */
void require_integer_token(const Json& object, const char* name) {
  const auto& value = object.at(name);
  if (!value.is_number_integer() && !value.is_number_unsigned())
    throw std::invalid_argument(std::string("workload field must be an integer token: ") + name);
}

struct Workload {
  std::vector<std::size_t> counts, intervals_per_case;
  std::int64_t interval_ns, command_period_ns;
  std::size_t warmups, repeats;
  double latitude_rad, longitude_rad, latitude_step_rad, longitude_step_rad;
  double initial_heading_rad, heading_step_rad, initial_speed_mps, initial_altitude_m;
  double heading_a_rad, heading_b_rad, speed_a_mps, speed_b_mps, altitude_a_m, altitude_b_m;
};

/** Load schema-validated workload values and reject arithmetic overflow before timing. */
Workload read_workload(const std::string& config_path, const std::string& schema_path) {
  std::ifstream config_file(config_path), schema_file(schema_path);
  if (!config_file || !schema_file) throw std::runtime_error("cannot open workload config or schema");
  const Json cfg = Json::parse(config_file), schema_doc = Json::parse(schema_file);
  valijson::Schema schema;
  valijson::SchemaParser parser;
  valijson::adapters::NlohmannJsonAdapter schema_adapter(schema_doc);
  parser.populateSchema(schema_adapter, schema);
  valijson::Validator validator;
  valijson::adapters::NlohmannJsonAdapter config_adapter(cfg);
  valijson::ValidationResults errors;
  if (!validator.validate(schema, config_adapter, &errors))
    throw std::invalid_argument("workload config violates schema: " + config_path);
  for (const char* key : {"interval_ns", "warmup_runs", "repeat_runs", "command_period_ns"})
    require_integer_token(cfg, key);
  for (const auto& count : cfg.at("intervals_per_case")) {
    if (!count.is_number_integer() && !count.is_number_unsigned())
      throw std::invalid_argument("intervals_per_case entries must be integer tokens");
  }
  for (const auto& count : cfg.at("entity_counts")) {
    if (!count.is_number_integer() && !count.is_number_unsigned())
      throw std::invalid_argument("entity_counts entries must be integer tokens");
  }

  Workload w{cfg.at("entity_counts").get<std::vector<std::size_t>>(),
             cfg.at("intervals_per_case").get<std::vector<std::size_t>>(),
             cfg.at("interval_ns").get<std::int64_t>(), cfg.at("command_period_ns").get<std::int64_t>(),
             cfg.at("warmup_runs").get<std::size_t>(),
             cfg.at("repeat_runs").get<std::size_t>(), cfg.at("latitude_rad").get<double>(),
             cfg.at("longitude_rad").get<double>(), cfg.at("latitude_step_rad").get<double>(),
             cfg.at("longitude_step_rad").get<double>(), cfg.at("initial_heading_rad").get<double>(),
             cfg.at("heading_step_rad").get<double>(), cfg.at("initial_speed_mps").get<double>(),
             cfg.at("initial_altitude_m").get<double>(),
             cfg.at("heading_a_rad").get<double>(), cfg.at("heading_b_rad").get<double>(),
             cfg.at("speed_a_mps").get<double>(), cfg.at("speed_b_mps").get<double>(),
             cfg.at("altitude_a_m").get<double>(), cfg.at("altitude_b_m").get<double>()};
  if (w.counts.empty() || w.counts.size() != w.intervals_per_case.size() ||
      w.interval_ns <= 0 || w.command_period_ns <= 0)
    throw std::invalid_argument("invalid workload duration or empty entity counts");
  for (std::size_t i = 0; i < w.counts.size(); ++i) {
    if (w.counts[i] == 0 || (i != 0 && w.counts[i] <= w.counts[i - 1]))
      throw std::invalid_argument("entity_counts must be positive and strictly increasing");
  }
  return w;
}

/** Create coherent initial ECEF position, velocity, and FRD attitude for one row. */
flight_models::PhysicalState initial_state(const Workload& w, std::size_t row) {
  namespace geo = math::coordinates;
  const geo::GeodeticPosition location{w.latitude_rad + row * w.latitude_step_rad,
                                        w.longitude_rad + row * w.longitude_step_rad, w.initial_altitude_m};
  const auto frame = geo::ned_to_ecef_rotation(location);
  const double heading = w.initial_heading_rad + row * w.heading_step_rad;
  const auto forward = geo::rotate(frame, {std::cos(heading), std::sin(heading), 0.0});
  const auto right = geo::rotate(frame, {-std::sin(heading), std::cos(heading), 0.0});
  const auto down = geo::rotate(frame, {0.0, 0.0, 1.0});
  flight_models::PhysicalState state{};
  state.position_ecef_m = geo::geodetic_to_ecef(location);
  state.velocity_ecef_mps = geo::rotate(frame, {w.initial_speed_mps * std::cos(heading),
                                                 w.initial_speed_mps * std::sin(heading), 0.0});
  state.body_frd_to_ecef = {forward.x, right.x, down.x, forward.y, right.y, down.y,
                            forward.z, right.z, down.z};
  return state;
}

/** Run warmups and timed repeats on one store so simulation and command phase continue. */
void run_case(const Workload& w, const flight_models::MotionProfile& profile,
              std::size_t count, std::size_t intervals, std::size_t case_index, std::ofstream& states) {
  using namespace flight_models;
  StateStore store(count, 1);
  StateBatch scratch(count), commands(count);
  std::vector<Handle> handles;
  handles.reserve(count);
  for (std::size_t row = 0; row < count; ++row) {
    const double heading = w.initial_heading_rad + row * w.heading_step_rad;
    const DesiredMotion desired{heading, w.initial_speed_mps, w.initial_altitude_m};
    handles.push_back(store.spawn("entity-" + std::to_string(row), 0, EntityMode::moving,
                                  initial_state(w, row), desired));
  }
  std::int64_t now = 0;
  auto one_run = [&](bool timed) {
    const auto start = Clock::now();
    for (std::size_t step = 0; step < intervals; ++step) {
      // Apply the initial command at t=0 and later commands at their absolute start times.
      if (now % w.command_period_ns == 0) {
        store.gather(handles.data(), handles.size(), commands);
        auto columns = commands.columns();
        const bool alternate = (now / w.command_period_ns) % 2 != 0;
        for (std::size_t row = 0; row < count; ++row) {
          columns.heading_rad[row] = alternate ? w.heading_b_rad : w.heading_a_rad;
          columns.horizontal_speed_mps[row] = alternate ? w.speed_b_mps : w.speed_a_mps;
          columns.altitude_hae_m[row] = alternate ? w.altitude_b_m : w.altitude_a_m;
        }
        store.commit(commands);
      }
      now += w.interval_ns;
      advance_motion(store, scratch, 0, profile, now);
    }
    return timed ? std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - start).count()
                 : std::int64_t{0};
  };
  for (std::size_t i = 0; i < w.warmups; ++i) one_run(false);
  for (std::size_t i = 0; i < w.repeats; ++i) {
    const auto ns = one_run(true);
    std::cout << "repeat,count=" << count << ",run=" << i << ",wall_ns=" << ns
              << ",intervals=" << intervals << ",entity_updates_per_s=" << std::setprecision(12)
              << (static_cast<double>(count) * static_cast<double>(intervals) * 1.0e9 / ns) << '\n';
  }
  for (std::size_t row = 0; row < count; ++row) {
    PhysicalState state{}; DesiredMotion desired{};
    if (!store.read_state(handles[row], state, desired)) throw std::runtime_error("final state read failed");
    states << case_index << ',' << count << ',' << row << ',' << std::setprecision(17)
           << state.position_ecef_m.x << ',' << state.position_ecef_m.y << ',' << state.position_ecef_m.z << ','
           << state.velocity_ecef_mps.x << ',' << state.velocity_ecef_mps.y << ',' << state.velocity_ecef_mps.z << ','
           << state.acceleration_ecef_mps2.x << ',' << state.acceleration_ecef_mps2.y << ',' << state.acceleration_ecef_mps2.z << ',';
    for (double value : state.body_frd_to_ecef) states << value << ',';
    states << state.angular_velocity_body_radps.x << ',' << state.angular_velocity_body_radps.y << ','
           << state.angular_velocity_body_radps.z << ',' << state.source_time_ns << ',' << desired.heading_rad << ','
           << desired.horizontal_speed_mps << ',' << desired.altitude_hae_m << '\n';
  }
}

/** Check time arithmetic, command cadence, placement, and configured motion envelope. */
void validate_workload(const Workload& workload, const flight_models::MotionProfile& profile) {
  if (workload.interval_ns > profile.integration.max_interval_ns ||
      workload.command_period_ns % workload.interval_ns != 0)
    throw std::invalid_argument("workload interval exceeds profile or does not divide command period");
  const auto max_safe_intervals = static_cast<std::size_t>(
      std::numeric_limits<std::int64_t>::max() / workload.interval_ns);
  for (std::size_t i = 0; i < workload.counts.size(); ++i) {
    const std::size_t rows = workload.counts[i];
    if (workload.intervals_per_case[i] == 0 || workload.intervals_per_case[i] > max_safe_intervals)
      throw std::invalid_argument("workload simulation duration exceeds int64 nanosecond range");
    const double last_row = static_cast<double>(rows - 1);
    const double last_latitude = workload.latitude_rad + last_row * workload.latitude_step_rad;
    const double last_longitude = workload.longitude_rad + last_row * workload.longitude_step_rad;
    if (last_latitude < -1.5707963267948966 || last_latitude > 1.5707963267948966 ||
        last_longitude < -3.141592653589793 || last_longitude > 3.141592653589793)
      throw std::invalid_argument("entity placement increments leave valid geodetic coordinates");
  }
  const auto& limits = profile.flight_envelope;
  const auto within = [](double value, double minimum, double maximum) {
    return value >= minimum && value <= maximum;
  };
  if (!within(workload.initial_speed_mps, limits.min_speed_mps, limits.max_speed_mps) ||
      !within(workload.initial_altitude_m, limits.min_altitude_hae_m, limits.max_altitude_hae_m) ||
      !within(workload.speed_a_mps, limits.min_speed_mps, limits.max_speed_mps) ||
      !within(workload.speed_b_mps, limits.min_speed_mps, limits.max_speed_mps) ||
      !within(workload.altitude_a_m, limits.min_altitude_hae_m, limits.max_altitude_hae_m) ||
      !within(workload.altitude_b_m, limits.min_altitude_hae_m, limits.max_altitude_hae_m))
    throw std::invalid_argument("initial state or commands exceed the loaded profile envelope");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 6) throw std::invalid_argument("usage: motion-benchmark WORKLOAD.json WORKLOAD.schema PROFILE.json PROFILE.schema STATES.csv");
    const Workload workload = read_workload(argv[1], argv[2]);
    const auto profile = flight_models::load_motion_profile(argv[3], argv[4]);
    validate_workload(workload, profile);
    std::ofstream states(argv[5]);
    if (!states) throw std::runtime_error("cannot create state output");
    states << "case,count,row,x_m,y_m,z_m,vx_mps,vy_mps,vz_mps,ax_mps2,ay_mps2,az_mps2";
    for (int i = 0; i < 9; ++i) states << ",dcm_" << i;
    states << ",wx_radps,wy_radps,wz_radps,time_ns,heading_rad,speed_mps,altitude_m\n";
    std::cout << "method=steady_clock ns; timed=command gather+commit and motion gather+integrate+commit; warmups="
              << workload.warmups << ", repeats=" << workload.repeats
              << "; timed repeats continue the same simulation trajectory\n";
    for (std::size_t i = 0; i < workload.counts.size(); ++i)
      run_case(workload, profile, workload.counts[i], workload.intervals_per_case[i], i, states);
  } catch (const std::exception& error) {
    std::cerr << "motion benchmark: " << error.what() << '\n';
    return 2;
  }
}
