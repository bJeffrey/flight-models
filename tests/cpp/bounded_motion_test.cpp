#include "flight_models/bounded_motion.h"

#include "math/coordinates/geodetic.hpp"
#include "math/numerical/angle_wrapping.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace flight_models;
namespace geo = math::coordinates;
constexpr double pi = 3.14159265358979323846;
constexpr std::int64_t second = 1000000000;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const char* message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

template <typename Function>
void rejects(Function operation, const char* message) {
  try {
    operation();
  } catch (const std::invalid_argument&) {
    return;
  }
  throw std::runtime_error(message);
}

/** Construct coherent no-wind equatorial fixtures, with FRD axes in ECEF. */
PhysicalState initial(double heading = pi / 2, double speed = 100, double climb_rate = 0,
                      const geo::GeodeticPosition& location = {0, 0, 1000}) {
  PhysicalState state;
  const auto rotation = geo::ned_to_ecef_rotation(location);
  const double c = std::cos(heading);
  const double s = std::sin(heading);
  const double pitch = std::atan2(climb_rate, speed);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const auto forward = geo::rotate(rotation, {c * cp, s * cp, -sp});
  const auto right = geo::rotate(rotation, {-s, c, 0});
  const auto down = geo::rotate(rotation, {c * sp, s * sp, cp});
  state.position_ecef_m = geo::geodetic_to_ecef(location);
  state.velocity_ecef_mps = geo::rotate(rotation, {speed * c, speed * s, -climb_rate});
  state.body_frd_to_ecef = {forward.x, right.x, down.x,
                          forward.y, right.y, down.y,
                          forward.z, right.z, down.z};
  return state;
}

PhysicalState read(const StateStore& store, Handle handle) {
  PhysicalState state;
  DesiredMotion desired;
  require(store.read_state(handle, state, desired), "test handle disappeared");
  return state;
}

double heading(const PhysicalState& state) {
  const auto frame = geo::transpose(geo::ned_to_ecef_rotation(geo::ecef_to_geodetic(state.position_ecef_m)));
  const auto forward = geo::rotate(frame, {state.body_frd_to_ecef[0],
                                         state.body_frd_to_ecef[3],
                                         state.body_frd_to_ecef[6]});
  return math::numerical::wrap_radians_positive(std::atan2(forward.y, forward.x));
}

double speed(const PhysicalState& state) {
  const auto frame = geo::transpose(geo::ned_to_ecef_rotation(geo::ecef_to_geodetic(state.position_ecef_m)));
  const auto velocity = geo::rotate(frame, state.velocity_ecef_mps);
  return std::hypot(velocity.x, velocity.y);
}

double distance(math::Vector3 a, math::Vector3 b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

template <typename Function>
void run_case(const char* name, Function test, const MotionProfile& profile) {
  try {
    test(profile);
  } catch (const std::exception& error) {
    throw std::runtime_error(std::string(name) + ": " + error.what());
  }
}

void straight_and_modes(const MotionProfile& profile) {
  StateStore store(4, 2);
  StateBatch scratch(4);
  const DesiredMotion desired{pi / 2, 100, 1000};
  const auto moving = store.spawn("moving", 0, EntityMode::moving, initial(), desired);
  const auto fixed = store.spawn("fixed", 0, EntityMode::fixed, initial(), desired);
  const auto disabled = store.spawn("disabled", 0, EntityMode::disabled, initial(), desired);
  const auto other = store.spawn("other-profile", 1, EntityMode::moving, initial(), desired);
  advance_motion(store, scratch, 0, profile, 2 * second);

  const auto state = read(store, moving);
  const auto location = geo::ecef_to_geodetic(state.position_ecef_m);
  // Eastward equatorial travel has an exact constant-radius solution on WGS-84.
  const auto expected = geo::geodetic_to_ecef({0, 200 / 6379137.0, 1000});
  near(distance(state.position_ecef_m, expected), 0, 1e-5, "equatorial straight travel differs from analytical arc");
  near(location.altitude_m, 1000, 1e-6, "constant-altitude motion drifted");
  near(speed(state), 100, 1e-9, "constant speed drifted");
  // Constant HAE eastward motion still has ECEF centripetal acceleration and
  // body rotation as the local tangent frame travels around the ellipsoid.
  near(state.acceleration_ecef_mps2.x, -10000 / 6379137.0, 1e-9,
       "endpoint acceleration omitted Earth curvature");
  near(state.angular_velocity_body_radps.y, -100 / 6379137.0, 1e-10,
       "body angular velocity omitted tangent-frame transport");
  require(state.source_time_ns == 2 * second, "motion source time rounded");
  for (const auto handle : {fixed, disabled, other}) {
    require(read(store, handle).source_time_ns == 0, "motion changed an excluded cohort");
  }
}

void bounded_controls(const MotionProfile& profile) {
  StateStore store(2, 1);
  StateBatch scratch(2);
  const auto accelerating = store.spawn("accelerating", 0, EntityMode::moving, initial(), {pi / 2, 105, 1015});
  const auto decelerating = store.spawn("decelerating", 0, EntityMode::moving, initial(),
                                      {pi / 2, profile.flight_envelope.min_speed_mps, 970});
  advance_motion(store, scratch, 0, profile, 2 * second);
  const auto up = read(store, accelerating);
  const auto down = read(store, decelerating);
  near(speed(up), 105, 1e-8, "acceleration overshot or missed target");
  near(speed(down), profile.flight_envelope.min_speed_mps, 1e-8, "deceleration overshot minimum");
  near(geo::ecef_to_geodetic(up.position_ecef_m).altitude_m, 1015, 1e-6, "climb missed capture");
  near(geo::ecef_to_geodetic(down.position_ecef_m).altitude_m, 970, 1e-6, "descent missed capture");
  // Integral of speed: 1.25 seconds accelerating at 4 m/s^2, then 0.75 at 105.
  near(geo::ecef_to_geodetic(up.position_ecef_m).longitude_rad * 6379137.0,
       206.875, 0.001, "speed target capture corrupted traveled distance");

  advance_motion(store, scratch, 0, profile, 4 * second);
  near(speed(read(store, accelerating)), 105, 1e-8, "captured speed did not hold");
}

void heading_wrap_and_turn(const MotionProfile& profile) {
  StateStore store(3, 1);
  StateBatch scratch(3);
  const auto wrapped = store.spawn("wrap", 0, EntityMode::moving,
                                  initial(2 * pi - 0.01), {0.01, 100, 1000});
  const auto half = store.spawn("half", 0, EntityMode::moving, initial(0), {pi, 100, 1000});
  const auto turn = store.spawn("turn", 0, EntityMode::moving, initial(0), {pi / 2, 100, 1000});
  advance_motion(store, scratch, 0, profile, 2 * second);
  near(heading(read(store, wrapped)), 0.01, 1e-10, "heading wrap did not take short path");
  near(heading(read(store, half)), 2 * profile.flight_envelope.max_turn_rate_radps,
       1e-10, "half-turn tie was not clockwise");

  const auto state = read(store, turn);
  const auto enu = geo::geodetic_to_enu(geo::ecef_to_geodetic(state.position_ecef_m), {0, 0, 1000});
  const double rate = profile.flight_envelope.max_turn_rate_radps;
  // Local constant-rate turn is circular. Over 200 m, ellipsoid/frame curvature
  // changes the tangent-plane solution by less than one centimetre.
  near(enu.x, 100 / rate * (1 - std::cos(2 * rate)), 0.01, "turn east displacement incorrect");
  near(enu.y, 100 / rate * std::sin(2 * rate), 0.01, "turn north displacement incorrect");
}

PhysicalState trajectory(MotionProfile profile, std::int64_t substep_ns, std::int64_t interval_ns) {
  profile.integration.max_substep_ns = substep_ns;
  StateStore store(1, 1);
  StateBatch scratch(1);
  const auto entity = store.spawn("convergence", 0, EntityMode::moving, initial(0), {pi / 2, 180, 1400});
  for (std::int64_t time = interval_ns; time <= 10 * second; time += interval_ns) {
    advance_motion(store, scratch, 0, profile, time);
  }
  return read(store, entity);
}

void substep_and_partition_agreement(const MotionProfile& profile) {
  const auto coarse = trajectory(profile, 10000000, second);
  const auto fine = trajectory(profile, 1000000, second);
  const auto partitioned = trajectory(profile, 10000000, second / 4);
  near(distance(coarse.position_ecef_m, fine.position_ecef_m), 0, 0.001,
       "10ms and 1ms midpoint trajectories differ by more than 1mm at 10s");
  near(distance(coarse.position_ecef_m, partitioned.position_ecef_m), 0, 1e-5,
       "aligned interval partition changed trajectory");
  near(speed(coarse), speed(fine), 1e-8, "substeps changed final speed");
  near(heading(coarse), heading(fine), 1e-9, "substeps changed final heading");
}

void failure_is_atomic(MotionProfile profile) {
  StateStore empty(1, 1);
  StateBatch empty_scratch(1);
  rejects([&] { advance_motion(empty, empty_scratch, 0, profile, -1); },
          "negative endpoint accepted for an empty cohort");

  StateStore store(2, 1);
  StateBatch scratch(2);
  const auto valid = store.spawn("a-valid", 0, EntityMode::moving, initial(), {pi / 2, 100, 1000});
  store.spawn("z-invalid", 0, EntityMode::moving, initial(), {pi / 2, 1000, 1000});
  rejects([&] { advance_motion(store, scratch, 0, profile, second); }, "out-of-envelope command accepted");
  require(read(store, valid).source_time_ns == 0, "failed cohort partially committed");

  StateStore single(1, 1);
  const auto entity = single.spawn("valid", 0, EntityMode::moving, initial(), {pi / 2, 100, 1000});
  rejects([&] { advance_motion(single, scratch, 0, profile, 3 * second); }, "oversize interval accepted");
  rejects([&] { advance_motion(single, scratch, 0, profile, -1); }, "negative time accepted");
  profile.integration.max_substeps_per_interval = 1;
  rejects([&] { advance_motion(single, scratch, 0, profile, second); }, "exhausted work budget accepted");
  require(read(single, entity).source_time_ns == 0, "failed work budget changed live state");
}

void initial_state_and_boundaries(const MotionProfile& profile) {
  StateBatch scratch(1);
  StateStore invalid(1, 1);
  auto state = initial();
  // A finite forward axis alone is insufficient: all three axes must form a
  // proper rotation before a kernel can interpret body-frame orientation.
  state.body_frd_to_ecef[1] = state.body_frd_to_ecef[0];
  state.body_frd_to_ecef[4] = state.body_frd_to_ecef[3];
  state.body_frd_to_ecef[7] = state.body_frd_to_ecef[6];
  invalid.spawn("bad-attitude", 0, EntityMode::moving, state, {pi / 2, 100, 1000});
  rejects([&] { advance_motion(invalid, scratch, 0, profile, second); }, "singular attitude accepted");
  invalid.clear();
  state = initial();
  state.body_frd_to_ecef = initial(pi / 2, 100, 20).body_frd_to_ecef;
  invalid.spawn("incoherent-pitch", 0, EntityMode::moving, state, {pi / 2, 100, 1000});
  rejects([&] { advance_motion(invalid, scratch, 0, profile, second); },
          "initial pitch inconsistent with velocity accepted");

  StateStore boundary(1, 1);
  state = initial(pi / 2, profile.flight_envelope.min_speed_mps);
  state.position_ecef_m = geo::geodetic_to_ecef({0, 0, profile.flight_envelope.min_altitude_hae_m});
  const auto entity = boundary.spawn("boundary", 0, EntityMode::moving, state,
                                     {pi / 2, profile.flight_envelope.min_speed_mps,
                                      profile.flight_envelope.min_altitude_hae_m});
  for (std::int64_t time = second; time <= 4 * second; time += second) {
    advance_motion(boundary, scratch, 0, profile, time);
  }
  near(speed(read(boundary, entity)), profile.flight_envelope.min_speed_mps, 1e-8,
       "roundoff at minimum speed prevented continued advancement");

  const auto before = read(boundary, entity);
  advance_motion(boundary, scratch, 0, profile, 4 * second);
  const auto after = read(boundary, entity);
  require(distance(before.position_ecef_m, after.position_ecef_m) == 0 &&
          distance(before.velocity_ecef_mps, after.velocity_ecef_mps) == 0 &&
          distance(before.acceleration_ecef_mps2, after.acceleration_ecef_mps2) == 0 &&
          before.body_frd_to_ecef == after.body_frd_to_ecef,
          "zero-duration call changed state");
}

void climb_preserves_horizontal_speed(const MotionProfile& profile) {
  StateStore store(1, 1);
  StateBatch scratch(1);
  const auto entity = store.spawn("climbing", 0, EntityMode::moving, initial(), {pi / 2, 100, 1400});
  advance_motion(store, scratch, 0, profile, second);
  const auto state = read(store, entity);
  const auto local = geo::rotate(geo::transpose(geo::ned_to_ecef_rotation(
      geo::ecef_to_geodetic(state.position_ecef_m))), state.velocity_ecef_mps);
  near(speed(state), 100, 1e-8, "climb treated horizontal speed as total speed");
  near(-local.z, profile.flight_envelope.max_climb_rate_mps, 1e-8,
       "climb rate differs from HAE rate");
  near(geo::ecef_to_geodetic(state.position_ecef_m).altitude_m, 1030, 1e-6,
       "altitude did not integrate configured climb rate");
}

void work_budget_and_fractional_capture(MotionProfile profile) {
  StateStore store(1, 1);
  StateBatch scratch(1);
  auto entity = store.spawn("straight", 0, EntityMode::moving, initial(), {pi / 2, 100, 1000});
  profile.integration.max_substeps_per_interval = 200;
  advance_motion(store, scratch, 0, profile, 2 * second);
  require(read(store, entity).source_time_ns == 2 * second,
          "roundoff consumed an extra substep in an exact work budget");

  store.clear();
  profile.integration.max_substeps_per_interval = 4096;
  entity = store.spawn("fractional-captures", 0, EntityMode::moving, initial(0),
                       {0.0137, 105.012345, 1015.012345});
  advance_motion(store, scratch, 0, profile, 2 * second);
  const auto state = read(store, entity);
  near(heading(state), 0.0137, 1e-10, "non-grid heading capture failed");
  near(speed(state), 105.012345, 1e-8, "non-grid speed capture failed");
  near(geo::ecef_to_geodetic(state.position_ecef_m).altitude_m, 1015.012345, 1e-6,
       "non-grid altitude capture failed");
}

void asymmetric_vertical_limits(MotionProfile profile) {
  profile.flight_envelope.max_descent_rate_mps = 10;
  StateStore store(1, 1);
  StateBatch scratch(1);
  store.spawn("excess-descent", 0, EntityMode::moving, initial(pi / 2, 100, -20), {pi / 2, 100, 950});
  rejects([&] { advance_motion(store, scratch, 0, profile, second); },
          "descent was validated against the climb limit");
  store.clear();
  const auto entity = store.spawn("permitted-descent", 0, EntityMode::moving,
                                  initial(pi / 2, 100, -10), {pi / 2, 100, 950});
  advance_motion(store, scratch, 0, profile, second);
  near(geo::ecef_to_geodetic(read(store, entity).position_ecef_m).altitude_m, 990, 1e-6,
       "asymmetric descent rate was not honored");
}

/** Exercise geometry reuse off the equator and across the longitude seam. */
void latitude_and_longitude_boundaries(const MotionProfile& profile) {
  for (const double latitude : {-85 * pi / 180, 0.0, 85 * pi / 180}) {
    for (const double altitude : {profile.flight_envelope.min_altitude_hae_m,
                                  profile.flight_envelope.max_altitude_hae_m}) {
      const geo::GeodeticPosition origin{latitude, pi - 1e-6, altitude};
      StateStore store(1, 1);
      StateBatch scratch(1);
      const auto entity = store.spawn("dateline", 0, EntityMode::moving,
                                      initial(pi / 2, 100, 0, origin), {pi / 2, 100, altitude});
      for (std::int64_t time = second; time <= 4 * second; time += second) {
        advance_motion(store, scratch, 0, profile, time);
      }
      const auto state = read(store, entity);
      const auto location = geo::ecef_to_geodetic(state.position_ecef_m);
      require(location.longitude_rad < 0, "eastward travel did not cross longitude wrap");
      near(location.altitude_m, altitude, 1e-6, "normal correction changed boundary altitude");
      near(speed(state), 100, 1e-8, "latitude changed horizontal speed");
      near(heading(state), pi / 2, 1e-10, "longitude wrap changed true heading");
    }
  }
}

/** Compare analytic endpoint derivatives with neighboring trajectory samples. */
void derivative_consistency(MotionProfile profile) {
  profile.integration.max_substep_ns = 1000000;
  auto sample = [&](std::int64_t time_ns) {
    StateStore store(1, 1);
    StateBatch scratch(1);
    const auto entity = store.spawn("derivative", 0, EntityMode::moving,
        initial(0.4, 110, 0, {0.7, -1.2, 1500}), {2.0, 180, 1900});
    advance_motion(store, scratch, 0, profile, time_ns);
    return read(store, entity);
  };
  const auto before = sample(second - 1000000);
  const auto at = sample(second);
  const auto after = sample(second + 1000000);
  const math::Vector3 velocity_difference{
      (after.position_ecef_m.x - before.position_ecef_m.x) / 0.002,
      (after.position_ecef_m.y - before.position_ecef_m.y) / 0.002,
      (after.position_ecef_m.z - before.position_ecef_m.z) / 0.002};
  const math::Vector3 acceleration_difference{
      (after.velocity_ecef_mps.x - before.velocity_ecef_mps.x) / 0.002,
      (after.velocity_ecef_mps.y - before.velocity_ecef_mps.y) / 0.002,
      (after.velocity_ecef_mps.z - before.velocity_ecef_mps.z) / 0.002};
  near(distance(velocity_difference, at.velocity_ecef_mps), 0, 0.0001,
       "position derivative disagrees with endpoint velocity");
  near(distance(acceleration_difference, at.acceleration_ecef_mps2), 0, 0.00001,
       "velocity derivative disagrees with endpoint acceleration");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "expected repository root");
    const std::string root = argv[1];
    const auto profile = load_motion_profile(root + "/config/profiles/synthetic_fixed_wing.json",
                                             root + "/config/motion_profile.schema.json");
    run_case("straight and modes", straight_and_modes, profile);
    run_case("bounded controls", bounded_controls, profile);
    run_case("heading wrap and turn", heading_wrap_and_turn, profile);
    run_case("substeps and partition", substep_and_partition_agreement, profile);
    run_case("atomic failure", failure_is_atomic, profile);
    run_case("initial state and boundaries", initial_state_and_boundaries, profile);
    run_case("horizontal speed in climb", climb_preserves_horizontal_speed, profile);
    run_case("work budget and fractional capture", work_budget_and_fractional_capture, profile);
    run_case("asymmetric vertical limits", asymmetric_vertical_limits, profile);
    run_case("latitude and longitude boundaries", latitude_and_longitude_boundaries, profile);
    run_case("endpoint derivative consistency", derivative_consistency, profile);
    std::cout << "bounded motion tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "bounded motion failed: " << error.what() << '\n';
    return 1;
  }
}
