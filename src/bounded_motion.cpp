#include "flight_models/bounded_motion.h"

#include "math/core/constants.hpp"
#include "math/coordinates/geodetic.hpp"
#include "math/coordinates/geometry.hpp"
#include "math/numerical/angle_wrapping.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flight_models {
namespace {

using math::Vector3;
using math::coordinates::GeodeticPosition;
using math::coordinates::LocalFrame;

// These bounds cover coordinate/rotation roundoff, not operational model limits.
constexpr double kRotationRoundoffTolerance = 1.0e-10;
constexpr double kHeadingRoundoffToleranceRad = 1.0e-8;
constexpr double kAttitudeCoherenceTolerance = 1.0e-8;
constexpr double kAltitudeRoundoffToleranceM = 1.0e-6;
constexpr double kSpeedRoundoffToleranceMps = 1.0e-8;

/** Check all vector components before geodesy or numeric integration. */
bool finite(const Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

Vector3 scale(Vector3 value, double factor) {
  return {value.x * factor, value.y * factor, value.z * factor};
}

/** Return local NED velocity for horizontal ground speed and independent HAE rate. */
Vector3 velocity_ned(double horizontal_speed, double heading, double hae_rate) {
  return {horizontal_speed * std::cos(heading), horizontal_speed * std::sin(heading), -hae_rate};
}

/** Recover clockwise heading from the projected body forward axis in local NED. */
double heading_from_attitude(const math::Matrix3& body_to_ecef,
                             const LocalFrame& frame) {
  const Vector3 forward{body_to_ecef[0], body_to_ecef[3], body_to_ecef[6]};
  const Vector3 ned = math::coordinates::rotate(
      math::coordinates::transpose(frame.ned_to_ecef), forward);
  const double horizontal = std::hypot(ned.x, ned.y);
  if (!(horizontal > 1.0e-8) || !std::isfinite(horizontal)) {
    throw std::invalid_argument("initial body forward axis has no horizontal projection");
  }
  return math::numerical::wrap_radians_positive(std::atan2(ned.y, ned.x));
}

/** Compose the zero-roll body-to-NED attitude with the local NED-to-ECEF frame. */
math::Matrix3 attitude(const LocalFrame& frame, double heading, double pitch) {
  // Body FRD axes in local NED, then rotate the columns into ECEF.
  const math::Matrix3 local_attitude = math::coordinates::body_frd_to_ned(heading, pitch, 0.0);
  return math::coordinates::compose_rotations(frame.ned_to_ecef, local_attitude);
}

/** Endpoint physical derivatives expressed in ECEF and body FRD frames. */
struct EndpointDerivatives {
  Vector3 acceleration_ecef_mps2;
  Vector3 angular_velocity_body_radps;
};

/** Compute endpoint derivatives from control rates and WGS-84 frame transport. */
EndpointDerivatives endpoint_derivatives(const LocalFrame& frame,
                                         const math::Matrix3& local_attitude,
                                         double heading, double horizontal_speed,
                                         double hae_rate, double speed_rate,
                                         double heading_rate) {
  const Vector3 velocity_local = velocity_ned(horizontal_speed, heading, hae_rate);
  const Vector3 frame_rate_ned = math::coordinates::ned_frame_transport_rate(frame, velocity_local);

  // The vertical command is constant inside a control segment, so only speed
  // control changes pitch; the altitude capture boundary sets its rate to zero.
  const double pitch = std::atan2(hae_rate, horizontal_speed);
  const double pitch_rate = -hae_rate * speed_rate /
      std::max(horizontal_speed * horizontal_speed + hae_rate * hae_rate, 1.0e-12);
  const Vector3 local_acceleration{speed_rate * std::cos(heading) -
                                       horizontal_speed * std::sin(heading) * heading_rate,
                                   speed_rate * std::sin(heading) +
                                       horizontal_speed * std::cos(heading) * heading_rate,
                                   0.0};
  const Vector3 transport_acceleration = math::coordinates::cross(frame_rate_ned, velocity_local);
  const Vector3 acceleration = math::coordinates::rotate(
      frame.ned_to_ecef,
      {local_acceleration.x + transport_acceleration.x,
       local_acceleration.y + transport_acceleration.y,
       local_acceleration.z + transport_acceleration.z});

  const Vector3 transport_body = math::coordinates::rotate(
      math::coordinates::transpose(local_attitude), frame_rate_ned);
  const Vector3 angular_rate{-heading_rate * std::sin(pitch) + transport_body.x,
                             pitch_rate + transport_body.y,
                             heading_rate * std::cos(pitch) + transport_body.z};
  return {acceleration, angular_rate};
}

}  // namespace

void advance_motion_batch(StateBatch& scratch, const MotionProfile& profile,
                          std::int64_t end_time_ns, std::int64_t* cumulative_substeps,
                          std::size_t count_capacity) {
  const auto& limits = profile.flight_envelope;
  const auto& policy = profile.integration;
  if (end_time_ns < 0) throw std::invalid_argument("end time must be nonnegative");
  if (policy.max_substep_ns <= 0 || policy.max_interval_ns <= 0 ||
      policy.max_substeps_per_interval <= 0 ||
      policy.max_substep_ns > policy.max_interval_ns) {
    throw std::invalid_argument("invalid motion integration policy");
  }
  if (!std::isfinite(limits.min_speed_mps) || !std::isfinite(limits.max_speed_mps) ||
      !std::isfinite(limits.max_acceleration_mps2) || !std::isfinite(limits.max_deceleration_mps2) ||
      !std::isfinite(limits.max_turn_rate_radps) || !std::isfinite(limits.max_climb_rate_mps) ||
      !std::isfinite(limits.max_descent_rate_mps) || !std::isfinite(limits.min_altitude_hae_m) ||
      !std::isfinite(limits.max_altitude_hae_m) || limits.min_speed_mps <= 0.0 ||
      limits.max_speed_mps < limits.min_speed_mps || limits.max_acceleration_mps2 <= 0.0 ||
      limits.max_deceleration_mps2 <= 0.0 || limits.max_turn_rate_radps <= 0.0 ||
      limits.max_climb_rate_mps < 0.0 || limits.max_descent_rate_mps < 0.0 ||
      limits.max_altitude_hae_m < limits.min_altitude_hae_m) {
    throw std::invalid_argument("invalid motion envelope");
  }
  const std::size_t count = scratch.size();
  if (cumulative_substeps != nullptr && count > count_capacity) {
    throw std::length_error("substep counter capacity exceeded");
  }
  MutableStateColumns columns = scratch.columns();
  const double pi = math::constants::pi;
  for (std::size_t row = 0; row < count; ++row) {
    if (scratch.modes()[row] != EntityMode::moving) continue;
    const std::int64_t source = columns.source_time_ns[row];
    if (source < 0 || end_time_ns < source || end_time_ns - source > policy.max_interval_ns) {
      throw std::invalid_argument("invalid or overlong motion interval");
    }
  }

  for (std::size_t row = 0; row < count; ++row) {
    if (scratch.modes()[row] != EntityMode::moving) continue;
    const std::int64_t source = columns.source_time_ns[row];
    const double desired_heading = columns.heading_rad[row];
    const double desired_speed = columns.horizontal_speed_mps[row];
    const double desired_altitude = columns.altitude_hae_m[row];
    if (!std::isfinite(desired_heading) || !std::isfinite(desired_speed) ||
        !std::isfinite(desired_altitude) || desired_speed < limits.min_speed_mps ||
        desired_speed > limits.max_speed_mps || desired_altitude < limits.min_altitude_hae_m ||
        desired_altitude > limits.max_altitude_hae_m) {
      throw std::invalid_argument("desired motion is outside the profile envelope");
    }

    Vector3 position{columns.position_ecef_m.x[row], columns.position_ecef_m.y[row],
                     columns.position_ecef_m.z[row]};
    Vector3 velocity{columns.velocity_ecef_mps.x[row], columns.velocity_ecef_mps.y[row],
                     columns.velocity_ecef_mps.z[row]};
    Vector3 initial_acceleration{columns.acceleration_ecef_mps2.x[row], columns.acceleration_ecef_mps2.y[row],
                                 columns.acceleration_ecef_mps2.z[row]};
    if (!finite(position) || !finite(velocity) || !finite(initial_acceleration)) {
      throw std::invalid_argument("non-finite initial state");
    }
    GeodeticPosition location = math::coordinates::ecef_to_geodetic(position);
    LocalFrame frame = math::coordinates::make_local_frame(location);
    if (location.altitude_m < limits.min_altitude_hae_m - kAltitudeRoundoffToleranceM ||
        location.altitude_m > limits.max_altitude_hae_m + kAltitudeRoundoffToleranceM) {
      throw std::invalid_argument("initial altitude is outside the profile envelope");
    }
    math::Matrix3 initial_attitude{};
    std::copy_n(columns.body_frd_to_ecef + 9 * row, 9, initial_attitude.begin());
    if (!math::coordinates::is_proper_rotation(initial_attitude, kRotationRoundoffTolerance)) {
      throw std::invalid_argument("invalid initial attitude DCM");
    }
    double heading = heading_from_attitude(initial_attitude, frame);
    Vector3 ned_velocity = math::coordinates::rotate(
        math::coordinates::transpose(frame.ned_to_ecef), velocity);
    double speed = std::hypot(ned_velocity.x, ned_velocity.y);
    const double vertical_rate = -ned_velocity.z;
    const double velocity_heading = math::numerical::wrap_radians_positive(
        std::atan2(ned_velocity.y, ned_velocity.x));
    if (std::abs(math::numerical::wrap_radians_signed(velocity_heading - heading)) >
        kHeadingRoundoffToleranceRad) {
      throw std::invalid_argument("initial attitude heading disagrees with velocity");
    }
    if (!std::isfinite(speed) || speed < limits.min_speed_mps - kSpeedRoundoffToleranceMps ||
        speed > limits.max_speed_mps + kSpeedRoundoffToleranceMps ||
        vertical_rate > limits.max_climb_rate_mps + kSpeedRoundoffToleranceMps ||
        vertical_rate < -limits.max_descent_rate_mps - kSpeedRoundoffToleranceMps) {
      throw std::invalid_argument("initial velocity is outside the profile envelope");
    }
    const math::Matrix3 modeled_initial_attitude = attitude(
        frame, heading, std::atan2(vertical_rate, speed));
    for (std::size_t element = 0; element < initial_attitude.size(); ++element) {
      if (std::abs(initial_attitude[element] - modeled_initial_attitude[element]) >
          kAttitudeCoherenceTolerance) {
        throw std::invalid_argument("initial attitude disagrees with zero-roll motion model");
      }
    }

    // Only representation roundoff may cross a bound; reject larger deviations
    // above and bring admitted boundary values back into the exact envelope.
    speed = std::clamp(speed, limits.min_speed_mps, limits.max_speed_mps);
    location.altitude_m = std::clamp(location.altitude_m,
                                    limits.min_altitude_hae_m, limits.max_altitude_hae_m);
    frame.origin.altitude_m = location.altitude_m;
    if (end_time_ns == source) continue;

    const double normalized_desired_heading = math::numerical::wrap_radians_positive(desired_heading);
    double remaining_heading = math::numerical::wrap_radians_signed(normalized_desired_heading - heading);
    if (std::abs(std::abs(remaining_heading) - pi) < 1.0e-14) remaining_heading = pi;
    const int direction = remaining_heading < 0.0 ? -1 : 1;
    std::int64_t remaining_ns = end_time_ns - source;
    std::int64_t local_steps = 0;
    std::int64_t& steps = cumulative_substeps == nullptr ? local_steps : cumulative_substeps[row];
    double endpoint_speed_rate = 0.0;
    double endpoint_heading_rate = 0.0;
    double endpoint_vertical_rate = 0.0;
    while (remaining_ns > 0) {
      const std::int64_t outer_step_ns = std::min(remaining_ns, policy.max_substep_ns);
      double outer_step_remaining = static_cast<double>(outer_step_ns) * 1.0e-9;
      while (outer_step_remaining > 0.0) {
        if (steps == policy.max_substeps_per_interval) {
          throw std::invalid_argument("motion substep budget exceeded");
        }
        ++steps;

        // Stop at the earliest control capture as well as the regular step boundary.
        double dt = outer_step_remaining;
        const double speed_delta = desired_speed - speed;
        const double speed_rate = speed_delta >= 0.0 ? limits.max_acceleration_mps2 : limits.max_deceleration_mps2;
        if (std::abs(speed_delta) > 0.0) dt = std::min(dt, std::abs(speed_delta) / speed_rate);
        if (std::abs(remaining_heading) > 0.0) dt = std::min(dt, std::abs(remaining_heading) / limits.max_turn_rate_radps);
        const double altitude_delta = desired_altitude - location.altitude_m;
        const double vertical_limit = altitude_delta >= 0.0 ? limits.max_climb_rate_mps : limits.max_descent_rate_mps;
        if (std::abs(altitude_delta) > 0.0) {
          if (vertical_limit <= 0.0) throw std::invalid_argument("invalid vertical rate limit");
          dt = std::min(dt, std::abs(altitude_delta) / vertical_limit);
        }
        if (!(dt > 0.0) || !std::isfinite(dt)) throw std::invalid_argument("motion integration made no progress");

        // Evolve achieved controls without crossing their admitted targets.
        const double speed_step = speed_rate * dt;
        const double next_speed = std::abs(speed_delta) <= speed_step
            ? desired_speed : speed + std::copysign(speed_step, speed_delta);
        const double heading_step = limits.max_turn_rate_radps * dt;
        const bool captures_heading = std::abs(remaining_heading) <= heading_step;
        const double next_heading = captures_heading
            ? normalized_desired_heading
            : math::numerical::wrap_radians_positive(heading + direction * heading_step);
        const double next_remaining_heading = captures_heading ? 0.0 : remaining_heading - direction * heading_step;
        const double previous_altitude = location.altitude_m;
        const double altitude_step = vertical_limit * dt;
        const double next_altitude = std::abs(altitude_delta) <= altitude_step
            ? desired_altitude : previous_altitude + std::copysign(altitude_step, altitude_delta);
        // Evaluate the ECEF derivative at the predicted midpoint. NED components
        // retain horizontal speed and independent HAE rate throughout the turn.
        const double middle_heading = math::numerical::wrap_radians_positive(
            heading + direction * std::min(std::abs(remaining_heading), limits.max_turn_rate_radps * dt * 0.5));
        const double middle_speed = (speed + next_speed) * 0.5;
        const double middle_vertical = (next_altitude - previous_altitude) / dt;
        const Vector3 midpoint_ned = velocity_ned(middle_speed, middle_heading, middle_vertical);
        const Vector3 midpoint_velocity = math::coordinates::rotate(
            frame.ned_to_ecef, midpoint_ned);
        const Vector3 midpoint_position{position.x + midpoint_velocity.x * dt * 0.5,
                                        position.y + midpoint_velocity.y * dt * 0.5,
                                        position.z + midpoint_velocity.z * dt * 0.5};
        const GeodeticPosition midpoint_geodetic = math::coordinates::ecef_to_geodetic(midpoint_position);
        const Vector3 step_displacement = scale(midpoint_ned, dt);
        const Vector3 displacement_ecef = math::coordinates::rotate(
            math::coordinates::ned_to_ecef_rotation(midpoint_geodetic), step_displacement);
        position = {position.x + displacement_ecef.x, position.y + displacement_ecef.y,
                    position.z + displacement_ecef.z};
        // The independent altitude control is exact; remove integration drift
        // normal to the ellipsoid before constructing the next local frame.
        location = math::coordinates::ecef_to_geodetic(position);
        frame = math::coordinates::make_local_frame(location);
        position = math::coordinates::replace_altitude_ecef(position, frame, next_altitude);
        location.altitude_m = next_altitude;
        frame.origin.altitude_m = next_altitude;
        const double vertical_rate_endpoint = next_altitude == desired_altitude
            ? 0.0 : std::copysign(vertical_limit, desired_altitude - next_altitude);
        endpoint_speed_rate = next_speed == desired_speed ? 0.0 : std::copysign(speed_rate, desired_speed - next_speed);
        endpoint_heading_rate = next_remaining_heading == 0.0 ? 0.0
            : direction * limits.max_turn_rate_radps;
        endpoint_vertical_rate = vertical_rate_endpoint;
        remaining_heading = next_remaining_heading;
        heading = next_heading;
        speed = next_speed;
        // Integer-nanosecond outer boundaries keep regular max-substep work counts
        // exact; floating-point event splits are contained within each boundary.
        outer_step_remaining = dt == outer_step_remaining
            ? 0.0 : std::max(0.0, outer_step_remaining - dt);
      }
      remaining_ns -= outer_step_ns;
    }

    // Publish pose and derivatives only after every control reaches its endpoint.
    const double final_pitch = std::atan2(endpoint_vertical_rate, speed);
    const auto local_attitude = math::coordinates::body_frd_to_ned(heading, final_pitch, 0.0);
    const math::Matrix3 final_attitude = math::coordinates::compose_rotations(frame.ned_to_ecef, local_attitude);
    velocity = math::coordinates::rotate(frame.ned_to_ecef,
        velocity_ned(speed, heading, endpoint_vertical_rate));
    const EndpointDerivatives derivatives = endpoint_derivatives(
        frame, local_attitude, heading, speed, endpoint_vertical_rate, endpoint_speed_rate,
        endpoint_heading_rate);
    columns.position_ecef_m.x[row] = position.x;
    columns.position_ecef_m.y[row] = position.y;
    columns.position_ecef_m.z[row] = position.z;
    columns.velocity_ecef_mps.x[row] = velocity.x;
    columns.velocity_ecef_mps.y[row] = velocity.y;
    columns.velocity_ecef_mps.z[row] = velocity.z;
    columns.acceleration_ecef_mps2.x[row] = derivatives.acceleration_ecef_mps2.x;
    columns.acceleration_ecef_mps2.y[row] = derivatives.acceleration_ecef_mps2.y;
    columns.acceleration_ecef_mps2.z[row] = derivatives.acceleration_ecef_mps2.z;
    std::copy(final_attitude.begin(), final_attitude.end(), columns.body_frd_to_ecef + 9 * row);
    columns.angular_velocity_body_radps.x[row] = derivatives.angular_velocity_body_radps.x;
    columns.angular_velocity_body_radps.y[row] = derivatives.angular_velocity_body_radps.y;
    columns.angular_velocity_body_radps.z[row] = derivatives.angular_velocity_body_radps.z;
    columns.source_time_ns[row] = end_time_ns;
  }
}

void advance_motion(StateStore& store, StateBatch& scratch,
                    std::size_t profile_index, const MotionProfile& profile,
                    std::int64_t end_time_ns) {
  store.gather_profile(profile_index, EntityMode::moving, scratch);
  bool any_advance = false;
  const MutableStateColumns columns = scratch.columns();
  for (std::size_t row = 0; row < scratch.size(); ++row)
    any_advance = any_advance || columns.source_time_ns[row] < end_time_ns;
  advance_motion_batch(scratch, profile, end_time_ns, nullptr, 0);
  if (any_advance) store.commit(scratch);
}

void refresh_motion_controls(StateBatch& scratch, const MotionProfile& profile) {
  const auto& limits = profile.flight_envelope;
  MutableStateColumns columns = scratch.columns();
  for (std::size_t row = 0; row < scratch.size(); ++row) {
    if (scratch.modes()[row] != EntityMode::moving) continue;
    Vector3 position{columns.position_ecef_m.x[row], columns.position_ecef_m.y[row], columns.position_ecef_m.z[row]};
    Vector3 velocity{columns.velocity_ecef_mps.x[row], columns.velocity_ecef_mps.y[row], columns.velocity_ecef_mps.z[row]};
    GeodeticPosition location = math::coordinates::ecef_to_geodetic(position);
    LocalFrame frame = math::coordinates::make_local_frame(location);
    const Vector3 ned_velocity = math::coordinates::rotate(math::coordinates::transpose(frame.ned_to_ecef), velocity);
    const double speed = std::hypot(ned_velocity.x, ned_velocity.y);
    const double heading = math::numerical::wrap_radians_positive(std::atan2(ned_velocity.y, ned_velocity.x));
    const double desired_heading = math::numerical::wrap_radians_positive(columns.heading_rad[row]);
    double heading_error = math::numerical::wrap_radians_signed(desired_heading - heading);
    if (std::abs(std::abs(heading_error) - math::constants::pi) < 1.0e-14) {
      heading_error = math::constants::pi;
    }
    const double heading_rate = std::abs(heading_error) < 1.0e-14 ? 0.0
        : std::copysign(limits.max_turn_rate_radps, heading_error);
    const double desired_speed = columns.horizontal_speed_mps[row];
    const double speed_rate = speed == desired_speed ? 0.0
        : std::copysign(desired_speed > speed ? limits.max_acceleration_mps2 : limits.max_deceleration_mps2,
                        desired_speed - speed);
    const double desired_altitude = columns.altitude_hae_m[row];
    const double altitude_delta = desired_altitude - location.altitude_m;
    const double vertical_limit = altitude_delta >= 0.0 ? limits.max_climb_rate_mps : limits.max_descent_rate_mps;
    if (altitude_delta != 0.0 && vertical_limit <= 0.0) {
      throw std::invalid_argument("invalid vertical rate limit");
    }
    const double control_vertical_rate = altitude_delta == 0.0 ? 0.0 : std::copysign(vertical_limit, altitude_delta);
    const double pitch = std::atan2(control_vertical_rate, speed);
    const math::Matrix3 local_attitude = math::coordinates::body_frd_to_ned(heading, pitch, 0.0);
    const math::Matrix3 body_to_ecef = math::coordinates::compose_rotations(frame.ned_to_ecef, local_attitude);
    const EndpointDerivatives derivatives = endpoint_derivatives(
        frame, local_attitude, heading, speed, control_vertical_rate, speed_rate, heading_rate);
    const Vector3 refreshed_velocity = math::coordinates::rotate(
        frame.ned_to_ecef, velocity_ned(speed, heading, control_vertical_rate));
    columns.velocity_ecef_mps.x[row] = refreshed_velocity.x;
    columns.velocity_ecef_mps.y[row] = refreshed_velocity.y;
    columns.velocity_ecef_mps.z[row] = refreshed_velocity.z;
    columns.acceleration_ecef_mps2.x[row] = derivatives.acceleration_ecef_mps2.x;
    columns.acceleration_ecef_mps2.y[row] = derivatives.acceleration_ecef_mps2.y;
    columns.acceleration_ecef_mps2.z[row] = derivatives.acceleration_ecef_mps2.z;
    std::copy(body_to_ecef.begin(), body_to_ecef.end(), columns.body_frd_to_ecef + 9 * row);
    columns.angular_velocity_body_radps.x[row] = derivatives.angular_velocity_body_radps.x;
    columns.angular_velocity_body_radps.y[row] = derivatives.angular_velocity_body_radps.y;
    columns.angular_velocity_body_radps.z[row] = derivatives.angular_velocity_body_radps.z;
  }
}

}  // namespace flight_models
