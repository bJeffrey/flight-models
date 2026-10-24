#include "flight_models/position_report.h"

#include "math/coordinates/geodetic.hpp"
#include "math/coordinates/geometry.hpp"
#include "math/measurements/covariance.hpp"

#include <cmath>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace flight_models {

std::string format_uci_simulation_time(const std::int64_t scenario_epoch_unix_ns,
                                      const std::int64_t elapsed_ns) {
  if (elapsed_ns < 0) throw std::invalid_argument("elapsed simulation time must be nonnegative");
  if (scenario_epoch_unix_ns > std::numeric_limits<std::int64_t>::max() - elapsed_ns) {
    throw std::overflow_error("simulation UTC timestamp exceeds int64 nanoseconds");
  }
  const std::int64_t absolute_ns = scenario_epoch_unix_ns + elapsed_ns;
  constexpr std::int64_t nanoseconds_per_second = 1'000'000'000;
  auto seconds = absolute_ns / nanoseconds_per_second;
  auto fraction = absolute_ns % nanoseconds_per_second;
  // Floor negative epochs so the fraction always describes time after the second.
  if (fraction < 0) {
    --seconds;
    fraction += nanoseconds_per_second;
  }
  const auto calendar_seconds = static_cast<std::time_t>(seconds);
  std::tm calendar{};
  if (gmtime_r(&calendar_seconds, &calendar) == nullptr) {
    throw std::runtime_error("cannot convert simulation time to UTC calendar");
  }
  std::ostringstream result;
  result << std::put_time(&calendar, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setfill('0') << std::setw(9) << fraction << 'Z';
  if (!result) throw std::runtime_error("cannot format simulation UTC timestamp");
  return result.str();
}

nlohmann::json encode_position_report_detailed(
    const PrdMetadata& metadata, const NavigationSolution& navigation,
    const std::int64_t scenario_epoch_unix_ns, const std::int64_t creation_elapsed_ns) {
  if (metadata.platform_system_id.empty() || metadata.navigation_subsystem_id.empty() ||
      metadata.classification.empty() || metadata.owner_government.empty() ||
      metadata.navigation_solution_state.empty()) {
    throw std::invalid_argument("PRD publication metadata must be explicit and nonempty");
  }
  if (navigation.source_time_ns < 0 || creation_elapsed_ns < navigation.source_time_ns) {
    throw std::invalid_argument("PRD creation cannot precede nonnegative navigation validity time");
  }
  for (const double value : {navigation.velocity_ecef_mps.x, navigation.velocity_ecef_mps.y,
                             navigation.velocity_ecef_mps.z}) {
    if (!std::isfinite(value)) throw std::invalid_argument("navigation velocity must be finite");
  }
  for (const double value : navigation.covariance_ecef) {
    if (!std::isfinite(value)) throw std::invalid_argument("navigation covariance must be finite");
  }
  if (!math::coordinates::is_proper_rotation(navigation.body_frd_to_ecef, 1e-6)) {
    throw std::invalid_argument("navigation attitude must be a proper rotation");
  }

  // Each reported position defines its own frame, including in a nonideal model.
  // This is the only ECEF-to-geodetic conversion at this report boundary.
  const auto position = math::coordinates::ecef_to_geodetic(navigation.position_ecef_m);
  const auto ecef_to_ned = math::coordinates::transpose(
      math::coordinates::ned_to_ecef_rotation(position));
  const auto velocity = math::coordinates::rotate(ecef_to_ned, navigation.velocity_ecef_mps);
  const auto body_to_ned = math::coordinates::compose_rotations(
      ecef_to_ned, navigation.body_frd_to_ecef);
  const auto orientation = math::coordinates::euler_zyx_from_rotation(body_to_ned);
  const auto covariance = math::measurements::rotate_position_velocity_covariance(
      navigation.covariance_ecef, ecef_to_ned);
  for (const double value : covariance) {
    if (!std::isfinite(value)) throw std::invalid_argument("NED covariance overflow");
  }
  for (const double value : {velocity.x, velocity.y, velocity.z}) {
    if (!std::isfinite(value)) throw std::invalid_argument("NED navigation velocity overflow");
  }

  const std::string validity = format_uci_simulation_time(
      scenario_epoch_unix_ns, navigation.source_time_ns);
  const std::string creation = format_uci_simulation_time(
      scenario_epoch_unix_ns, creation_elapsed_ns);

  // UCI's position/velocity cross block is a full 3x3; preserve all nine entries.
  const nlohmann::json errors = {
      {"PositionPositionCovariance", {{"PnPn", covariance[0]}, {"PnPe", covariance[1]},
          {"PnPd", covariance[2]}, {"PePe", covariance[7]}, {"PePd", covariance[8]},
          {"PdPd", covariance[14]}}},
      {"PositionVelocityCovariance", {{"PnVn", covariance[3]}, {"PnVe", covariance[4]},
          {"PnVd", covariance[5]}, {"PeVn", covariance[9]}, {"PeVe", covariance[10]},
          {"PeVd", covariance[11]}, {"PdVn", covariance[15]}, {"PdVe", covariance[16]},
          {"PdVd", covariance[17]}}},
      {"VelocityVelocityCovariance", {{"VnVn", covariance[21]}, {"VnVe", covariance[22]},
          {"VnVd", covariance[23]}, {"VeVe", covariance[28]}, {"VeVd", covariance[29]},
          {"VdVd", covariance[35]}}}};
  const nlohmann::json report = {
      {"PositionSource", {{"SubsystemID", {{"UUID", metadata.navigation_subsystem_id}}}}},
      {"NavigationSolutionState", metadata.navigation_solution_state},
      {"Kinematics", {
          {"Position", {{"AbsolutePoint", {{"Latitude", position.latitude_rad},
              {"Longitude", position.longitude_rad}, {"Altitude", position.altitude_m},
              {"AltitudeReference", "WGS_HAE"}, {"Timestamp", validity}}}}},
          {"Velocity", {{"NorthSpeed", velocity.x}, {"EastSpeed", velocity.y},
              {"DownSpeed", velocity.z}, {"Timestamp", validity}}},
          {"Orientation", {{"Yaw", orientation.x}, {"Pitch", orientation.y},
              {"Roll", orientation.z}, {"Timestamp", validity}}}}},
      {"KinematicsError", errors}};

  // The standard message has no model provenance field. Keep ideal/nonideal
  // provenance with navigation state/configuration rather than invent wire fields.
  return {{"PositionReportDetailed", {
      {"SecurityInformation", {{"Classification", metadata.classification},
          {"OwnerProducer", nlohmann::json::array({
              {{"GovernmentIdentifier", metadata.owner_government}}})}}},
      {"MessageHeader", {{"SystemID", {{"UUID", metadata.platform_system_id}}},
          {"Timestamp", creation}, {"SchemaVersion", "002.5.0"}, {"Mode", "SIMULATION"}}},
      {"MessageData", {{"PositionReportData", nlohmann::json::array({report})}}}}}};
}

}  // namespace flight_models
