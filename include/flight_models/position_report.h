/** @file UCI 2.5 PRD projection of reported navigation, independent of transport. */
#pragma once

#include "flight_models/navigation.h"
#include <nlohmann/json.hpp>
#include <cstdint>
#include <string>

namespace flight_models {

/** Explicit platform identity and publication metadata supplied by integration. */
struct PrdMetadata {
  std::string platform_system_id; /**< Aircraft SystemID UUID; not the EGI identity. */
  std::string navigation_subsystem_id; /**< Producing EGI SubsystemID UUID. */
  std::string classification; /**< UCI security classification, e.g. U. */
  std::string owner_government; /**< UCI OwnerProducer GovernmentIdentifier. */
  std::string navigation_solution_state; /**< UCI solution-state enumeration. */
};

/**
 * Format authoritative simulation time as UCI's Zulu xs:dateTime.
 * @param scenario_epoch_unix_ns Scenario origin, signed Unix-epoch nanoseconds.
 * @param elapsed_ns Nonnegative elapsed scenario nanoseconds, starting at zero.
 * @return UTC timestamp with exactly nine fractional second digits and Z suffix.
 * No system clock is read. This adapter currently uses the Linux UTC calendar API.
 * @throws std::invalid_argument for negative elapsed time.
 * @throws std::overflow_error if epoch plus elapsed exceeds int64 nanoseconds.
 * @throws std::runtime_error if the platform cannot format the resulting date.
 */
[[nodiscard]] std::string format_uci_simulation_time(
    std::int64_t scenario_epoch_unix_ns, std::int64_t elapsed_ns);

/**
 * Project one reported navigation solution to a UCI 2.5 PRD JSON object.
 * @param metadata Explicit identity/security/solution state; enum/UUID validity
 *        is an integration/schema responsibility. Empty fields are rejected.
 * @param navigation Completed navigation state in ECEF SI units. Covariance is
 *        symmetric positive semidefinite, ordered [x,y,z,vx,vy,vz], supplied by
 *        the navigation model. It is not synthesized from physical truth here.
 * @param scenario_epoch_unix_ns Authoritative scenario origin in Unix ns.
 * @param creation_elapsed_ns Message creation time in scenario ns, at or after
 *        navigation.source_time_ns; forwarding preserves this original header.
 * @return Owned PRD with geodetic WGS-84 HAE position, NED velocity/covariance,
 *         local yaw/pitch/roll, EGI SubsystemID and separate creation/validity times.
 * The adapter derives its local frame from reported navigation, never truth.
 * Callers apply reporting membership before publication. No SimulationTargetNumber
 * or other simulation truth correlation is added to the operational report.
 * @throws std::invalid_argument for invalid time, pose, nonfinite covariance,
 *         velocity or empty metadata. Covariance PSD validity belongs to the model.
 * @throws std::overflow_error for absolute timestamp overflow.
 */
[[nodiscard]] nlohmann::json encode_position_report_detailed(
    const PrdMetadata& metadata, const NavigationSolution& navigation,
    std::int64_t scenario_epoch_unix_ns, std::int64_t creation_elapsed_ns);

}  // namespace flight_models
