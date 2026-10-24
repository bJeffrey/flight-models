/** @file
 * OpenDIS7 serialization boundary for flight-model physical state.
 */
#pragma once

#include "flight_models/state_store.h"

#include <cstdint>
#include <string>
#include <vector>

namespace flight_models {

/** DIS entity type tuple copied to the PDU without reinterpretation. */
struct DisEntityType {
  std::uint8_t kind{}; /**< DIS entity kind. */
  std::uint8_t domain{}; /**< DIS entity domain. */
  std::uint16_t country{}; /**< DIS country code. */
  std::uint8_t category{}; /**< DIS entity category. */
  std::uint8_t subcategory{}; /**< DIS entity subcategory. */
  std::uint8_t specific{}; /**< DIS entity specific value. */
  std::uint8_t extra{}; /**< DIS entity extra value. */
};

/** DIS entity identity and type fields, including the fixed-width display marking. */
struct DisEntityMetadata {
  std::uint8_t exercise_id{}; /**< DIS exercise ID, 0..255. */
  std::uint16_t site_id{}; /**< DIS site ID. */
  std::uint16_t application_id{}; /**< DIS application ID. */
  std::uint16_t entity_id{}; /**< DIS entity ID. */
  std::uint8_t force_id{}; /**< DIS force affiliation. */
  DisEntityType type{}; /**< DIS entity type record. */
  std::string marking; /**< Printable ASCII marking, at most 11 bytes; empty allowed. */
};

/** Serialize one flight-model state as a DIS 7 Entity State PDU.
 * Position is ECEF metres; velocity and acceleration are ECEF m/s and m/s²;
 * attitude is a body-FRD to ECEF proper DCM; angular rate is body-FRD rad/s.
 * source_time_ns is elapsed scenario time and must be nonnegative. The epoch
 * offset locates scenario time zero within a UTC hour. DIS timestamp ticks are
 * computed with floor(hour_fraction * (2^31-1)), shifted and marked absolute,
 * and wrap hourly. Fixed entities publish zero velocity and DR derivatives;
 * moving entities use DR algorithm 4 with supplied ECEF acceleration and body
 * angular rate. Disabled entities cannot be serialized.
 * @param metadata Explicit scenario-provided identity/type; no slot identity is used.
 * @param state Immutable completed physical sample; never modified.
 * @param mode Fixed or moving publication behavior; fixed uses DR algorithm 1.
 * @param scenario_epoch_hour_ns Scenario UTC epoch offset within an hour,
 *        in [0, 3,600,000,000,000) nanoseconds; independent of wall-clock time.
 * @return Owned 144-byte network-order datagram, serialized by OpenDIS. No
 *         variable parameters, damage appearance or capability flags are emitted.
 * @throws std::invalid_argument for invalid metadata, time, mode, finite/range,
 *         or DCM constraints.
 */
[[nodiscard]] std::vector<std::uint8_t> encode_entity_state(
    const DisEntityMetadata& metadata, const PhysicalState& state, EntityMode mode,
    std::int64_t scenario_epoch_hour_ns);

}  // namespace flight_models
