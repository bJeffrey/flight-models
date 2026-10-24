#include "flight_models/dis_entity_state.h"

#include "math/coordinates/geometry.hpp"

#include <dis7/EntityStatePdu.h>
#include <dis7/utils/Endian.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace flight_models {
namespace {
constexpr std::uint64_t kHourNs = 3600'000'000'000ULL;
constexpr std::uint64_t kTimestampUnitsMaximum = (std::uint64_t{1} << 31) - 1;

float checked_float(double value, const char* field) {
  if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
    throw std::invalid_argument(std::string(field) + " must be finite and representable as float");
  }
  return static_cast<float>(value);
}

DIS::Vector3Float vector_float(const math::Vector3& value, const char* field) {
  DIS::Vector3Float result;
  result.setX(checked_float(value.x, field));
  result.setY(checked_float(value.y, field));
  result.setZ(checked_float(value.z, field));
  return result;
}

}  // namespace

std::vector<std::uint8_t> encode_entity_state(
    const DisEntityMetadata& metadata, const PhysicalState& state, const EntityMode mode,
    const std::int64_t scenario_epoch_hour_ns) {
  if (mode == EntityMode::disabled) {
    throw std::invalid_argument("disabled entities have no DIS state");
  }
  if (mode != EntityMode::fixed && mode != EntityMode::moving) {
    throw std::invalid_argument("invalid entity mode");
  }
  if (state.source_time_ns < 0 || scenario_epoch_hour_ns < 0 ||
      static_cast<std::uint64_t>(scenario_epoch_hour_ns) >= kHourNs) {
    throw std::invalid_argument("source time must be nonnegative and epoch offset within one UTC hour");
  }
  if (metadata.marking.size() > 11) {
    throw std::invalid_argument("marking must contain at most 11 ASCII printable bytes");
  }
  for (const unsigned char byte : metadata.marking) {
    if (byte < 0x20 || byte > 0x7e) {
      throw std::invalid_argument("marking must contain at most 11 ASCII printable bytes");
    }
  }
  for (const double value : {state.position_ecef_m.x, state.position_ecef_m.y,
                             state.position_ecef_m.z}) {
    if (!std::isfinite(value)) throw std::invalid_argument("ECEF position must be finite");
  }
  // Preserve canonical ECEF vectors; only attitude representation and wire
  // float precision change here. The shared inverse validates the DCM.
  const auto euler = math::coordinates::euler_zyx_from_rotation(state.body_frd_to_ecef);
  const auto state_velocity = vector_float(state.velocity_ecef_mps, "velocity");
  const auto acceleration = vector_float(state.acceleration_ecef_mps2, "acceleration");
  const auto angular_velocity = vector_float(state.angular_velocity_body_radps, "angular velocity");

  // DIS timestamps occupy 31 fraction bits plus the absolute-time indicator.
  const std::uint64_t elapsed_remainder = static_cast<std::uint64_t>(state.source_time_ns) % kHourNs;
  const std::uint64_t within_hour =
      (static_cast<std::uint64_t>(scenario_epoch_hour_ns) + elapsed_remainder) % kHourNs;
  // Split whole and fractional seconds to preserve integer quantization without overflow.
  const std::uint64_t whole_seconds = within_hour / 1'000'000'000ULL;
  const std::uint64_t remainder_ns = within_hour % 1'000'000'000ULL;
  const std::uint64_t whole_product = whole_seconds * kTimestampUnitsMaximum;
  const std::uint64_t ticks = whole_product / 3600ULL +
      ((whole_product % 3600ULL) * 1'000'000'000ULL +
       remainder_ns * kTimestampUnitsMaximum) / kHourNs;

  DIS::EntityStatePdu pdu;
  pdu.setProtocolVersion(7);
  pdu.setExerciseID(metadata.exercise_id);
  pdu.setProtocolFamily(1);
  pdu.setTimestamp((static_cast<unsigned int>(ticks) << 1U) | 1U);
  pdu.setLength(static_cast<unsigned short>(pdu.getMarshalledSize()));

  // Identity/type come from the scenario mapping, never array slots or markings.
  DIS::SimulationAddress address;
  address.setSite(metadata.site_id);
  address.setApplication(metadata.application_id);
  DIS::EntityID id;
  id.setSimulationAddress(address);
  id.setEntityNumber(metadata.entity_id);
  pdu.setEntityID(id);
  pdu.setForceId(metadata.force_id);
  DIS::EntityType type;
  type.setEntityKind(metadata.type.kind);
  type.setDomain(metadata.type.domain);
  type.setCountry(metadata.type.country);
  type.setCategory(metadata.type.category);
  type.setSubcategory(metadata.type.subcategory);
  type.setSpecific(metadata.type.specific);
  type.setExtra(metadata.type.extra);
  pdu.setEntityType(type);

  DIS::Vector3Double location;
  location.setX(state.position_ecef_m.x);
  location.setY(state.position_ecef_m.y);
  location.setZ(state.position_ecef_m.z);
  pdu.setEntityLocation(location);
  DIS::Vector3Float velocity;
  if (mode == EntityMode::moving) velocity = state_velocity;
  pdu.setEntityLinearVelocity(velocity);
  DIS::EulerAngles orientation;
  orientation.setPsi(checked_float(euler.x, "yaw"));
  orientation.setTheta(checked_float(euler.y, "pitch"));
  orientation.setPhi(checked_float(euler.z, "roll"));
  pdu.setEntityOrientation(orientation);

  DIS::EntityMarking marking;
  marking.setCharacterSet(1);
  char marking_chars[12]{};
  std::memcpy(marking_chars, metadata.marking.data(), metadata.marking.size());
  marking.setCharacters(marking_chars);
  pdu.setMarking(marking);

  // World-frame acceleration and body angular rates retain their native frames.
  // Static entities publish zero derivatives even if scratch values exist.
  DIS::DeadReckoningParameters dr;
  dr.setDeadReckoningAlgorithm(mode == EntityMode::moving ? 4 : 1);
  if (mode == EntityMode::moving) {
    dr.setEntityLinearAcceleration(acceleration);
    dr.setEntityAngularVelocity(angular_velocity);
  }
  pdu.setDeadReckoningParameters(dr);

  // OpenDIS owns the complete packet layout and byte ordering.
  DIS::DataStream stream(DIS::BIG);
  pdu.marshal(stream);
  std::vector<std::uint8_t> bytes(stream.size());
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    bytes[i] = static_cast<std::uint8_t>(stream[static_cast<unsigned int>(i)]);
  }
  return bytes;
}

}  // namespace flight_models
