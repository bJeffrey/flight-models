#include "flight_models/dis_entity_state.h"

#include "math/coordinates/geometry.hpp"

#include <dis7/EntityStatePdu.h>
#include <dis7/utils/Endian.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace {
using namespace flight_models;

void require(bool condition) {
  if (!condition) throw std::runtime_error("DIS Entity State adapter check failed");
}

DIS::EntityStatePdu decode(const std::vector<std::uint8_t>& bytes) {
  DIS::EntityStatePdu pdu;
  const auto* raw = reinterpret_cast<const char*>(bytes.data());
  DIS::DataStream stream(raw, bytes.size(), DIS::BIG);
  pdu.unmarshal(stream);
  return pdu;
}

void rejects(const DisEntityMetadata& metadata, const PhysicalState& state,
             EntityMode mode, std::int64_t epoch) {
  bool threw = false;
  try {
    (void)encode_entity_state(metadata, state, mode, epoch);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw);
}

}  // namespace

int main() {
  DisEntityMetadata metadata;
  metadata.exercise_id = 9;
  metadata.site_id = 12;
  metadata.application_id = 34;
  metadata.entity_id = 56;
  metadata.force_id = 2;
  metadata.type = {1, 2, 225, 3, 4, 5, 6};
  metadata.marking = "TEST-PLANE";

  PhysicalState state;
  state.position_ecef_m = {6378137.125, -20.0, 30.5};
  state.velocity_ecef_mps = {100.25, -2.5, 0.125};
  state.acceleration_ecef_mps2 = {1.0, 2.0, 3.0};
  state.angular_velocity_body_radps = {0.1, 0.2, 0.3};
  const auto angles = math::Vector3{0.7, -0.3, 0.4};
  state.body_frd_to_ecef = math::coordinates::body_frd_to_ned(angles.x, angles.y, angles.z);
  state.source_time_ns = 3'599'000'000'000LL;

  const auto bytes = encode_entity_state(metadata, state, EntityMode::moving,
                                          2'000'000'000LL);
  require(bytes.size() == 144);
  const auto pdu = decode(bytes);
  require(pdu.getProtocolVersion() == 7);
  require(pdu.getPduType() == 1 && pdu.getProtocolFamily() == 1);
  require(pdu.getLength() == 144 && (pdu.getTimestamp() & 1U) == 1U);
  require(pdu.getTimestamp() == ((static_cast<unsigned int>(
      (((std::uint64_t{1} << 31) - 1) / 3600ULL)) << 1U) | 1U));
  const auto& entity_id = pdu.getEntityID();
  require(entity_id.getSimulationAddress().getSite() == 12);
  require(entity_id.getSimulationAddress().getApplication() == 34);
  require(entity_id.getEntityNumber() == 56 && pdu.getForceId() == 2);
  require(pdu.getEntityType().getCountry() == 225);
  require(pdu.getMarking().getCharacterSet() == 1);
  require(std::string(pdu.getMarking().getCharacters(), 10) == "TEST-PLANE");
  require(std::abs(pdu.getEntityLocation().getX() - state.position_ecef_m.x) < 1e-6);
  require(std::abs(pdu.getEntityLinearVelocity().getX() - 100.25F) < 1e-6F);
  require(pdu.getDeadReckoningParameters().getDeadReckoningAlgorithm() == 4);
  require(pdu.getDeadReckoningParameters().getEntityLinearAcceleration().getZ() == 3.0F);
  require(pdu.getDeadReckoningParameters().getEntityAngularVelocity().getY() == 0.2F);
  const auto& orientation = pdu.getEntityOrientation();
  require(std::abs(orientation.getPsi() - angles.x) < 1e-6);
  require(std::abs(orientation.getTheta() - angles.y) < 1e-6);
  require(std::abs(orientation.getPhi() - angles.z) < 1e-6);

  auto fixed = decode(encode_entity_state(metadata, state, EntityMode::fixed, 0));
  require(fixed.getDeadReckoningParameters().getDeadReckoningAlgorithm() == 1);
  require(fixed.getEntityLinearVelocity().getX() == 0.0F);
  require(fixed.getDeadReckoningParameters().getEntityLinearAcceleration().getX() == 0.0F);
  auto gimbal = state;
  gimbal.body_frd_to_ecef = math::coordinates::body_frd_to_ned(0.5, 1.5707963267948966, -0.2);
  const auto gimbal_decoded = decode(encode_entity_state(metadata, gimbal, EntityMode::moving, 0));
  require(std::abs(gimbal_decoded.getEntityOrientation().getPhi()) < 1e-6);

  auto bad = state;
  bad.source_time_ns = -1;
  rejects(metadata, bad, EntityMode::moving, 0);
  rejects(metadata, state, EntityMode::disabled, 0);
  rejects(metadata, state, EntityMode::moving, 3'600'000'000'000LL);
  auto bad_rotation = state;
  bad_rotation.body_frd_to_ecef[0] = 2.0;
  rejects(metadata, bad_rotation, EntityMode::moving, 0);
  auto bad_float = state;
  bad_float.velocity_ecef_mps.x = std::numeric_limits<double>::max();
  rejects(metadata, bad_float, EntityMode::moving, 0);
  auto bad_marking = metadata;
  bad_marking.marking = "TOO-LONG-MARK";
  rejects(bad_marking, state, EntityMode::moving, 0);
  bad_marking.marking = "bad\n";
  rejects(bad_marking, state, EntityMode::moving, 0);
  bad_marking.marking = std::string(1, static_cast<char>(0x80));
  rejects(bad_marking, state, EntityMode::moving, 0);
  metadata.marking = "12345678901";
  require(encode_entity_state(metadata, state, EntityMode::moving, 0).size() == 144);
  metadata.marking.clear();
  require(encode_entity_state(metadata, state, EntityMode::moving, 0).size() == 144);
  bad = state;
  bad.position_ecef_m.x = std::numeric_limits<double>::quiet_NaN();
  rejects(metadata, bad, EntityMode::moving, 0);
  bad = state;
  bad.angular_velocity_body_radps.z = std::numeric_limits<double>::infinity();
  rejects(metadata, bad, EntityMode::moving, 0);
  rejects(metadata, state, EntityMode::moving, -1);

  // The full elapsed-time range must wrap before arithmetic can overflow.
  state.source_time_ns = std::numeric_limits<std::int64_t>::max();
  const auto maximum_time = decode(encode_entity_state(metadata, state, EntityMode::moving, 0));
  const auto remainder = state.source_time_ns % 3'600'000'000'000LL;
  const long double represented_ns = static_cast<long double>(maximum_time.getTimestamp() >> 1U) *
      3'600'000'000'000.0L / 2'147'483'647.0L;
  require(std::abs(represented_ns - remainder) < 1677.0L);
}
