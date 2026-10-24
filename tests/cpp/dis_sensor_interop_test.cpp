/** Exercise flight output through the existing sensor-models receiving code. */
#include "flight_models/dis_entity_state.h"
#include "math/coordinates/geometry.hpp"
#include "sensor_models/dis/entity_state_decoder.hpp"
#include "sensor_models/dis/timestamp_alignment.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

// Read the retained dis-rs/SuperCell fixture; this is fixture loading, not a codec.
std::vector<std::uint8_t> read_hex(const char* path) {
  std::ifstream input(path);
  std::string hex;
  require(static_cast<bool>(input >> hex), "could not read SuperCell fixture");
  require(hex.size() % 2 == 0, "fixture must contain complete bytes");
  std::vector<std::uint8_t> bytes;
  for (std::size_t offset = 0; offset < hex.size(); offset += 2) {
    bytes.push_back(static_cast<std::uint8_t>(std::stoul(hex.substr(offset, 2), nullptr, 16)));
  }
  return bytes;
}

void near_vector(const math::Vector3& actual, const math::Vector3& expected,
                 double tolerance, const char* message) {
  require(std::abs(actual.x - expected.x) <= tolerance &&
          std::abs(actual.y - expected.y) <= tolerance &&
          std::abs(actual.z - expected.z) <= tolerance, message);
}
}  // namespace

int main(int argc, char** argv) {
  require(argc == 2, "expected SuperCell fixture path");
  const auto reference = sensor_models::dis::decode_entity_state_pdu(read_hex(argv[1]));
  const flight_models::DisEntityMetadata metadata{
      reference.identity.exercise, reference.identity.site,
      reference.identity.application, reference.identity.entity, reference.force_id,
      {reference.entity_type.kind, reference.entity_type.domain, reference.entity_type.country,
       reference.entity_type.category, reference.entity_type.subcategory,
       reference.entity_type.specific, reference.entity_type.extra}, reference.marking};
  flight_models::PhysicalState state;
  state.position_ecef_m = reference.position_ecef_m;
  state.velocity_ecef_mps = reference.velocity_ecef_mps;
  state.acceleration_ecef_mps2 = {0.125, -0.25, 0.5};
  state.angular_velocity_body_radps = {0.01, -0.02, 0.03};
  state.body_frd_to_ecef = math::coordinates::body_frd_to_ned(
      reference.orientation_rad.x, reference.orientation_rad.y, reference.orientation_rad.z);

  // Verify independent field interpretation, not merely OpenDIS's own round trip.
  const auto packet = flight_models::encode_entity_state(
      metadata, state, flight_models::EntityMode::moving, 0);
  require(packet.size() == 144, "unexpected base PDU length");
  const auto decoded = sensor_models::dis::decode_entity_state_pdu(packet);
  require(decoded.identity.exercise == reference.identity.exercise &&
          decoded.identity.site == reference.identity.site &&
          decoded.identity.application == reference.identity.application &&
          decoded.identity.entity == reference.identity.entity, "identity changed");
  require(decoded.force_id == reference.force_id && decoded.marking == reference.marking,
          "force or marking changed");
  require(decoded.entity_type.kind == reference.entity_type.kind &&
          decoded.entity_type.domain == reference.entity_type.domain &&
          decoded.entity_type.country == reference.entity_type.country &&
          decoded.entity_type.category == reference.entity_type.category &&
          decoded.entity_type.subcategory == reference.entity_type.subcategory &&
          decoded.entity_type.specific == reference.entity_type.specific &&
          decoded.entity_type.extra == reference.entity_type.extra, "entity type changed");
  near_vector(decoded.position_ecef_m, state.position_ecef_m, 0, "ECEF position changed");
  near_vector(decoded.velocity_ecef_mps, state.velocity_ecef_mps, 0, "ECEF velocity changed");
  near_vector(decoded.acceleration_ecef_mps2, state.acceleration_ecef_mps2, 0,
              "world acceleration changed");
  near_vector(decoded.angular_velocity_radps, state.angular_velocity_body_radps, 2e-9,
              "body angular velocity changed");
  near_vector(decoded.orientation_rad, reference.orientation_rad, 2e-7,
              "DIS orientation changed");
  require(decoded.dead_reckoning_algorithm == 4 && decoded.dis_timestamp == 1,
          "moving DR or initial absolute timestamp mismatch");

  // Non-hour-aligned scenario zero and an hourly wrap must retain source time.
  // DIS quantization is <1677 ns. Tick and epoch here are existing consumer context.
  constexpr std::int64_t hour_ns = 3'600'000'000'000LL;
  constexpr std::int64_t offset_ns = hour_ns - 20'000'000'000LL;
  for (const std::int64_t elapsed : {0LL, 19'999'999'999LL, 20'000'000'000LL,
                                    30'123'456'789LL, 7'200'000'000'000LL}) {
    state.source_time_ns = elapsed;
    const auto output = sensor_models::dis::decode_entity_state_pdu(
        flight_models::encode_entity_state(metadata, state, flight_models::EntityMode::moving,
                                           offset_ns));
    const auto expected = offset_ns + elapsed;
    const auto aligned = sensor_models::dis::align_absolute_timestamp(
        output.dis_timestamp, expected, 42, 0, 1'677);
    require(std::abs(aligned.scenario_time_ns - expected) <= 1'677,
            "source time exceeds DIS precision budget");
    require(aligned.tick_id == 42 && aligned.timeline_epoch == 0,
            "consumer context was changed");
  }

  const auto fixed = sensor_models::dis::decode_entity_state_pdu(
      flight_models::encode_entity_state(metadata, state, flight_models::EntityMode::fixed, 0));
  require(fixed.dead_reckoning_algorithm == 1, "fixed entity must use static DR");
  near_vector(fixed.velocity_ecef_mps, {}, 0, "fixed velocity must be zero");
  near_vector(fixed.acceleration_ecef_mps2, {}, 0, "fixed acceleration must be zero");
  near_vector(fixed.angular_velocity_radps, {}, 0, "fixed angular velocity must be zero");
}
