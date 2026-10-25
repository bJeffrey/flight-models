/** Verify reported-state frames, covariance fields and simulation timestamps. */
#include "flight_models/position_report.h"
#include "math/coordinates/geodetic.hpp"
#include "math/coordinates/geometry.hpp"
#include "math/measurements/covariance.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Json = nlohmann::json;
void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

// Float comparisons have fixture tolerances; structural/string fields are exact.
void compare(const Json& actual, const Json& expected) {
  if (expected.is_number()) {
    require(actual.is_number() && std::abs(actual.get<double>() - expected.get<double>()) < 1e-8,
            "numeric PRD fixture mismatch");
  } else if (expected.is_object()) {
    for (const auto& item : expected.items()) compare(actual.at(item.key()), item.value());
  } else if (expected.is_array()) {
    require(actual.size() == expected.size(), "PRD array count changed");
    for (std::size_t index = 0; index < expected.size(); ++index) compare(actual.at(index), expected[index]);
  } else {
    require(actual == expected, "PRD metadata mismatch");
  }
}

template <typename Exception, typename Operation>
void rejects(Operation operation) {
  bool rejected = false;
  try { operation(); } catch (const Exception&) { rejected = true; }
  require(rejected, "expected invalid report input rejection");
}
}  // namespace

int main(int argc, char** argv) {
  require(argc == 2 || argc == 3, "usage: position-report-test <repo> [generated-prd.json]");
  std::ifstream input(std::string(argv[1]) + "/tests/fixtures/uci_position_report_detailed_v2_5.json");
  require(static_cast<bool>(input), "cannot read PRD fixture");
  Json fixture;
  input >> fixture;
  const auto& root = fixture.at("PositionReportDetailed");
  const auto& report = root.at("MessageData").at("PositionReportData")[0];
  const auto& kinematics = report.at("Kinematics");
  const auto& point = kinematics.at("Position").at("AbsolutePoint");
  const auto& velocity = kinematics.at("Velocity");
  const auto& attitude = kinematics.at("Orientation");
  flight_models::PrdMetadata metadata{
      root.at("MessageHeader").at("SystemID").at("UUID"),
      report.at("PositionSource").at("SystemID").at("UUID"),
      root.at("SecurityInformation").at("Classification"),
      root.at("SecurityInformation").at("OwnerProducer")[0].at("GovernmentIdentifier"),
      report.at("NavigationSolutionState")};
  const math::coordinates::GeodeticPosition position{
      point.at("Latitude"), point.at("Longitude"), point.at("Altitude")};
  const auto ned_to_ecef = math::coordinates::ned_to_ecef_rotation(position);
  flight_models::NavigationSolution navigation;
  navigation.position_ecef_m = math::coordinates::geodetic_to_ecef(position);
  navigation.velocity_ecef_mps = math::coordinates::rotate(ned_to_ecef,
      {velocity.at("NorthSpeed"), velocity.at("EastSpeed"), velocity.at("DownSpeed")});
  navigation.body_frd_to_ecef = math::coordinates::compose_rotations(ned_to_ecef,
      math::coordinates::body_frd_to_ned(attitude.at("Yaw"), attitude.at("Pitch"), attitude.at("Roll")));

  // This is a reported-state adapter specimen, not an approved nonideal model.
  // Full nonsymmetric cross blocks expose transposed or omitted wire entries.
  math::Covariance6 covariance_ned{
      4, .5, .2, .1, .2, .3,
      .5, 9, .3, .4, .5, .6,
      .2, .3, 16, .7, .8, .9,
      .1, .4, .7, 1, .1, .2,
      .2, .5, .8, .1, 2, .3,
      .3, .6, .9, .2, .3, 3};
  navigation.covariance_ecef = math::measurements::rotate_position_velocity_covariance(
      covariance_ned, ned_to_ecef);
  constexpr std::int64_t epoch_ns = 1'786'708'800'000'000'000LL; // 2026-08-14T12:00:00Z
  const auto encoded = flight_models::encode_position_report_detailed(metadata, navigation, epoch_ns, 0);

  Json expected = fixture;
  auto& expected_root = expected["PositionReportDetailed"];
  expected_root["MessageHeader"]["Timestamp"] = "2026-08-14T12:00:00.000000000Z";
  auto& expected_report = expected_root["MessageData"]["PositionReportData"][0];
  expected_report["PositionSource"] = {{"SubsystemID", {{"UUID", metadata.navigation_subsystem_id}}}};
  expected_report["Kinematics"]["Position"]["AbsolutePoint"]["Timestamp"] =
      "2026-08-14T12:00:00.000000000Z";
  expected_report["Kinematics"]["Velocity"]["Timestamp"] = "2026-08-14T12:00:00.000000000Z";
  compare(encoded, expected);

  using flight_models::format_uci_simulation_time;
  require(format_uci_simulation_time(0, 30'123'456'789) == "1970-01-01T00:00:30.123456789Z",
          "scenario-relative nanoseconds lost");
  require(format_uci_simulation_time(-1, 0) == "1969-12-31T23:59:59.999999999Z",
          "negative UTC epoch fraction incorrect");
  require(format_uci_simulation_time(epoch_ns, 43'200'000'000'001) ==
              "2026-08-15T00:00:00.000000001Z", "calendar rollover incorrect");
  rejects<std::overflow_error>([] { (void)format_uci_simulation_time(
      std::numeric_limits<std::int64_t>::max(), 1); });
  rejects<std::invalid_argument>([] { (void)format_uci_simulation_time(0, -1); });
  auto bad = navigation;
  bad.source_time_ns = -1;
  rejects<std::invalid_argument>([&] { (void)flight_models::encode_position_report_detailed(metadata, bad, epoch_ns, 0); });
  bad = navigation;
  bad.source_time_ns = 1;
  rejects<std::invalid_argument>([&] { (void)flight_models::encode_position_report_detailed(metadata, bad, epoch_ns, 0); });
  bad = navigation;
  bad.covariance_ecef[0] = std::numeric_limits<double>::quiet_NaN();
  rejects<std::invalid_argument>([&] { (void)flight_models::encode_position_report_detailed(metadata, bad, epoch_ns, 0); });

  // Exercise the actual ideal model -> reported state -> PRD path. The adapter
  // receives only the reported copy, which stays valid after physical mutation.
  flight_models::StateStore store(1, 1);
  flight_models::PhysicalState physical;
  physical.position_ecef_m = navigation.position_ecef_m;
  physical.velocity_ecef_mps = navigation.velocity_ecef_mps;
  physical.body_frd_to_ecef = navigation.body_frd_to_ecef;
  physical.source_time_ns = 30'123'456'789;
  const auto handle = store.spawn("ownship", 0, flight_models::EntityMode::moving, physical, {});
  flight_models::Snapshot snapshot(1);
  store.snapshot(snapshot);
  flight_models::NavigationBatch batch(1, {"ideal_test", {}, {}, 0.0, 0.0, 0});
  batch.generate(snapshot);
  store.erase(handle);
  const auto generated = flight_models::encode_position_report_detailed(
      metadata, batch.solution(0), epoch_ns, 31'987'654'321);
  const auto& generated_root = generated.at("PositionReportDetailed");
  const auto& generated_report = generated_root.at("MessageData").at("PositionReportData")[0];
  require(generated_root.at("MessageHeader").at("Timestamp") ==
              "2026-08-14T12:00:31.987654321Z", "creation time is not simulation time");
  require(generated_report.at("Kinematics").at("Position").at("AbsolutePoint").at("Timestamp") ==
              "2026-08-14T12:00:30.123456789Z", "validity time was replaced with creation time");
  for (const auto& block : generated_report.at("KinematicsError")) {
    for (const auto& value : block) require(value == 0, "ideal covariance is not zero");
  }
  require(!generated_report.contains("SimulationTargetNumber"), "truth correlation leaked into PRD");
  if (argc == 3) {
    std::ofstream output(argv[2]);
    output << generated.dump(2) << '\n';
    require(static_cast<bool>(output), "cannot write generated schema specimen");
  }
}
