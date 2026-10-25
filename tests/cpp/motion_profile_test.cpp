#include "flight_models/motion_profile.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using Json = nlohmann::json;

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    path_ = std::filesystem::temp_directory_path() /
            ("motion-profile-test-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory() { std::filesystem::remove_all(path_); }
  std::string write(const std::string& name, const Json& value) const {
    const auto path = path_ / name;
    std::ofstream output(path);
    output << value.dump(2);
    if (!output) throw std::runtime_error("cannot write temporary fixture");
    return path.string();
  }

 private:
  std::filesystem::path path_;
};

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename Exception, typename Function>
void rejects(Function&& function, const std::string& context) {
  try {
    function();
  } catch (const Exception&) {
    return;
  }
  throw std::runtime_error("expected rejection: " + context);
}

void check(const std::string& profile_path, const std::string& schema_path) {
  const auto profile = flight_models::load_motion_profile(profile_path, schema_path);
  require(profile.profile_id == "synthetic-fixed-wing", "profile id was not loaded");
  require(profile.model == "bounded_kinematic_no_wind" && profile.navigation.model == "ideal_test" &&
              profile.navigation.position_sigma_m == 0 && profile.navigation.seed == 0,
          "explicit model/navigation metadata was not retained");
  require(profile.flight_envelope.min_speed_mps == 92.6 &&
              profile.flight_envelope.max_altitude_hae_m == 15000.0,
          "flight envelope values differ from profile");
  require(profile.waypoint_capture_radius_m == 150.0, "capture radius was not loaded");
  require(profile.integration.max_substep_ns == 10000000 &&
              profile.integration.max_interval_ns == 2000000000 &&
              profile.integration.max_substeps_per_interval == 4096,
          "integration values differ from profile");
  require(profile.publication.dis_period_ns == 100000000 &&
              profile.publication.prd_period_ns == 200000000,
          "publication values differ from profile");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: motion-profile-test <repository-root>");
    const std::filesystem::path root(argv[1]);
    const auto profile_path = (root / "config/profiles/synthetic_fixed_wing.json").string();
    const auto schema_path = (root / "config/motion_profile.schema.json").string();
    check(profile_path, schema_path);
    const auto error_profile = flight_models::load_motion_profile(
        (root / "config/profiles/synthetic_navigation_error.json").string(), schema_path);
    require(error_profile.navigation.model == "bias_gaussian" &&
                error_profile.navigation.position_sigma_m > 0,
            "nonzero navigation error profile was not loaded");

    TemporaryDirectory temporary;
    std::ifstream input(profile_path);
    Json base = Json::parse(input);
    const auto accepts = [&](const Json& value) {
      flight_models::load_motion_profile(temporary.write("candidate.json", value), schema_path);
    };
    const auto rejects_profile = [&](const Json& value) {
      rejects<std::invalid_argument>([&] { accepts(value); }, "invalid profile");
    };

    Json changed = base;
    changed["model"] = "unsupported_model";
    rejects_profile(changed);
    changed = base;
    changed["navigation"]["model"] = "nonideal_unimplemented";
    rejects_profile(changed);
    changed = base;
    changed["navigation"].erase("seed");
    rejects_profile(changed);
    changed = base;
    changed["navigation"]["seed"] = -1;
    rejects_profile(changed);
    changed = base;
    changed["navigation"]["seed"] = 1.0;
    rejects_profile(changed);
    changed = base;
    changed["navigation"]["seed"] = std::numeric_limits<std::uint64_t>::max();
    accepts(changed);
    changed = base;
    changed["navigation"]["position_bias_ecef_m"][0] = 1.0;
    rejects_profile(changed);  // ideal_test requires zero bias and sigma.
    changed = base;
    changed.erase("description");
    rejects_profile(changed);
    changed = base;
    changed["flight_envelope"]["unexpected"] = 1;
    rejects_profile(changed);
    changed = base;
    changed["flight_envelope"]["min_speed_mps"] = 0;
    rejects_profile(changed);
    changed = base;
    changed["flight_envelope"]["min_speed_mps"] = 210;
    rejects_profile(changed);
    changed = base;
    changed["flight_envelope"]["min_altitude_hae_m"] = 16000;
    rejects_profile(changed);
    changed["flight_envelope"]["min_altitude_hae_m"] = 15000;
    rejects_profile(changed);
    changed = base;
    changed["integration"]["max_interval_ns"] = -1;
    rejects_profile(changed);
    changed = base;
    changed["publication"]["dis_period_ns"] = -1;
    rejects_profile(changed);
    changed = base;
    changed["publication"]["dis_period_ns"] = 1.0;
    rejects_profile(changed);  // Floating JSON tokens are not exact integer durations.

    // Cadences are independent schedules and need not divide one another.
    changed = base;
    changed["publication"]["dis_period_ns"] = 70000000;
    changed["publication"]["prd_period_ns"] = 110000000;
    accepts(changed);

    changed = base;
    changed["integration"]["max_substep_ns"] = 2;
    changed["integration"]["max_interval_ns"] = 5;
    changed["integration"]["max_substeps_per_interval"] = 2;
    rejects_profile(changed);  // ceil(5/2) requires three work units.
    changed["integration"]["max_substeps_per_interval"] = 3;
    accepts(changed);
    changed["integration"]["max_interval_ns"] = 6;
    accepts(changed);  // Exact division needs exactly three work units.

    // Largest representable interval exercises quotient logic without overflow.
    changed = base;
    changed["integration"]["max_substep_ns"] = 1;
    changed["integration"]["max_interval_ns"] = std::numeric_limits<std::int64_t>::max();
    changed["integration"]["max_substeps_per_interval"] = std::numeric_limits<std::int64_t>::max();
    const auto largest_interval = flight_models::load_motion_profile(
        temporary.write("candidate.json", changed), schema_path);
    require(largest_interval.integration.max_interval_ns == std::numeric_limits<std::int64_t>::max(),
            "INT64_MAX interval did not survive typed conversion");
    changed["integration"]["max_substep_ns"] = 2;
    changed["integration"]["max_substeps_per_interval"] = std::numeric_limits<std::int64_t>::max() / 2;
    rejects_profile(changed);
    changed["integration"]["max_interval_ns"] =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max()) + 1;
    rejects_profile(changed);

    std::cout << "motion profile loader tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
