#include "flight_models/runtime.h"

#include <cassert>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace {
using nlohmann::json;

json entity(bool prd, int dis_id) {
  json value{{"id", prd ? "reported" : "red"}, {"mode", prd ? "moving" : "fixed"},
      {"latitude_rad", 0.2}, {"longitude_rad", -0.3}, {"altitude_hae_m", prd ? 1000.0 : 0.0},
      {"heading_rad", 0.4}, {"horizontal_speed_mps", prd ? 120.0 : 0.0},
      {"dis", {{"exercise_id", 1}, {"site_id", 2}, {"application_id", 3},
          {"entity_id", dis_id}, {"force_id", 1},
          {"type", {{"kind", 1}, {"domain", 2}, {"country", 225}, {"category", 1},
              {"subcategory", 0}, {"specific", 0}, {"extra", 0}}}, {"marking", "TEST"}}},
      {"prd", nullptr}};
  if (prd) value["prd"] = {{"platform_system_id", "58087a47-53e1-53e8-a811-10f024f89b80"},
      {"navigation_subsystem_id", "1750e9e0-ab09-5a48-97ce-e0b4e566363d"}, {"classification", "U"},
      {"owner_government", "USA"}, {"navigation_solution_state", "BLENDED"},
      {"asb_url", "ws://asb-ownship:9000/owp"}, {"topic", "navigation"}};
  return value;
}

json config(const std::filesystem::path& root) {
  return {{"capacity", 2}, {"profile_path", (root / "config/profiles/synthetic_fixed_wing.json").string()},
      {"profile_schema_path", (root / "config/motion_profile.schema.json").string()},
      {"scenario_epoch_unix_ns", 1760000000000000000LL},
      {"limits", {{"max_commands", 1}, {"max_waypoints", 1}, {"max_samples", 32},
          {"max_transitions", 8}, {"max_events", 32}}},
      {"entities", json::array({entity(true, 4), entity(false, 5)})}};
}

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("expected flight-models source directory");
    const std::filesystem::path root(argv[1]);
    auto settings = config(root);

    flight_models::Runtime partitioned(settings);
    const auto initial = partitioned.advance_to(0);
    require(initial.size() == 3, "initial boundary must publish physical rows and configured PRD");
    const auto at100 = partitioned.advance_to(100000000);
    require(at100.size() == 2, "100 ms boundary must publish physical rows only");
    const auto at200 = partitioned.advance_to(200000000);
    require(at200.size() == 3, "200 ms boundary must publish physical rows and configured PRD");
    flight_models::Runtime direct(settings);
    direct.advance_to(0);
    const auto direct200 = direct.advance_to(200000000);
    require(direct200.size() == at100.size() + at200.size(), "partitioning must preserve the sample set");
    for (std::size_t i = 0; i < at200.size(); ++i) {
      require(direct200[i + at100.size()].entity_id == at200[i].entity_id && direct200[i + at100.size()].kind == at200[i].kind &&
          direct200[i + at100.size()].dis == at200[i].dis && direct200[i + at100.size()].prd == at200[i].prd,
          "partitioning must preserve sample values and adapter output");
    }
    bool saw_red_physical = false;
    bool saw_red_report = false;
    bool saw_prd = false;
    for (const auto& output : initial) {
      if (output.entity_id == "red") {
        saw_red_physical = output.kind == flight_models::SampleKind::physical && output.dis.size() == 144 && output.prd.is_null();
        saw_red_report = output.kind == flight_models::SampleKind::navigation;
      }
      if (output.entity_id == "reported" && output.kind == flight_models::SampleKind::navigation) {
        saw_prd = output.prd.contains("PositionReportDetailed");
      }
    }
    require(saw_red_physical && !saw_red_report && saw_prd, "PRD membership and physical DIS output must be independent");
    auto error_settings = settings;
    error_settings["profile_path"] = (root / "config/profiles/synthetic_navigation_error.json").string();
    flight_models::Runtime noisy(error_settings);
    const auto noisy_initial = noisy.advance_to(0);
    require(noisy_initial.size() == initial.size(), "navigation error changed initial event membership");
    bool saw_unchanged_dis = false;
    bool saw_changed_prd = false;
    for (std::size_t i = 0; i < initial.size(); ++i) {
      require(noisy_initial[i].entity_id == initial[i].entity_id && noisy_initial[i].kind == initial[i].kind,
              "navigation error changed initial event ordering");
      if (initial[i].kind == flight_models::SampleKind::physical) {
        require(noisy_initial[i].dis == initial[i].dis, "navigation error changed physical DIS bytes");
        if (initial[i].entity_id == "reported") saw_unchanged_dis = true;
      } else if (initial[i].entity_id == "reported") {
        saw_changed_prd = noisy_initial[i].prd != initial[i].prd;
      }
    }
    require(saw_unchanged_dis && saw_changed_prd, "noise must affect PRD while leaving DIS unchanged");
    const auto noisy_split_a = noisy.advance_to(100000000);
    const auto noisy_split_b = noisy.advance_to(200000000);
    flight_models::Runtime noisy_direct(error_settings);
    noisy_direct.advance_to(0);
    const auto noisy_direct_200 = noisy_direct.advance_to(200000000);
    require(noisy_direct_200.size() == noisy_split_a.size() + noisy_split_b.size(),
            "noise changed navigation event partitioning");
    for (std::size_t i = 0; i < noisy_split_b.size(); ++i) {
      const auto& split = noisy_split_b[i];
      const auto& direct_output = noisy_direct_200[i + noisy_split_a.size()];
      require(direct_output.entity_id == split.entity_id && direct_output.kind == split.kind &&
              direct_output.dis == split.dis && direct_output.prd == split.prd,
              "navigation samples changed when runtime boundaries were partitioned");
    }
    const auto& report = at200.back().prd;
    require(!report.is_null(), "reported entity must emit its scheduled report");
    flight_models::Runtime delayed(settings);
    delayed.advance_to(0);
    const auto delayed_outputs = delayed.advance_to(250000000);
    const auto delayed_report = delayed_outputs.back().prd;
    const auto& data = delayed_report["PositionReportDetailed"]["MessageData"]["PositionReportData"][0];
    const auto& header = delayed_report["PositionReportDetailed"]["MessageHeader"];
    require(data["Kinematics"]["Position"]["AbsolutePoint"]["Timestamp"] != header["Timestamp"],
            "sample validity must retain its source time when created at a later advance boundary");

    bool rejected = false;
    try { auto invalid = settings; invalid["capacity"] = 0; flight_models::Runtime ignored(invalid); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "zero capacity must fail");
    rejected = false;
    try { auto invalid = settings; invalid["profile_path"] = "missing-profile.json"; flight_models::Runtime ignored(invalid); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "missing profile input must fail clearly");
    rejected = false;
    try {
      auto invalid = settings;
      invalid["entities"][0]["commands"] = json::array();
      flight_models::Runtime ignored(invalid);
    } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "unsupported commands must be rejected rather than ignored");
    std::cout << "runtime-test passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "runtime-test: " << error.what() << '\n';
    return 1;
  }
}
