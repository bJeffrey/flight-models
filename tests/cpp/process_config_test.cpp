#include "flight_models/process_config.h"
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace {
using Json = nlohmann::json;
void require(bool condition) { if (!condition) throw std::runtime_error("process config check failed"); }
}
int main(int argc, char** argv) {
  require(argc == 2);
  const auto root = std::filesystem::path(argv[1]);
  std::ifstream input(root / "tests/fixtures/runtime_process.json");
  Json valid; input >> valid;
  const auto path = std::filesystem::temp_directory_path() /
      ("flight-process-config-" + std::to_string(getpid()) + ".json");
  const auto schema = (root / "config/runtime.schema.json").string();
  auto load = [&](const Json& value) {
    { std::ofstream output(path); output << value.dump(); }
    return flight_models::load_process_config(path.string(), schema);
  };
  auto reject = [&](const Json& value) {
    bool rejected = false;
    try { (void)load(value); } catch (const std::invalid_argument&) { rejected = true; }
    require(rejected);
  };
  require(load(valid).at("entities").size() == 3);
  auto bad = valid; bad["scenario_epoch_unix_ns"] = std::numeric_limits<std::uint64_t>::max(); reject(bad);
  bad = valid; bad["scenario_epoch_utc"] = "2026-10-25T00:00:00Z"; reject(bad);
  bad = valid; bad["transport"].erase("pending_byte_limit"); reject(bad);
  bad = valid; bad["transport"]["clock_replay_window"] = 1.5; reject(bad);
  bad = valid; bad["entities"][1]["dis"] = bad["entities"][0]["dis"]; reject(bad);
  bad = valid; bad["entities"][1]["prd"]["platform_system_id"] =
      bad["entities"][0]["prd"]["platform_system_id"]; reject(bad);
  bad = valid; bad["entities"][0]["commands"] = Json::array(); reject(bad);
  bad = valid; bad["entities"][2]["horizontal_speed_mps"] = 1; reject(bad);
  bad = valid; bad["profile_path"] = "profiles/example.json";
  require(load(bad).at("profile_path") == (path.parent_path() / "profiles/example.json").string());
  bad = valid; bad["scenario_epoch_unix_ns"] = std::numeric_limits<std::int64_t>::max();
  bad["scenario_epoch_utc"] = "2262-04-11T23:47:16.854775807Z";
  bool overflowed = false;
  try { (void)load(bad); } catch (const std::overflow_error&) { overflowed = true; }
  require(overflowed);
  std::filesystem::remove(path);
}
