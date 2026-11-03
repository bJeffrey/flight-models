#include "flight_models/checkpoint_state_owner.h"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {
using nlohmann::json;
using namespace checkpoint_runtime;
using namespace flight_models;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
        ("flight-models-checkpoint-owner-" + std::to_string(stamp));
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory() { std::filesystem::remove_all(path_); }
  const std::filesystem::path& path() const noexcept { return path_; }

 private:
  std::filesystem::path path_;
};

void require_same_outputs(const std::vector<RuntimeOutput>& expected,
                          const std::vector<RuntimeOutput>& actual,
                          const char* message) {
  require(expected.size() == actual.size(), message);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    require(expected[i].entity_id == actual[i].entity_id && expected[i].kind == actual[i].kind &&
        expected[i].dis == actual[i].dis && expected[i].prd == actual[i].prd, message);
  }
}

json noisy_config(const std::filesystem::path& root, const std::filesystem::path& temporary) {
  std::ifstream input(root / "tests/fixtures/runtime_process.json");
  if (!input) throw std::runtime_error("could not open runtime process fixture");
  json config;
  input >> config;
  config["profile_schema_path"] = (root / "config/motion_profile.schema.json").string();

  std::ifstream profile_input(root / "config/profiles/synthetic_fixed_wing.json");
  if (!profile_input) throw std::runtime_error("could not open motion profile fixture");
  json profile;
  profile_input >> profile;
  profile["navigation"]["model"] = "bias_gaussian";
  profile["navigation"]["position_sigma_m"] = 0.75;
  profile["navigation"]["velocity_sigma_mps"] = 0.04;
  profile["navigation"]["seed"] = 7241;
  const auto profile_path = temporary / "noisy-profile.json";
  std::ofstream profile_output(profile_path);
  profile_output << profile.dump(2);
  profile_output.close();
  config["profile_path"] = profile_path.string();
  return config;
}

Request make_request(const std::string& operation, const std::string& checkpoint,
                     OperationKind kind, Phase phase, const ClockBoundary& source,
                     std::uint32_t current_epoch, std::uint32_t target_epoch,
                     std::string manifest = {}) {
  return {operation, checkpoint, kind, phase, source, current_epoch, target_epoch,
      {"flight-models"}, {}, std::move(manifest)};
}

void capture(CheckpointParticipant& participant, const ClockBoundary& source) {
  const auto fence = make_request("capture-200ms", "flight-state", OperationKind::capture,
      Phase::fence, source, source.epoch, source.epoch);
  require(participant.handle_request(fence).success, "capture fence failed");
  const auto snapshot = make_request("capture-200ms", "flight-state", OperationKind::capture,
      Phase::snapshot, source, source.epoch, source.epoch);
  const auto saved = participant.handle_request(snapshot);
  require(saved.success && saved.payload_size > 0, "flight runtime checkpoint was not saved");
  const auto release = make_request("capture-200ms", "flight-state", OperationKind::capture,
      Phase::release, source, source.epoch, source.epoch);
  require(participant.handle_request(release).success, "capture release failed");
}

void require_rejected_payload(CheckpointStateOwner& owner, const std::string& payload,
                              std::uint32_t target_epoch, const std::string& reason) {
  bool rejected = false;
  std::string actual_reason;
  try {
    static_cast<void>(owner.prepare_restore(payload, target_epoch));
  } catch (const std::invalid_argument& error) {
    actual_reason = error.what();
    rejected = actual_reason.find(reason) != std::string::npos;
  }
  if (!rejected) {
    const std::string detail = actual_reason.empty()
        ? "payload was accepted"
        : "unexpected rejection: " + actual_reason;
    throw std::runtime_error("continuation expected rejection containing '" + reason + "', " + detail);
  }
}

void run(const std::filesystem::path& root) {
  TemporaryDirectory temporary;
  const json config = noisy_config(root, temporary.path());
  std::vector<std::vector<RuntimeOutput>> published;
  CheckpointStateOwner owner(config, [&published](const auto& outputs) { published.push_back(outputs); });
  require(!owner.initialized() && owner.current_timeline_epoch() == 0 && owner.current_t_us() == 0,
          "owner did not begin at the unannounced epoch-zero boundary");
  require(published.empty(), "constructor published time-zero output before authority admission");

  CheckpointStore store(temporary.path() / "checkpoint-store");
  ServiceIdentity service{"flight-models", "flight-models.runtime-checkpoint", "1",
      "flight-models-test", "runtime-process-fixture"};
  const std::string run_id = config.at("run_id").get<std::string>();
  const std::string origin = config.at("scenario_epoch_utc").get<std::string>();
  constexpr std::uint64_t interval_us = 100'000;
  ClockBoundary initial{run_id, 0, 0, 0, interval_us, origin};
  CheckpointParticipant participant(owner, store, initial, service, {4, 16'000'000, 16'000'000, 8});
  participant.start_session();

  // SNAPSHOT itself proves that unannounced initial state cannot be captured.
  const auto early_fence = make_request("early", "early-state", OperationKind::capture,
      Phase::fence, initial, 0, 0);
  const auto early_snapshot = make_request("early", "early-state", OperationKind::capture,
      Phase::snapshot, initial, 0, 0);
  const auto early_abort = make_request("early", "early-state", OperationKind::capture,
      Phase::abort, initial, 0, 0);
  require(participant.handle_request(early_fence).success, "early capture fence failed");
  require(!participant.handle_request(early_snapshot).success,
          "capture was accepted before authoritative time-zero publication");
  require(participant.handle_request(early_abort).success, "early capture could not safely abort");
  require(published.empty(), "rejected preinitial capture published data");

  owner.publish_initial();
  require(owner.initialized() && !published.empty(), "authoritative time zero did not publish initial samples");
  require(std::any_of(published.front().begin(), published.front().end(), [](const RuntimeOutput& output) {
    return output.kind == SampleKind::navigation && !output.prd.is_null();
  }), "noisy fixture did not emit a PRD sample");
  const std::size_t initial_batch_count = published.size();
  bool repeated_initial_rejected = false;
  try {
    owner.publish_initial();
  } catch (const std::logic_error&) {
    repeated_initial_rejected = true;
  }
  require(repeated_initial_rejected && published.size() == initial_batch_count,
          "initial samples were published more than once");

  const ClockBoundary at100{run_id, 0, 1, interval_us, interval_us, origin};
  const ClockBoundary at200{run_id, 0, 2, interval_us * 2, interval_us, origin};
  require(participant.handle_tick({at100, false, true}), "100 ms owner advancement failed");
  require(participant.handle_tick({at200, false, true}), "200 ms owner advancement failed");
  const ClockBoundary source = participant.clock();
  capture(participant, source);
  const std::string saved_payload = owner.capture_state();

  const ClockBoundary at300{run_id, 0, 3, interval_us * 3, interval_us, origin};
  const ClockBoundary at400{run_id, 0, 4, interval_us * 4, interval_us, origin};
  require(participant.handle_tick({at300, false, true}), "300 ms owner advancement failed");
  const std::vector<RuntimeOutput> expected300 = published.back();
  require(participant.handle_tick({at400, false, true}), "400 ms owner advancement failed");
  const std::vector<RuntimeOutput> expected400 = published.back();

  std::vector<std::vector<RuntimeOutput>> restart_publication;
  CheckpointStateOwner restart_owner(config, [&restart_publication](const auto& outputs) {
    restart_publication.push_back(outputs);
  });
  auto restart_candidate = restart_owner.prepare_restore(saved_payload, 1);
  restart_owner.apply_restore(std::move(restart_candidate));
  require(restart_owner.initialized() && restart_owner.current_t_us() == 200'000 &&
          restart_owner.current_timeline_epoch() == 1 && restart_publication.empty(),
          "restart recovery did not install before publishing a fresh time-zero sample");

  const json encoded = json::parse(saved_payload);
  auto submicro = encoded;
  submicro["driver"]["time_ns"] = 200'000'001;
  for (auto& row : submicro["driver"]["rows"]) row["physical"]["source_time_ns"] = 200'000'001;
  require_rejected_payload(owner, submicro.dump(), 1, "whole-microsecond");
  auto beyond_end = encoded;
  beyond_end["driver"]["time_ns"] = 400'001'000;
  for (auto& row : beyond_end["driver"]["rows"]) row["physical"]["source_time_ns"] = 400'001'000;
  require_rejected_payload(owner, beyond_end.dump(), 1, "end_time_ns");
  require(owner.current_t_us() == 400'000 && owner.current_timeline_epoch() == 0 &&
          published.size() == initial_batch_count + 4,
          "invalid restore candidates changed live time, epoch, or output publication");
  bool stale_epoch_rejected = false;
  try {
    static_cast<void>(owner.prepare_restore(saved_payload, 0));
  } catch (const std::invalid_argument&) {
    stale_epoch_rejected = true;
  }
  require(stale_epoch_rejected, "restore candidate accepted a non-new epoch");

  const std::size_t output_count_before_prepare = published.size();
  const auto prepare = make_request("restore-1", "flight-state", OperationKind::restore,
      Phase::prepare_restore, source, 0, 1, "checkpoint://flight-models/flight-state/manifest.json");
  const Response prepared = participant.handle_request(prepare);
  require(prepared.success && owner.current_t_us() == 400'000 && owner.current_timeline_epoch() == 0 &&
          published.size() == output_count_before_prepare,
          "restore preparation mutated or published live state");
  const auto apply = make_request("restore-1", "flight-state", OperationKind::restore,
      Phase::apply_restore, source, 0, 1, prepare.manifest_ref);
  require(participant.handle_request(apply).success, "prepared Runtime candidate did not install");
  require(owner.current_t_us() == 200'000 && owner.current_timeline_epoch() == 1 &&
          published.size() == output_count_before_prepare,
          "restore apply changed output publication or restored boundary");
  const ClockBoundary restored{run_id, 1, 0, 200'000, interval_us, origin};
  require(!participant.handle_tick({restored, true, false}), "paused restored announcement performed work");
  const auto release = make_request("restore-1", "flight-state", OperationKind::restore,
      Phase::release, source, 0, 1, prepare.manifest_ref);
  require(participant.handle_request(release).success, "restored Runtime could not be released");

  const ClockBoundary resumed300{run_id, 1, 1, 300'000, interval_us, origin};
  const ClockBoundary resumed400{run_id, 1, 2, 400'000, interval_us, origin};
  require(participant.handle_tick({resumed300, false, true}), "restored Runtime did not advance to 300 ms");
  require_same_outputs(expected300, published.back(), "restored noisy output changed at 300 ms");
  require(participant.handle_tick({resumed400, false, true}), "restored Runtime did not advance to 400 ms");
  require_same_outputs(expected400, published.back(), "restored noisy output changed at 400 ms");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("expected flight-models source directory");
    run(argv[1]);
    std::cout << "checkpoint Runtime StateOwner passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "checkpoint Runtime StateOwner: " << error.what() << '\n';
    return 1;
  }
}
