/** @file Bounded process-level clock, DIS, and PRD transport smoke check. */
#include "system_owp_probe.h"
#include "sensor_models/dis/entity_state_decoder.hpp"
#include "flight_models/runtime.h"
#include "runtime_control.pb.h"
#include "checkpoint_runtime/store.hpp"
#include <algorithm>
#include "sim_time.pb.h"
#include <nats/nats.h>
#include <boost/asio.hpp>
#include <chrono>
#include <array>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <spawn.h>
#include <fstream>
#include <nlohmann/json.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

extern char** environ;

namespace {
using namespace std::chrono_literals;
void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
struct Nats {
  natsConnection* connection{};
  ~Nats() { if (connection) natsConnection_Destroy(connection); }
};
struct Child {
  pid_t pid{-1};
  ~Child() { stop(); }
  // posix_spawn avoids running non-signal-safe C++/stdio after fork while the
  // ASB observer threads are active during the second process launch.
  void launch(const std::string& runner, const std::string& config,
              const std::string& schema, const std::string& log) {
    posix_spawn_file_actions_t actions;
    int status = posix_spawn_file_actions_init(&actions);
    require(status == 0, "cannot initialize child file actions");
    status = posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log.c_str(),
                                             O_CREAT | O_WRONLY | O_TRUNC, 0644);
    if (status == 0) status = posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    if (status == 0) {
      // POSIX does not mutate these argument strings despite its char* API.
      char* args[] = {const_cast<char*>(runner.c_str()), const_cast<char*>(config.c_str()),
                      const_cast<char*>(schema.c_str()), nullptr};
      status = posix_spawn(&pid, runner.c_str(), &actions, nullptr, args, environ);
    }
    posix_spawn_file_actions_destroy(&actions);
    require(status == 0, "cannot launch child process: " + std::string(std::strerror(status)));
  }
  int wait_for_exit(std::chrono::milliseconds timeout) {
    int status{};
    const auto end = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < end) {
      if (waitpid(pid, &status, WNOHANG) == pid) { pid = -1; return status; }
      std::this_thread::sleep_for(20ms);
    }
    throw std::runtime_error("child did not exit before its configured operation deadline");
  }
  void stop() {
    if (pid <= 0) return;
    kill(pid, SIGTERM);
    int status{};
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < end) {
      if (waitpid(pid, &status, WNOHANG) == pid) { pid = -1; return; }
      std::this_thread::sleep_for(20ms);
    }
    kill(pid, SIGKILL);
    waitpid(pid, &status, 0);
    pid = -1;
  }
};
struct IoRunner {
  boost::asio::io_context io;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work{
      boost::asio::make_work_guard(io)};
  std::thread thread{[this] { io.run(); }};
  ~IoRunner() { work.reset(); io.stop(); if (thread.joinable()) thread.join(); }
};
std::string read_log(const std::string& path) {
  std::ifstream in(path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
void publish_tick(natsConnection* nc, const std::string& run, std::uint64_t id,
                  std::uint64_t us, std::uint64_t dt, bool advances, bool paused,
                  std::uint32_t epoch = 0) {
  rlbm::proto::SimTimeTick tick;
  tick.set_run_id(run); tick.set_tick_id(id); tick.set_t_us(us); tick.set_dt_us(dt);
  tick.set_rate(1.0); tick.set_advances_time(advances); tick.set_paused(paused);
  tick.set_timeline_epoch(epoch); tick.set_scenario_epoch_utc("2026-10-24T00:00:00Z");
  tick.set_mode(rlbm::proto::MODE_CONSTRUCTIVE);
  std::string bytes;
  require(tick.SerializeToString(&bytes), "cannot serialize SimTimeTick");
  const auto status = natsConnection_Publish(nc, "sim.time.tick", bytes.data(), static_cast<int>(bytes.size()));
  require(status == NATS_OK, std::string("NATS tick publish: ") + natsStatus_GetText(status));
  natsConnection_FlushTimeout(nc, 1000);
}

struct ResponseSubscription {
  natsSubscription* subscription{};
  ~ResponseSubscription() { if (subscription) natsSubscription_Destroy(subscription); }
};

rlbm::proto::ServiceCheckpointRequest checkpoint_request(
    const std::string& operation, rlbm::proto::CheckpointOperationKind kind,
    rlbm::proto::CheckpointPhase phase, std::uint64_t tick, std::uint64_t time_us,
    std::uint32_t current_epoch, std::uint32_t target_epoch) {
  rlbm::proto::ServiceCheckpointRequest request;
  request.set_operation_id(operation);
  request.set_checkpoint_id("motion-200");
  request.set_kind(kind);
  request.set_phase(phase);
  request.set_current_epoch(current_epoch);
  request.set_target_epoch(target_epoch);
  request.add_required_service_ids("flight-models");
  auto* source = request.mutable_source();
  source->set_run_id("flight-process-smoke");
  source->set_source_epoch(0);
  source->set_tick_id(tick);
  source->set_t_us(time_us);
  source->set_dt_us(100000);
  source->set_scenario_epoch_utc("2026-10-24T00:00:00Z");
  if (kind != rlbm::proto::CHECKPOINT_OPERATION_CAPTURE) {
    request.set_manifest_ref("checkpoint://flight-models/motion-200/manifest.json");
  }
  return request;
}

rlbm::proto::ServiceCheckpointResponse exchange_checkpoint(
    natsConnection* connection, natsSubscription* subscription,
    const rlbm::proto::ServiceCheckpointRequest& request, int timeout_ms) {
  std::string bytes;
  require(request.SerializeToString(&bytes), "cannot encode checkpoint request");
  require(natsConnection_Publish(connection, "sim.checkpoint.request", bytes.data(),
          static_cast<int>(bytes.size())) == NATS_OK, "checkpoint request publication failed");
  require(natsConnection_FlushTimeout(connection, timeout_ms) == NATS_OK, "checkpoint request flush failed");
  natsMsg* received = nullptr;
  require(natsSubscription_NextMsg(&received, subscription, timeout_ms) == NATS_OK,
          "checkpoint acknowledgement timed out");
  const std::unique_ptr<natsMsg, decltype(&natsMsg_Destroy)> guard(received, &natsMsg_Destroy);
  rlbm::proto::ServiceCheckpointResponse response;
  require(response.ParseFromArray(natsMsg_GetData(received), natsMsg_GetDataLength(received)),
          "cannot decode checkpoint acknowledgement");
  require(response.operation_id() == request.operation_id() && response.phase() == request.phase() &&
          response.kind() == request.kind() && response.current_epoch() == request.current_epoch() &&
          response.target_epoch() == request.target_epoch() &&
          response.source().SerializeAsString() == request.source().SerializeAsString(),
          "checkpoint acknowledgement correlation changed");
  require(response.service_id() == "flight-models" && response.software_id() == "native-process-test" &&
          response.config_id() == "native-process-fixture" &&
          response.state_schema() == "flight-models.runtime-checkpoint" && response.state_version() == "1",
          "checkpoint acknowledgement omitted exact service identity");
  return response;
}

}

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: runtime-process-smoke <runner> <config> <schema> <artifact-dir>\n";
    return 2;
  }
  Child child;
  try {
    const std::string runner = argv[1], config = argv[2], schema = argv[3], artifacts = argv[4];
    const auto log = artifacts + "/flight-process.log";
    child.launch(runner, config, schema, log);

    // Build exact expected PRD payloads through the same public domain runtime.
    nlohmann::json fixture;
    { std::ifstream input(config); input >> fixture; }
    const auto operation_timeout = std::chrono::milliseconds(
        fixture.at("transport").at("operation_timeout_ms").get<int>());
    flight_models::Runtime reference(fixture);
    std::vector<std::string> expected_prds;
    auto collect = [&](std::int64_t ns, std::int64_t creation_ns) {
      for (const auto& item : reference.advance_to(ns, creation_ns))
        if (!item.prd.is_null()) expected_prds.push_back(item.prd.dump());
    };
    collect(0, 0); collect(100000000, 100000000); collect(200000000, 200000000);
    collect(400000000, 450000000);
    require(expected_prds.size() == 6, "fixture did not produce six reference PRDs");

    IoRunner io_runner;
    const auto probe_timeout = static_cast<int>(operation_timeout.count());
    auto a = std::make_shared<comms_models::test::SystemOwpProbe>(io_runner.io, "ws://asb-1:9000/owp", "contract-test", "002.5.0", probe_timeout);
    auto b = std::make_shared<comms_models::test::SystemOwpProbe>(io_runner.io, "ws://asb-2:9000/owp", "contract-test", "002.5.0", probe_timeout);
    a->start(); b->start();
    require(a->wait_ready() && b->wait_ready(), "ASB INIT failed");
    require(a->subscribe("PositionReportDetailed", "contract.prd"), "ASB 1 SUB failed");
    require(b->subscribe("PositionReportDetailed", "contract.prd"), "ASB 2 SUB failed");

    boost::asio::io_context udp_io;
    boost::asio::ip::udp::socket udp(udp_io, {boost::asio::ip::udp::v4(), 3000});
    udp.non_blocking(true);

    natsOptions* options{};
    natsOptions_Create(&options);
    natsOptions_SetURL(options, "nats://nats:4222");
    Nats nc;
    auto status = natsConnection_Connect(&nc.connection, options);
    natsOptions_Destroy(options);
    require(status == NATS_OK, "NATS connection failed");

    const auto ready_deadline = std::chrono::steady_clock::now() + operation_timeout + 2s;
    while (std::chrono::steady_clock::now() < ready_deadline && read_log(log).find("flight-models ready run=") == std::string::npos)
      std::this_thread::sleep_for(20ms);
    require(read_log(log).find("flight-models ready run=flight-process-smoke") != std::string::npos,
            "process did not become ready; see " + log);
    const std::string run = "flight-process-smoke";
    publish_tick(nc.connection, run, 0, 0, 0, false, true);
    publish_tick(nc.connection, run, 1, 100000, 100000, true, false);
    publish_tick(nc.connection, run, 2, 200000, 100000, true, false);
    publish_tick(nc.connection, run, 2, 200000, 100000, true, false); // replay is ignored
    publish_tick(nc.connection, run, 2, 200000, 0, false, true); // paused heartbeat retains the last tick
    publish_tick(nc.connection, run, 3, 450000, 250000, true, false); // clipped at configured 400 ms

    std::size_t datagrams = 0;
    std::array<std::size_t, 3> entity_counts{};
    std::map<std::uint32_t, std::array<std::size_t, 3>> timestamp_counts;
    const auto udp_deadline = std::chrono::steady_clock::now() + 3s;
    while (datagrams < 15 && std::chrono::steady_clock::now() < udp_deadline) {
      std::uint8_t packet[2048];
      boost::system::error_code error;
      boost::asio::ip::udp::endpoint sender;
      const auto size = udp.receive_from(boost::asio::buffer(packet), sender, 0, error);
      if (error == boost::asio::error::would_block || error == boost::asio::error::try_again) {
        std::this_thread::sleep_for(5ms);
        continue;
      }
      require(!error, "DIS UDP receive failed: " + error.message());
      const auto decoded = sensor_models::dis::decode_entity_state_pdu(packet, size);
      require(decoded.protocol_version == 7, "received DIS protocol version changed");
      require(decoded.identity.entity >= 1 && decoded.identity.entity <= 3,
              "received unexpected DIS entity identity");
      require(decoded.identity.exercise == 1 && decoded.identity.site == 2 &&
                  decoded.identity.application == 3,
              "received DIS exercise/site/application identity changed");
      const auto entity_index = static_cast<std::size_t>(decoded.identity.entity - 1);
      const auto expected_force = decoded.identity.entity == 3 ? 2 : 1;
      require(decoded.force_id == expected_force, "received DIS force identity changed");
      ++entity_counts[entity_index];
      ++timestamp_counts[decoded.dis_timestamp][entity_index];
      if (decoded.identity.entity == 3) {
        require(decoded.dead_reckoning_algorithm == 1, "fixed entity dead-reckoning mode changed");
        require(decoded.acceleration_ecef_mps2.x == 0.0 && decoded.acceleration_ecef_mps2.y == 0.0 &&
                    decoded.acceleration_ecef_mps2.z == 0.0 && decoded.angular_velocity_radps.x == 0.0 &&
                    decoded.angular_velocity_radps.y == 0.0 && decoded.angular_velocity_radps.z == 0.0,
                "fixed entity has nonzero dead-reckoning derivatives");
      }
      ++datagrams;
    }
    require(datagrams == 15, "expected 15 decodable DIS datagrams, got " + std::to_string(datagrams));
    require(entity_counts == std::array<std::size_t, 3>{5, 5, 5}, "DIS entity multiplicity changed");
    require(timestamp_counts.size() == 5, "DIS output did not contain the five expected sample times");
    for (const auto& [timestamp, counts] : timestamp_counts) {
      (void)timestamp;
      require(counts == std::array<std::size_t, 3>{1, 1, 1},
              "each DIS timestamp must contain exactly one PDU per entity");
    }

    // Publications are validated through the existing ASB observer, with one PRD per platform.
    for (std::size_t i = 0; i < expected_prds.size(); i += 2) {
      require(a->expect("contract.prd", expected_prds[i], true, 3000), "ASB 1 PRD differed from expected payload");
      require(b->expect("contract.prd", expected_prds[i + 1], true, 3000), "ASB 2 PRD differed from expected payload");
    }
    require(a->expect("contract.prd", "", false, 200) && b->expect("contract.prd", "", false, 200),
            "an ASB received an unexpected PRD count");
    for (const auto& payload : {expected_prds[4], expected_prds[5]}) {
      const auto report = nlohmann::json::parse(payload).at("PositionReportDetailed");
      require(report.at("MessageHeader").at("Timestamp") == "2026-10-24T00:00:00.450000000Z",
              "final PRD creation timestamp must retain announced 450 ms");
      require(report.at("MessageData").at("PositionReportData").at(0).at("Kinematics")
                  .at("Position").at("AbsolutePoint").at("Timestamp") ==
                  "2026-10-24T00:00:00.400000000Z",
              "final PRD validity timestamp must remain clipped at 400 ms");
    }
    const auto process_status = child.wait_for_exit(operation_timeout + 2s);
    require(WIFEXITED(process_status) && WEXITSTATUS(process_status) == 0,
            "normal flight process did not exit successfully after the final tick");
    require(read_log(log).find("flight-models stopped dis=15 prd=6") != std::string::npos,
            "process output counts differ from expected 15 DIS / 6 PRD");
    a->stop(); b->stop();

    // A disallowed topic must fail publication at Sleet without stalling the clock.
    fixture["entities"][0]["prd"]["topic"] = "forbidden.prd";
    const auto negative_config = artifacts + "/runtime-forbidden-topic.json";
    { std::ofstream file(negative_config); file << fixture.dump(2); }
    const auto negative_log = artifacts + "/flight-process-forbidden-topic.log";
    child.launch(runner, negative_config, schema, negative_log);
    const auto negative_ready = std::chrono::steady_clock::now() + operation_timeout + 2s;
    while (std::chrono::steady_clock::now() < negative_ready &&
           read_log(negative_log).find("flight-models ready run=") == std::string::npos)
      std::this_thread::sleep_for(20ms);
    require(read_log(negative_log).find("flight-models ready run=flight-process-smoke") != std::string::npos,
            "negative process did not become ready");
    publish_tick(nc.connection, run, 0, 0, 0, false, true);
    const int negative_status = child.wait_for_exit(operation_timeout + 1s);
    require(WIFEXITED(negative_status) && WEXITSTATUS(negative_status) == 1,
            "forbidden-topic publication did not return a bounded process failure");
    require(read_log(negative_log).find("OWP PUB") != std::string::npos,
            "forbidden-topic failure was not surfaced");
    // Reuse the real NATS and platform ASBs for native checkpoint/epoch control.
    // Remove any truth emitted before the deliberately failed prior publication.
    for (;;) {
      std::array<std::uint8_t, 2048> discarded{};
      boost::system::error_code error;
      udp.receive(boost::asio::buffer(discarded), 0, error);
      if (error == boost::asio::error::would_block || error == boost::asio::error::try_again) break;
      require(!error, "cannot drain prior test truth");
    }
    fixture["entities"][0]["prd"]["topic"] = "contract.prd";
    fixture["profile_path"] = "/src/config/profiles/synthetic_navigation_error.json";
    { std::ifstream profile("tests/fixtures/checkpoint_profile.json"); profile >> fixture["checkpoint"]; }
    fixture["checkpoint"]["run_directory"] = artifacts + "/checkpoint-state";
    const auto checkpoint_config = artifacts + "/runtime-checkpoint.json";
    { std::ofstream file(checkpoint_config); file << fixture.dump(2); }
    ResponseSubscription responses;
    require(natsConnection_SubscribeSync(&responses.subscription, nc.connection, "sim.checkpoint.response") == NATS_OK,
            "cannot subscribe checkpoint responses");
    require(natsConnection_FlushTimeout(nc.connection, probe_timeout) == NATS_OK,
            "checkpoint response subscription was not admitted");
    auto checkpoint_a = std::make_shared<comms_models::test::SystemOwpProbe>(
        io_runner.io, "ws://asb-1:9000/owp", "contract-test", "002.5.0", probe_timeout);
    auto checkpoint_b = std::make_shared<comms_models::test::SystemOwpProbe>(
        io_runner.io, "ws://asb-2:9000/owp", "contract-test", "002.5.0", probe_timeout);
    checkpoint_a->start();
    checkpoint_b->start();
    require(checkpoint_a->wait_ready() && checkpoint_b->wait_ready(), "checkpoint observers did not initialize");
    require(checkpoint_a->subscribe("PositionReportDetailed", "contract.prd") &&
            checkpoint_b->subscribe("PositionReportDetailed", "contract.prd"), "checkpoint PRD subscription failed");
    auto launch_checkpoint = [&](const std::string& process_log, bool recovery) {
      child.launch(runner, checkpoint_config, schema, process_log);
      const auto deadline = std::chrono::steady_clock::now() + operation_timeout + 2s;
      while (std::chrono::steady_clock::now() < deadline &&
             read_log(process_log).find("checkpoint=flight-models") == std::string::npos) {
        std::this_thread::sleep_for(20ms);
      }
      require(read_log(process_log).find(recovery ? "recovery=1" : "recovery=0") != std::string::npos,
              "checkpoint process startup/session state differs; see " + process_log);
    };
    auto exchange = [&](const auto& request, bool success) {
      const auto response = exchange_checkpoint(nc.connection, responses.subscription, request, probe_timeout);
      require(response.success() == success, "checkpoint phase outcome differed: " + response.failure_reason());
      return response;
    };
    std::vector<std::vector<std::uint8_t>> expected_dis;
    std::vector<std::string> checkpoint_prds;
    auto collect_checkpoint = [&](const auto& outputs) {
      for (const auto& output : outputs) {
        if (!output.dis.empty()) expected_dis.push_back(output.dis);
        if (!output.prd.is_null()) checkpoint_prds.push_back(output.prd.dump());
      }
    };
    flight_models::Runtime checkpoint_reference(fixture);
    collect_checkpoint(checkpoint_reference.advance_to(0));
    collect_checkpoint(checkpoint_reference.advance_to(100000000));
    collect_checkpoint(checkpoint_reference.advance_to(200000000));
    const auto saved_payload = checkpoint_reference.encode_checkpoint();
    collect_checkpoint(checkpoint_reference.advance_to(300000000));
    checkpoint_reference = checkpoint_reference.prepare_restore(saved_payload);
    collect_checkpoint(checkpoint_reference.advance_to(300000000));
    collect_checkpoint(checkpoint_reference.advance_to(400000000));

    const auto checkpoint_log = artifacts + "/flight-process-checkpoint.log";
    launch_checkpoint(checkpoint_log, false);
    exchange(checkpoint_request("early", rlbm::proto::CHECKPOINT_OPERATION_CAPTURE,
             rlbm::proto::CHECKPOINT_PHASE_FENCE, 0, 0, 0, 0), false);
    publish_tick(nc.connection, run, 0, 0, 0, false, true);
    publish_tick(nc.connection, run, 1, 100000, 100000, true, false);
    publish_tick(nc.connection, run, 2, 200000, 100000, true, false);
    auto capture_request = checkpoint_request("capture", rlbm::proto::CHECKPOINT_OPERATION_CAPTURE,
        rlbm::proto::CHECKPOINT_PHASE_FENCE, 2, 200000, 0, 0);
    exchange(capture_request, true);
    capture_request.set_phase(rlbm::proto::CHECKPOINT_PHASE_SNAPSHOT);
    const auto captured = exchange(capture_request, true);
    require(captured.payload_size() > 0 && captured.payload_sha256().size() == 64 &&
            captured.bundle_size() > captured.payload_size(), "capture omitted opaque storage metadata");
    require(exchange(capture_request, true).SerializeAsString() == captured.SerializeAsString(),
            "duplicate snapshot did not return its original acknowledgement");
    capture_request.set_phase(rlbm::proto::CHECKPOINT_PHASE_RELEASE);
    exchange(capture_request, true);
    publish_tick(nc.connection, run, 3, 300000, 100000, true, false);
    auto restore_request = checkpoint_request("restore", rlbm::proto::CHECKPOINT_OPERATION_RESTORE,
        rlbm::proto::CHECKPOINT_PHASE_PREPARE_RESTORE, 2, 200000, 0, 1);
    exchange(restore_request, true);
    restore_request.set_phase(rlbm::proto::CHECKPOINT_PHASE_APPLY_RESTORE);
    exchange(restore_request, true);
    restore_request.set_phase(rlbm::proto::CHECKPOINT_PHASE_RELEASE);
    exchange(restore_request, false); // Clock commit has not yet been announced.
    publish_tick(nc.connection, run, 0, 200000, 100000, false, true, 1);
    publish_tick(nc.connection, run, 4, 400000, 100000, true, false, 0); // Fenced old epoch.
    exchange(restore_request, true);
    publish_tick(nc.connection, run, 1, 300000, 100000, true, false, 1);
    publish_tick(nc.connection, run, 2, 400000, 100000, true, false, 1);

    auto verify_checkpoint_outputs = [&](const std::vector<std::vector<std::uint8_t>>& physical,
                                         const std::vector<std::string>& navigation,
                                         const std::string& process_log) {
      std::vector<std::vector<std::uint8_t>> received;
      const auto deadline = std::chrono::steady_clock::now() + operation_timeout;
      while (received.size() < physical.size() && std::chrono::steady_clock::now() < deadline) {
        std::array<std::uint8_t, 2048> packet{};
        boost::system::error_code error;
        const auto count = udp.receive(boost::asio::buffer(packet), 0, error);
        if (error == boost::asio::error::would_block || error == boost::asio::error::try_again) {
          std::this_thread::sleep_for(5ms);
          continue;
        }
        require(!error, "checkpoint truth receive failed");
        received.emplace_back(packet.begin(), packet.begin() + count);
      }
      auto expected = physical;
      std::sort(expected.begin(), expected.end());
      std::sort(received.begin(), received.end());
      require(received == expected, "checkpoint continuation changed physical DIS bytes or multiplicity");
      for (std::size_t index = 0; index < navigation.size(); index += 2) {
        require(checkpoint_a->expect("contract.prd", navigation[index], true, probe_timeout) &&
                checkpoint_b->expect("contract.prd", navigation[index + 1], true, probe_timeout),
                "checkpoint continuation changed noisy PRDs or repeated captured outputs");
      }
      require(checkpoint_a->expect("contract.prd", "", false, 100) &&
              checkpoint_b->expect("contract.prd", "", false, 100), "unexpected checkpoint PRD publication");
      const auto status = child.wait_for_exit(operation_timeout + 2s);
      require(WIFEXITED(status) && WEXITSTATUS(status) == 0, "checkpoint process did not exit normally");
      const std::string count_line = "flight-models stopped dis=" + std::to_string(physical.size()) +
          " prd=" + std::to_string(navigation.size());
      require(read_log(process_log).find(count_line) != std::string::npos,
              "checkpoint process published unexpected extra outputs");
    };
    verify_checkpoint_outputs(expected_dis, checkpoint_prds, checkpoint_log);

    // Restart from the same durable session: no fresh zero-time output may escape.
    const auto recovery_log = artifacts + "/flight-process-checkpoint-recovery.log";
    launch_checkpoint(recovery_log, true);
    publish_tick(nc.connection, run, 0, 0, 0, false, true);
    require(checkpoint_a->expect("contract.prd", "", false, 100) &&
            checkpoint_b->expect("contract.prd", "", false, 100), "restart leaked fresh initial PRDs");
    auto recovery_request = checkpoint_request("recover", rlbm::proto::CHECKPOINT_OPERATION_RECOVER,
        rlbm::proto::CHECKPOINT_PHASE_PREPARE_RESTORE, 2, 200000, 1, 2);
    exchange(recovery_request, true);
    recovery_request.set_phase(rlbm::proto::CHECKPOINT_PHASE_APPLY_RESTORE);
    exchange(recovery_request, true);
    publish_tick(nc.connection, run, 0, 200000, 100000, false, true, 2);
    recovery_request.set_phase(rlbm::proto::CHECKPOINT_PHASE_RELEASE);
    exchange(recovery_request, true);
    publish_tick(nc.connection, run, 1, 300000, 100000, true, false, 2);
    publish_tick(nc.connection, run, 2, 400000, 100000, true, false, 2);
    expected_dis.clear();
    checkpoint_prds.clear();
    flight_models::Runtime recovered_reference(fixture);
    recovered_reference = recovered_reference.prepare_restore(saved_payload);
    collect_checkpoint(recovered_reference.advance_to(300000000));
    collect_checkpoint(recovered_reference.advance_to(400000000));
    verify_checkpoint_outputs(expected_dis, checkpoint_prds, recovery_log);
    checkpoint_a->stop();
    checkpoint_b->stop();
    nlohmann::json checkpoint_result;
    checkpoint_result["accepted"] = true;
    checkpoint_result["capture_elapsed_us"] = 200000;
    checkpoint_result["restore_epochs"] = {1, 2};
    checkpoint_result["cold_restart_initial_output_fenced"] = true;
    checkpoint_result["noisy_prd_and_dis_continuation_exact"] = true;
    { std::ofstream file(artifacts + "/checkpoint-result.json"); file << checkpoint_result.dump(2) << '\n'; }

    std::cout << "runtime process smoke passed; log=" << log << '\n';
    return 0;
  } catch (const std::exception& error) {
    child.stop();
    std::cerr << "runtime-process-smoke: " << error.what() << '\n';
    return 1;
  }
}
