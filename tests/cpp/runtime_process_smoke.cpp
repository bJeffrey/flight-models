/** @file Bounded process-level clock, DIS, and PRD transport smoke check. */
#include "system_owp_probe.h"
#include "sensor_models/dis/entity_state_decoder.hpp"
#include "flight_models/runtime.h"
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
                  std::uint64_t us, std::uint64_t dt, bool advances, bool paused) {
  rlbm::proto::SimTimeTick tick;
  tick.set_run_id(run); tick.set_tick_id(id); tick.set_t_us(us); tick.set_dt_us(dt);
  tick.set_rate(1.0); tick.set_advances_time(advances); tick.set_paused(paused);
  tick.set_timeline_epoch(0); tick.set_scenario_epoch_utc("2026-10-24T00:00:00Z");
  tick.set_mode(rlbm::proto::MODE_CONSTRUCTIVE);
  std::string bytes;
  require(tick.SerializeToString(&bytes), "cannot serialize SimTimeTick");
  const auto status = natsConnection_Publish(nc, "sim.time.tick", bytes.data(), static_cast<int>(bytes.size()));
  require(status == NATS_OK, std::string("NATS tick publish: ") + natsStatus_GetText(status));
  natsConnection_FlushTimeout(nc, 1000);
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
    std::cout << "runtime process smoke passed; log=" << log << '\n';
    return 0;
  } catch (const std::exception& error) {
    child.stop();
    std::cerr << "runtime-process-smoke: " << error.what() << '\n';
    return 1;
  }
}
