/** Fresh-run process: authoritative clock -> domain runtime -> direct transports. */
#include "flight_models/process_config.h"
#include "flight_models/checkpoint_process.h"
#include "flight_models/runtime.h"
#include "flight_models/runtime_transport.h"
#include "runtime_support/simulation_clock.hpp"
#include <csignal>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
volatile std::sig_atomic_t stopping = 0;
void request_stop(int) { stopping = 1; }
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: flight-models-run <runtime.json> <runtime.schema.json>\n";
    return 2;
  }
  try {
    const auto config = flight_models::load_process_config(argv[1], argv[2]);
    if (config.contains("checkpoint")) return flight_models::run_checkpoint_process(config);
    flight_models::Runtime runtime(config);
    // Validate all initial adapters before opening sockets. Hold these outputs
    // until the authority announces time zero; startup itself publishes no truth.
    auto initial = runtime.advance_to(0);
    const auto epoch_ns = config.at("scenario_epoch_unix_ns").get<std::int64_t>();
    const auto maximum_elapsed_ns = epoch_ns > 0 ?
        std::numeric_limits<std::int64_t>::max() - epoch_ns : std::numeric_limits<std::int64_t>::max();
    runtime_support::SimulationClock clock(config.at("run_id"), {epoch_ns},
        config.at("transport").at("clock_replay_window"), maximum_elapsed_ns);
    flight_models::RuntimeTransport transport(config);
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    transport.open();
    std::cout << "flight-models ready run=" << config.at("run_id").get<std::string>() << std::endl;

    bool initialized = false;
    std::size_t dis_count = 0, prd_count = 0;
    while (!stopping) {
      const auto bytes = transport.next_clock();
      if (!bytes) continue;
      rlbm::proto::SimTimeTick tick;
      if (bytes->size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
          !tick.ParseFromArray(bytes->data(), static_cast<int>(bytes->size()))) {
        throw std::invalid_argument("cannot decode authoritative SimTimeTick protobuf");
      }
      const auto announced_ns = clock.accept(tick);
      if (!announced_ns) continue;

      // Each accepted clock boundary is processed in full; global run duration
      // belongs to the clock authority that announces these boundaries.
      auto outputs = initialized ? runtime.advance_to(*announced_ns) : std::move(initial);
      initialized = true;
      transport.publish(outputs);
      for (const auto& output : outputs) {
        if (output.kind == flight_models::SampleKind::physical) ++dis_count;
        else ++prd_count;
      }
    }
    // Drain already-running publication on pause/termination; no future clock
    // interval is admitted after the stop request. No tick ACK protocol is added.
    transport.flush();
    std::cout << "flight-models stopped dis=" << dis_count << " prd=" << prd_count << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "flight-models-run: " << error.what() << '\n';
    return 1;
  }
}
