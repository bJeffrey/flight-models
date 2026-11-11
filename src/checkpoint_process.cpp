#include "flight_models/checkpoint_process.h"

#include "flight_models/checkpoint_state_owner.h"
#include "flight_models/runtime_transport.h"
#include "checkpoint_runtime/participant.hpp"
#include "runtime_support/simulation_clock.hpp"
#include "runtime_control.pb.h"

#include <algorithm>
#include <csignal>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace flight_models {
namespace {
namespace cr = checkpoint_runtime;
volatile std::sig_atomic_t stopping = 0;
void request_stop(int) { stopping = 1; }

cr::ClockBoundary boundary(const rlbm::proto::SimTimeTick& tick) {
  return {tick.run_id(), tick.timeline_epoch(), tick.tick_id(), tick.t_us(),
          tick.dt_us(), tick.scenario_epoch_utc()};
}

cr::Request request_from_wire(const rlbm::proto::ServiceCheckpointRequest& wire) {
  const auto& source = wire.source();
  cr::Request request;
  request.operation_id       = wire.operation_id();
  request.checkpoint_id      = wire.checkpoint_id();
  request.kind               = static_cast<cr::OperationKind>(wire.kind());
  request.phase              = static_cast<cr::Phase>(wire.phase());
  request.source             = {source.run_id(), source.source_epoch(), source.tick_id(),
                                source.t_us(), source.dt_us(), source.scenario_epoch_utc()};
  request.current_epoch      = wire.current_epoch();
  request.target_epoch       = wire.target_epoch();
  request.required_service_ids.assign(wire.required_service_ids().begin(), wire.required_service_ids().end());
  request.manifest_ref = wire.manifest_ref();
  for (const auto& position : wire.input_positions()) {
    request.input_positions.emplace_back(position.source_id(), position.sequence());
  }
  return request;
}

std::vector<std::uint8_t> response_bytes(const cr::Response& response, const cr::ClockBoundary& current) {
  const auto& request = response.request;
  rlbm::proto::ServiceCheckpointResponse wire;
  wire.set_operation_id(request.operation_id);
  wire.set_checkpoint_id(request.checkpoint_id);
  wire.set_kind(static_cast<rlbm::proto::CheckpointOperationKind>(request.kind));
  wire.set_phase(static_cast<rlbm::proto::CheckpointPhase>(request.phase));
  auto* source = wire.mutable_source();
  source->set_run_id(request.source.run_id);
  source->set_source_epoch(request.source.epoch);
  source->set_tick_id(request.source.tick);
  source->set_t_us(request.source.t_us);
  source->set_dt_us(request.source.dt_us);
  source->set_scenario_epoch_utc(request.source.scenario_epoch_utc);
  wire.set_current_epoch(request.current_epoch);
  wire.set_target_epoch(request.target_epoch);
  wire.set_service_id(response.identity.service_id);
  wire.set_success(response.success);
  wire.set_failure_reason(response.failure_reason);
  wire.set_software_id(response.identity.software_id);
  wire.set_config_id(response.identity.config_id);
  wire.set_state_schema(response.identity.state_schema);
  wire.set_state_version(response.identity.state_version);
  wire.set_payload_ref(response.payload_ref);
  wire.set_payload_sha256(response.payload_sha256);
  wire.set_payload_size(response.payload_size);
  wire.set_bundle_size(response.bundle_size);
  wire.set_capacity_blocked(response.capacity_blocked);
  for (const auto& receipt : response.delivery_receipts) {
    auto* output = wire.add_delivery_receipts();
    output->set_stream_id(receipt.stream_id);
    output->set_message_count(receipt.message_count);
    output->set_payload_digest(receipt.payload_digest);
  }
  for (const auto& position : request.input_positions) {
    auto* output = wire.add_input_positions();
    output->set_source_id(position.first);
    output->set_sequence(position.second);
  }

  // Checkpoint responses describe a logical processed boundary, not wall time.
  auto* header = wire.mutable_message_header();
  header->set_message_id(response.identity.service_id + "/" + request.operation_id + "/" +
                         std::to_string(static_cast<int>(request.phase)));
  header->mutable_timestamp()->set_t_us(current.t_us);
  header->mutable_originator()->set_kind(rlbm::proto::MessageOriginator::KIND_SERVICE);
  header->mutable_originator()->set_id(response.identity.service_id);
  header->mutable_originator()->set_role("checkpoint-participant");
  header->mutable_schema()->set_name("ServiceCheckpointResponse");
  header->mutable_schema()->set_version("1");
  std::string bytes;
  if (!wire.SerializeToString(&bytes)) throw std::runtime_error("cannot encode checkpoint response");
  return {bytes.begin(), bytes.end()};
}

template <typename Message>
Message decode(const std::vector<std::uint8_t>& bytes) {
  Message message;
  if (bytes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      !message.ParseFromArray(bytes.data(), static_cast<int>(bytes.size()))) {
    throw std::invalid_argument("cannot decode native simulation-control protobuf");
  }
  return message;
}
}  // namespace

int run_checkpoint_process(const nlohmann::json& config) {
  const auto& profile = config.at("checkpoint");
  const cr::ServiceIdentity identity{
      profile.at("service_id"), profile.at("state_schema"), profile.at("state_version"),
      profile.at("software_id"), profile.at("config_id")};
  const cr::ParticipantLimits limits{
      profile.at("max_count"), profile.at("max_bytes"), profile.at("max_snapshot_bytes"),
      profile.at("max_completed_operations")};
  const auto origin_ns = config.at("scenario_epoch_unix_ns").get<std::int64_t>();
  const auto maximum_elapsed_ns = origin_ns > 0 ? std::numeric_limits<std::int64_t>::max() - origin_ns :
                                                 std::numeric_limits<std::int64_t>::max();
  const auto run_id = config.at("run_id").get<std::string>();
  runtime_support::SimulationClock clock(run_id, {origin_ns},
      config.at("transport").at("clock_replay_window"), maximum_elapsed_ns);
  RuntimeTransport transport(config);
  std::size_t dis_count = 0;
  std::size_t prd_count = 0;
  CheckpointStateOwner owner(config, [&](const auto& outputs) {
    transport.publish(outputs);
    for (const auto& output : outputs) {
      if (output.kind == SampleKind::physical) ++dis_count;
      else ++prd_count;
    }
  });
  cr::CheckpointStore store(profile.at("run_directory").get<std::string>());
  const cr::ClockBoundary initial{run_id, 0, 0, 0, profile.at("dt_us"), config.at("scenario_epoch_utc")};
  cr::CheckpointParticipant participant(owner, store, initial, identity, limits);
  // A durable prior session fences this fresh owner before any initial truth can escape.
  participant.start_session();
  transport.open();
  stopping = 0;
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::cout << "flight-models ready run=" << run_id << " checkpoint=" << identity.service_id
            << " recovery=" << participant.recovery_required() << std::endl;

  while (!stopping) {
    const auto event = transport.next_event();
    if (!event) continue;
    if (event->kind == RuntimeEventKind::checkpoint_request) {
      const auto wire = decode<rlbm::proto::ServiceCheckpointRequest>(event->payload);
      const auto request = request_from_wire(wire);
      if (std::find(request.required_service_ids.begin(), request.required_service_ids.end(), identity.service_id) ==
          request.required_service_ids.end()) continue;
      cr::Response response;
      if (request.phase == cr::Phase::delivery_receipts) {
        response = participant.delivery_receipts(request, {transport.dis_delivery_receipt()});
      } else if (!owner.initialized() && !participant.recovery_required()) {
        response.request        = request;
        response.identity       = identity;
        response.failure_reason = "initial authoritative outputs have not been admitted";
      } else {
        try {
          if (request.phase == cr::Phase::fence || request.phase == cr::Phase::prepare_restore) {
            // ASB admission completes before freezing state or discarding the old continuation.
            transport.flush();
          }
          response = participant.handle_request(request);
        } catch (const std::exception& error) {
          // Publication uncertainty must never permit an ordinary resume or snapshot.
          participant.require_recovery();
          response.request        = request;
          response.identity       = identity;
          response.failure_reason = error.what();
        }
      }
      transport.publish_checkpoint_response(response_bytes(response, participant.clock()));
      continue;
    }

    const auto tick = decode<rlbm::proto::SimTimeTick>(event->payload);
    if (tick.timeline_epoch() < participant.clock().epoch) continue;
    if (participant.fenced()) {
      // Queued ordinary ticks cannot process while fenced. Only the post-APPLY
      // paused announcement can establish a new clock boundary before RELEASE.
      const bool restored_announcement = owner.initialized() && tick.paused() && !tick.advances_time() &&
          tick.tick_id() == 0 && tick.timeline_epoch() > participant.clock().epoch &&
          tick.timeline_epoch() == owner.current_timeline_epoch() && tick.t_us() == owner.current_t_us();
      if (!restored_announcement) continue;
      participant.handle_tick({boundary(tick), true, false});
      try { clock.restore_boundary(tick); }
      catch (...) { participant.require_recovery(); throw; }
      transport.reset_delivery_receipts();
      continue;
    }

    const auto elapsed_ns = clock.accept(tick);
    if (!elapsed_ns) {
      if (!tick.advances_time()) participant.handle_tick({boundary(tick), tick.paused(), false});
      continue;
    }
    if (!owner.initialized()) {
      participant.handle_tick({boundary(tick), tick.paused(), tick.advances_time()});
      owner.publish_initial();
    } else {
      participant.handle_tick({boundary(tick), tick.paused(), tick.advances_time()});
    }
  }
  transport.flush();
  std::cout << "flight-models stopped dis=" << dis_count << " prd=" << prd_count << std::endl;
  return 0;
}
}  // namespace flight_models
