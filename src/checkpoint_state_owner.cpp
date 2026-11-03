#include "flight_models/checkpoint_state_owner.h"

#include <limits>
#include <stdexcept>
#include <utility>

namespace flight_models {
namespace {

std::int64_t configured_end_time_ns(const nlohmann::json& config) {
  if (!config.is_object() || !config.contains("end_time_ns")) {
    throw std::invalid_argument("checkpoint state owner requires configured end_time_ns");
  }
  const auto& value = config.at("end_time_ns");
  if (value.is_number_unsigned()) {
    const auto parsed = value.get<std::uint64_t>();
    if (parsed > static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
      throw std::invalid_argument("end_time_ns exceeds signed nanosecond range");
    }
    return static_cast<std::int64_t>(parsed);
  }
  if (!value.is_number_integer()) {
    throw std::invalid_argument("end_time_ns must be an integer nanosecond boundary");
  }
  const auto parsed = value.get<std::int64_t>();
  if (parsed < 0) throw std::invalid_argument("end_time_ns must be nonnegative");
  return parsed;
}

std::int64_t elapsed_nanoseconds(std::uint64_t time_us, std::int64_t end_time_ns) {
  constexpr auto maximum_us = static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000);
  if (time_us > maximum_us) throw std::invalid_argument("elapsed microseconds exceed nanosecond range");
  const auto time_ns = static_cast<std::int64_t>(time_us * 1000);
  if (time_ns > end_time_ns) throw std::invalid_argument("elapsed time exceeds configured end_time_ns");
  return time_ns;
}

struct RuntimeRestoreCandidate final : checkpoint_runtime::RestoreCandidate {
  const CheckpointStateOwner* owner{};
  Runtime runtime;
  std::uint32_t epoch{};
  std::uint64_t time_us{};

  RuntimeRestoreCandidate(const CheckpointStateOwner* source_owner, Runtime prepared,
                          std::uint32_t target_epoch, std::uint64_t target_time_us)
      : owner(source_owner), runtime(std::move(prepared)), epoch(target_epoch), time_us(target_time_us) {}
};

}  // namespace

CheckpointStateOwner::CheckpointStateOwner(
    const nlohmann::json& config,
    std::function<void(const std::vector<RuntimeOutput>&)> publish)
    : runtime_(config), publish_(std::move(publish)), end_time_ns_(configured_end_time_ns(config)) {
  if (!publish_) throw std::invalid_argument("checkpoint state owner requires an output publisher");

  // Runtime computes the initial sample now, but authority admission controls publication.
  held_initial_ = runtime_.advance_to(0);
}

void CheckpointStateOwner::publish_initial() {
  if (initialization_attempted_) {
    throw std::logic_error("initial output publication was already attempted");
  }
  initialization_attempted_ = true;
  publish_(held_initial_);
  held_initial_.clear();
  processed_t_us_ = 0;
  initialized_    = true;
}

bool CheckpointStateOwner::initialized() const noexcept { return initialized_; }

std::uint64_t CheckpointStateOwner::current_t_us() const { return processed_t_us_; }

std::uint32_t CheckpointStateOwner::current_timeline_epoch() const { return epoch_; }

std::string CheckpointStateOwner::capture_state() const {
  if (!initialized_) throw std::logic_error("cannot capture before initial output publication");
  return runtime_.encode_checkpoint();
}

std::unique_ptr<checkpoint_runtime::RestoreCandidate> CheckpointStateOwner::prepare_restore(
    const std::string& payload, std::uint32_t target_epoch) const {
  if (target_epoch <= epoch_) throw std::invalid_argument("restore target epoch must exceed current owner epoch");

  Runtime prepared = runtime_.prepare_restore(payload);
  const RuntimeCheckpoint checkpoint = prepared.checkpoint();
  const std::int64_t time_ns = checkpoint.driver.time_ns;
  if (!checkpoint.started || !checkpoint.driver.initialized || time_ns < 0 || time_ns % 1000 != 0) {
    throw std::invalid_argument("restore candidate must be initialized at a whole-microsecond boundary");
  }
  const auto time_us = static_cast<std::uint64_t>(time_ns / 1000);
  static_cast<void>(elapsed_nanoseconds(time_us, end_time_ns_));
  return std::make_unique<RuntimeRestoreCandidate>(this, std::move(prepared), target_epoch, time_us);
}

void CheckpointStateOwner::apply_restore(
    std::unique_ptr<checkpoint_runtime::RestoreCandidate> candidate) {
  auto* prepared = dynamic_cast<RuntimeRestoreCandidate*>(candidate.get());
  if (prepared == nullptr || prepared->owner != this || prepared->epoch <= epoch_) {
    throw std::invalid_argument("restore candidate is not valid for this checkpoint state owner");
  }

  runtime_        = std::move(prepared->runtime);
  epoch_          = prepared->epoch;
  processed_t_us_ = prepared->time_us;
  held_initial_.clear();
  initialization_attempted_ = true;
  initialized_              = true;
}

void CheckpointStateOwner::advance_to(std::uint64_t time_us) {
  if (!initialized_) throw std::logic_error("cannot advance before initial output publication");
  if (time_us < processed_t_us_) throw std::invalid_argument("elapsed time cannot move backward");
  const auto time_ns = elapsed_nanoseconds(time_us, end_time_ns_);

  // Publish only after the runtime has produced the complete owning output batch.
  const auto outputs = runtime_.advance_to(time_ns);
  publish_(outputs);
  processed_t_us_ = time_us;
}

}  // namespace flight_models
