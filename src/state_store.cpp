/** @file Bounded, single-owner structure-of-arrays state storage. */
#include "flight_models/state_store.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace flight_models {
namespace {

bool finite(const math::Vector3& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool valid_mode(EntityMode mode) {
  return mode == EntityMode::fixed || mode == EntityMode::moving || mode == EntityMode::disabled;
}

bool valid(const PhysicalState& state, const DesiredMotion& desired) {
  if (!finite(state.position_ecef_m) || !finite(state.velocity_ecef_mps) ||
      !finite(state.acceleration_ecef_mps2) || !finite(state.angular_velocity_body_radps) ||
      state.source_time_ns < 0 || !std::isfinite(desired.heading_rad) ||
      !std::isfinite(desired.horizontal_speed_mps) || !std::isfinite(desired.altitude_hae_m)) {
    return false;
  }
  return std::all_of(state.body_frd_to_ecef.begin(), state.body_frd_to_ecef.end(),
                     [](double value) { return std::isfinite(value); });
}

std::size_t checked_batch_capacity(std::size_t capacity) {
  if (capacity > std::numeric_limits<std::size_t>::max() / 24) {
    throw std::invalid_argument("batch capacity is not representable");
  }
  return capacity;
}

struct VecArrays {
  std::unique_ptr<double[]> x;
  std::unique_ptr<double[]> y;
  std::unique_ptr<double[]> z;

  explicit VecArrays(std::size_t capacity)
      : x(new double[capacity]{}), y(new double[capacity]{}), z(new double[capacity]{}) {}

  ConstVector3Columns view() const noexcept { return {x.get(), y.get(), z.get()}; }
};

}  // namespace

struct StateStore::Impl {
  Impl(std::size_t slot_capacity, std::size_t profiles)
      : capacity(slot_capacity), profile_count(profiles), active(new std::uint8_t[slot_capacity]{}),
        generations(new std::uint64_t[slot_capacity]{}), profiles_by_slot(new std::size_t[slot_capacity]{}),
        modes(new EntityMode[slot_capacity]{}), ids(slot_capacity), position(slot_capacity),
        velocity(slot_capacity), acceleration(slot_capacity), attitude(new double[slot_capacity * 9]{}),
        angular_velocity(slot_capacity), times(new std::int64_t[slot_capacity]{}),
        heading(new double[slot_capacity]{}), speed(new double[slot_capacity]{}),
        altitude(new double[slot_capacity]{}), seen(new std::uint64_t[slot_capacity]{}) {
    free_slots.reserve(slot_capacity);
    for (std::size_t slot = 0; slot < slot_capacity; ++slot) free_slots.push_back(slot);
    std::make_heap(free_slots.begin(), free_slots.end(), std::greater<std::size_t>{});
  }

  std::size_t capacity;
  std::size_t profile_count;
  std::size_t count{};
  std::uint64_t revision{};
  std::uint64_t validation_tag{};
  std::unique_ptr<std::uint8_t[]> active;
  std::unique_ptr<std::uint64_t[]> generations;
  std::unique_ptr<std::size_t[]> profiles_by_slot;
  std::unique_ptr<EntityMode[]> modes;
  std::vector<std::string> ids;
  std::map<std::string, std::size_t> slots_by_id;
  std::vector<std::size_t> free_slots;
  VecArrays position;
  VecArrays velocity;
  VecArrays acceleration;
  std::unique_ptr<double[]> attitude;
  VecArrays angular_velocity;
  std::unique_ptr<std::int64_t[]> times;
  std::unique_ptr<double[]> heading;
  std::unique_ptr<double[]> speed;
  std::unique_ptr<double[]> altitude;
  std::unique_ptr<std::uint64_t[]> seen;

  bool current(Handle handle) const noexcept {
    return handle.slot < capacity && active[handle.slot] &&
           generations[handle.slot] == handle.generation;
  }

  void ensure_revision_available() const {
    if (revision == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("state store revision exhausted");
    }
  }

  void begin_validation() {
    if (validation_tag == std::numeric_limits<std::uint64_t>::max()) {
      std::fill_n(seen.get(), capacity, 0);
      validation_tag = 1;
    } else {
      ++validation_tag;
    }
  }

  void write(std::size_t slot, const PhysicalState& state, const DesiredMotion& desired) {
    position.x[slot] = state.position_ecef_m.x;
    position.y[slot] = state.position_ecef_m.y;
    position.z[slot] = state.position_ecef_m.z;
    velocity.x[slot] = state.velocity_ecef_mps.x;
    velocity.y[slot] = state.velocity_ecef_mps.y;
    velocity.z[slot] = state.velocity_ecef_mps.z;
    acceleration.x[slot] = state.acceleration_ecef_mps2.x;
    acceleration.y[slot] = state.acceleration_ecef_mps2.y;
    acceleration.z[slot] = state.acceleration_ecef_mps2.z;
    std::copy(state.body_frd_to_ecef.begin(), state.body_frd_to_ecef.end(), attitude.get() + 9 * slot);
    angular_velocity.x[slot] = state.angular_velocity_body_radps.x;
    angular_velocity.y[slot] = state.angular_velocity_body_radps.y;
    angular_velocity.z[slot] = state.angular_velocity_body_radps.z;
    times[slot] = state.source_time_ns;
    heading[slot] = desired.heading_rad;
    speed[slot] = desired.horizontal_speed_mps;
    altitude[slot] = desired.altitude_hae_m;
  }

  void read(std::size_t slot, PhysicalState& state, DesiredMotion& desired) const {
    state.position_ecef_m = {position.x[slot], position.y[slot], position.z[slot]};
    state.velocity_ecef_mps = {velocity.x[slot], velocity.y[slot], velocity.z[slot]};
    state.acceleration_ecef_mps2 = {acceleration.x[slot], acceleration.y[slot], acceleration.z[slot]};
    std::copy_n(attitude.get() + 9 * slot, 9, state.body_frd_to_ecef.begin());
    state.angular_velocity_body_radps = {
        angular_velocity.x[slot], angular_velocity.y[slot], angular_velocity.z[slot]};
    state.source_time_ns = times[slot];
    desired = {heading[slot], speed[slot], altitude[slot]};
  }
};

Snapshot::Snapshot(std::size_t capacity) : rows_(capacity) {}

StateBatch::StateBatch(std::size_t capacity)
    : capacity_(checked_batch_capacity(capacity)), handles_(new Handle[capacity_]),
      profile_indices_(new std::size_t[capacity_]), modes_(new EntityMode[capacity_]),
      numbers_(new double[capacity_ * 24]{}), times_(new std::int64_t[capacity_] {}) {}

MutableStateColumns StateBatch::columns() noexcept {
  double* const base = numbers_.get();
  const std::size_t n = capacity_;
  MutableStateColumns columns{};
  columns.position_ecef_m = {base, base + n, base + 2 * n};
  columns.velocity_ecef_mps = {base + 3 * n, base + 4 * n, base + 5 * n};
  columns.acceleration_ecef_mps2 = {base + 6 * n, base + 7 * n, base + 8 * n};
  columns.body_frd_to_ecef = base + 9 * n;
  columns.angular_velocity_body_radps = {base + 18 * n, base + 19 * n, base + 20 * n};
  columns.source_time_ns = times_.get();
  columns.heading_rad = base + 21 * n;
  columns.horizontal_speed_mps = base + 22 * n;
  columns.altitude_hae_m = base + 23 * n;
  return columns;
}

void StateBatch::copy_out(std::size_t row, PhysicalState& state, DesiredMotion& desired) const {
  const double* const base = numbers_.get();
  const std::size_t n = capacity_;
  state.position_ecef_m = {base[row], base[n + row], base[2 * n + row]};
  state.velocity_ecef_mps = {base[3 * n + row], base[4 * n + row], base[5 * n + row]};
  state.acceleration_ecef_mps2 = {base[6 * n + row], base[7 * n + row], base[8 * n + row]};
  std::copy_n(base + 9 * n + 9 * row, 9, state.body_frd_to_ecef.begin());
  state.angular_velocity_body_radps = {base[18 * n + row], base[19 * n + row], base[20 * n + row]};
  state.source_time_ns = times_[row];
  desired = {base[21 * n + row], base[22 * n + row], base[23 * n + row]};
}

StateStore::StateStore(std::size_t capacity, std::size_t profile_count) {
  if (capacity == 0 || profile_count == 0 || capacity > std::numeric_limits<std::size_t>::max() / 24) {
    throw std::invalid_argument("state store bounds must be positive and representable");
  }
  impl_ = std::make_unique<Impl>(capacity, profile_count);
}

StateStore::~StateStore() = default;
StateStore::StateStore(StateStore&&) noexcept = default;
StateStore& StateStore::operator=(StateStore&&) noexcept = default;

Handle StateStore::spawn(std::string id, std::size_t profile_index, EntityMode mode,
                         const PhysicalState& state, const DesiredMotion& desired) {
  if (id.empty() || profile_index >= impl_->profile_count || !valid_mode(mode) || !valid(state, desired)) {
    throw std::invalid_argument("invalid entity identity, profile, mode, or state");
  }
  if (impl_->slots_by_id.find(id) != impl_->slots_by_id.end()) {
    throw std::invalid_argument("entity ID already exists");
  }
  if (impl_->count == impl_->capacity) throw std::length_error("state store is full");
  impl_->ensure_revision_available();

  const std::size_t slot = impl_->free_slots.front();
  if (impl_->generations[slot] == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("slot generation exhausted");
  }
  const std::uint64_t generation = impl_->generations[slot] + 1;
  // Allocate/map the identity before publishing occupancy, so insertion failure is harmless.
  const auto inserted = impl_->slots_by_id.emplace(id, slot);
  std::pop_heap(impl_->free_slots.begin(), impl_->free_slots.end(), std::greater<std::size_t>{});
  impl_->free_slots.pop_back();
  impl_->ids[slot] = std::move(id);
  impl_->generations[slot] = generation;
  impl_->profiles_by_slot[slot] = profile_index;
  impl_->modes[slot] = mode;
  impl_->write(slot, state, desired);
  impl_->active[slot] = 1;
  ++impl_->count;
  ++impl_->revision;
  return {inserted.first->second, generation};
}

bool StateStore::erase(Handle handle) {
  if (!impl_->current(handle)) return false;
  impl_->ensure_revision_available();
  impl_->slots_by_id.erase(impl_->ids[handle.slot]);
  impl_->ids[handle.slot].clear();
  impl_->active[handle.slot] = 0;
  impl_->free_slots.push_back(handle.slot);
  std::push_heap(impl_->free_slots.begin(), impl_->free_slots.end(), std::greater<std::size_t>{});
  --impl_->count;
  ++impl_->revision;
  return true;
}

void StateStore::clear() {
  impl_->ensure_revision_available();
  for (std::size_t slot = 0; slot < impl_->capacity; ++slot) {
    if (impl_->active[slot] && impl_->generations[slot] == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("slot generation exhausted");
    }
  }
  for (std::size_t slot = 0; slot < impl_->capacity; ++slot) {
    if (!impl_->active[slot]) continue;
    ++impl_->generations[slot];
    impl_->active[slot] = 0;
    impl_->ids[slot].clear();
  }
  impl_->slots_by_id.clear();
  impl_->free_slots.clear();
  for (std::size_t slot = 0; slot < impl_->capacity; ++slot) impl_->free_slots.push_back(slot);
  std::make_heap(impl_->free_slots.begin(), impl_->free_slots.end(), std::greater<std::size_t>{});
  impl_->count = 0;
  ++impl_->revision;
}

std::optional<Handle> StateStore::find(const std::string& id) const {
  const auto it = impl_->slots_by_id.find(id);
  if (it == impl_->slots_by_id.end()) return std::nullopt;
  const std::size_t slot = it->second;
  return Handle{slot, impl_->generations[slot]};
}

bool StateStore::set_mode(Handle handle, EntityMode mode) {
  if (!impl_->current(handle) || !valid_mode(mode)) return false;
  impl_->ensure_revision_available();
  impl_->modes[handle.slot] = mode;
  ++impl_->revision;
  return true;
}

bool StateStore::read_state(Handle handle, PhysicalState& state, DesiredMotion& desired) const {
  if (!impl_->current(handle)) return false;
  impl_->read(handle.slot, state, desired);
  return true;
}

std::size_t StateStore::size() const noexcept { return impl_->count; }
std::size_t StateStore::capacity() const noexcept { return impl_->capacity; }

StateColumns StateStore::columns() const noexcept {
  const Impl& x = *impl_;
  return {x.position.view(), x.velocity.view(), x.acceleration.view(), x.attitude.get(),
          x.angular_velocity.view(), x.times.get(), x.heading.get(), x.speed.get(),
          x.altitude.get(), x.active.get(), x.profiles_by_slot.get(), x.modes.get()};
}

void StateStore::gather(const Handle* handles, std::size_t count, StateBatch& batch) const {
  batch.prepared_ = false;
  batch.count_ = 0;
  if (count > batch.capacity_) throw std::length_error("batch capacity exceeded");
  if (count != 0 && handles == nullptr) throw std::invalid_argument("null handle array");
  impl_->begin_validation();
  for (std::size_t row = 0; row < count; ++row) {
    if (!impl_->current(handles[row])) throw std::invalid_argument("stale handle in gather");
    const std::size_t slot = handles[row].slot;
    if (impl_->seen[slot] == impl_->validation_tag) throw std::invalid_argument("duplicate handle in gather");
    impl_->seen[slot] = impl_->validation_tag;
  }

  MutableStateColumns columns = batch.columns();
  for (std::size_t row = 0; row < count; ++row) {
    const Handle handle = handles[row];
    batch.handles_[row] = handle;
    batch.profile_indices_[row] = impl_->profiles_by_slot[handle.slot];
    batch.modes_[row] = impl_->modes[handle.slot];
    PhysicalState state;
    DesiredMotion desired;
    impl_->read(handle.slot, state, desired);
    columns.position_ecef_m.x[row] = state.position_ecef_m.x;
    columns.position_ecef_m.y[row] = state.position_ecef_m.y;
    columns.position_ecef_m.z[row] = state.position_ecef_m.z;
    columns.velocity_ecef_mps.x[row] = state.velocity_ecef_mps.x;
    columns.velocity_ecef_mps.y[row] = state.velocity_ecef_mps.y;
    columns.velocity_ecef_mps.z[row] = state.velocity_ecef_mps.z;
    columns.acceleration_ecef_mps2.x[row] = state.acceleration_ecef_mps2.x;
    columns.acceleration_ecef_mps2.y[row] = state.acceleration_ecef_mps2.y;
    columns.acceleration_ecef_mps2.z[row] = state.acceleration_ecef_mps2.z;

    std::copy(state.body_frd_to_ecef.begin(), state.body_frd_to_ecef.end(), columns.body_frd_to_ecef + 9 * row);
    columns.angular_velocity_body_radps.x[row] = state.angular_velocity_body_radps.x;
    columns.angular_velocity_body_radps.y[row] = state.angular_velocity_body_radps.y;
    columns.angular_velocity_body_radps.z[row] = state.angular_velocity_body_radps.z;

    columns.source_time_ns[row] = state.source_time_ns;
    columns.heading_rad[row] = desired.heading_rad;
    columns.horizontal_speed_mps[row] = desired.horizontal_speed_mps;
    columns.altitude_hae_m[row] = desired.altitude_hae_m;
  }
  batch.count_ = count;
  batch.owner_ = impl_.get();
  batch.revision_ = impl_->revision;
  batch.prepared_ = true;
}

void StateStore::gather_profile(std::size_t profile_index, EntityMode mode, StateBatch& batch) const {
  batch.prepared_ = false;
  batch.count_ = 0;
  if (profile_index >= impl_->profile_count || !valid_mode(mode)) {
    throw std::invalid_argument("invalid profile or mode");
  }
  // The ordered ID map supplies canonical order without per-update sorting or allocation.
  std::size_t count = 0;
  for (const auto& [id, slot] : impl_->slots_by_id) {
    (void)id;
    if (impl_->profiles_by_slot[slot] == profile_index && impl_->modes[slot] == mode) {
      if (count == batch.capacity_) throw std::length_error("batch capacity exceeded");
      batch.handles_[count++] = {slot, impl_->generations[slot]};
    }
  }
  gather(batch.handles_.get(), count, batch);
}

void StateStore::commit(StateBatch& batch) {
  if (!batch.prepared_ || batch.owner_ != impl_.get() || batch.revision_ != impl_->revision) {
    throw std::invalid_argument("batch is unprepared, foreign, or stale");
  }
  impl_->ensure_revision_available();
  impl_->begin_validation();

  // Validate every row before changing any live value.
  for (std::size_t row = 0; row < batch.count_; ++row) {
    const Handle handle = batch.handles_[row];
    if (!impl_->current(handle)) throw std::invalid_argument("batch handle is stale");
    if (impl_->seen[handle.slot] == impl_->validation_tag) throw std::invalid_argument("duplicate handle in batch");
    impl_->seen[handle.slot] = impl_->validation_tag;
    PhysicalState state;
    DesiredMotion desired;
    batch.copy_out(row, state, desired);
    if (!valid(state, desired) || state.source_time_ns < impl_->times[handle.slot]) {
      throw std::invalid_argument("batch has invalid values or decreasing source time");
    }
  }

  // All writes are now known to be valid and remain bounded by gathered rows.
  for (std::size_t row = 0; row < batch.count_; ++row) {
    PhysicalState state;
    DesiredMotion desired;
    batch.copy_out(row, state, desired);
    impl_->write(batch.handles_[row].slot, state, desired);
  }
  ++impl_->revision;
  batch.prepared_ = false;
}

void StateStore::snapshot(Snapshot& snapshot) const {
  if (snapshot.rows_.size() < impl_->count) throw std::length_error("snapshot capacity exceeded");
  snapshot.count_ = 0;
  snapshot.revision_ = 0;
  std::size_t row = 0;
  // Ordered identity traversal also makes the copied snapshot canonical.
  for (const auto& [id, slot] : impl_->slots_by_id) {
    SnapshotRow& output = snapshot.rows_[row++];
    output.handle = {slot, impl_->generations[slot]};
    output.profile_index = impl_->profiles_by_slot[slot];
    output.mode = impl_->modes[slot];
    output.id = id;
    impl_->read(slot, output.state, output.desired);
  }
  snapshot.count_ = row;
  snapshot.revision_ = impl_->revision;
}

}  // namespace flight_models
