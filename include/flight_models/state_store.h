/** @file
 * Bounded structure-of-arrays storage for committed physical and desired state.
 */
#pragma once

#include "math/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace flight_models {

/** Motion classification stored with each entity. */
enum class EntityMode : std::uint8_t { fixed, moving, disabled };

/** ECEF state and body-frame angular velocity at one simulation timestamp. */
struct PhysicalState {
  math::Vector3 position_ecef_m{};       /**< ECEF position, metres. */
  math::Vector3 velocity_ecef_mps{};     /**< ECEF velocity, metres/second. */
  math::Vector3 acceleration_ecef_mps2{}; /**< ECEF acceleration, metres/second². */
  math::Matrix3 body_frd_to_ecef{};      /**< Row-major body-FRD to ECEF DCM. */
  math::Vector3 angular_velocity_body_radps{}; /**< Body-FRD angular velocity, rad/s. */
  std::int64_t source_time_ns{};          /**< Source simulation time, ns. */
};

/** Heading, horizontal speed, and height above the WGS-84 ellipsoid. */
struct DesiredMotion {
  double heading_rad{};       /**< Heading clockwise from true north, radians. */
  double horizontal_speed_mps{}; /**< Horizontal ground speed, m/s. */
  double altitude_hae_m{};    /**< Height above WGS-84 ellipsoid, metres. */
};

/** Stable reference to a slot; generation changes whenever a slot is reused. */
struct Handle {
  std::size_t slot{};         /**< Storage slot index. */
  std::uint64_t generation{}; /**< Slot generation at handle creation. */
  friend bool operator==(Handle a, Handle b) noexcept {
    return a.slot == b.slot && a.generation == b.generation;
  }
  friend bool operator!=(Handle a, Handle b) noexcept { return !(a == b); }
};

/** Three separately contiguous component columns. */
struct Vector3Columns {
  double* x{}; /**< X component column. */
  double* y{}; /**< Y component column. */
  double* z{}; /**< Z component column. */
};

/** Read-only component columns for a vector field. */
struct ConstVector3Columns {
  const double* x{}; /**< X component column. */
  const double* y{}; /**< Y component column. */
  const double* z{}; /**< Z component column. */
};

/** Mutable numeric scratch columns owned by a StateBatch. */
struct MutableStateColumns {
  Vector3Columns position_ecef_m; /**< ECEF position, m. */
  Vector3Columns velocity_ecef_mps; /**< ECEF velocity, m/s. */
  Vector3Columns acceleration_ecef_mps2; /**< ECEF acceleration, m/s². */
  double* body_frd_to_ecef{}; /**< Row-major matrices, nine doubles per row. */
  Vector3Columns angular_velocity_body_radps; /**< Body angular velocity, rad/s. */
  std::int64_t* source_time_ns{}; /**< Source simulation time, ns. */
  double* heading_rad{}; /**< Desired heading, radians. */
  double* horizontal_speed_mps{}; /**< Desired horizontal speed, m/s. */
  double* altitude_hae_m{}; /**< Desired WGS-84 HAE, m. */
};

/** Read-only live storage columns and occupancy mask. */
struct StateColumns {
  ConstVector3Columns position_ecef_m; /**< ECEF position, m. */
  ConstVector3Columns velocity_ecef_mps; /**< ECEF velocity, m/s. */
  ConstVector3Columns acceleration_ecef_mps2; /**< ECEF acceleration, m/s². */
  const double* body_frd_to_ecef{}; /**< Row-major matrices, nine doubles per slot. */
  ConstVector3Columns angular_velocity_body_radps; /**< Body angular velocity, rad/s. */
  const std::int64_t* source_time_ns{}; /**< Source simulation time, ns. */
  const double* heading_rad{}; /**< Desired heading, radians. */
  const double* horizontal_speed_mps{}; /**< Desired horizontal speed, m/s. */
  const double* altitude_hae_m{}; /**< Desired HAE, m. */
  const std::uint8_t* active{}; /**< Nonzero for occupied slots. */
  const std::size_t* profile_index{}; /**< Configured profile index by slot. */
  const EntityMode* mode{}; /**< Entity mode by slot. */
};

/** One immutable copied row in a snapshot. */
struct SnapshotRow {
  Handle handle{}; /**< Handle valid in the captured store revision. */
  std::size_t profile_index{}; /**< Bound profile index. */
  EntityMode mode{}; /**< Stored entity mode. */
  std::string id; /**< Entity identifier. */
  PhysicalState state{}; /**< Captured physical state and source time. */
  DesiredMotion desired{}; /**< Captured desired motion. */
};

/**
 * Preallocated destination for a committed-state snapshot.
 * Do not reuse the object while consumers are reading its rows; another snapshot
 * call overwrites copied contents and invalidates the prior logical view.
 */
class Snapshot {
 public:
  /** Allocate capacity rows; @throws std::length_error for impossible size. */
  explicit Snapshot(std::size_t capacity);
  /** Captured row count. */
  std::size_t size() const noexcept { return count_; }
  /** Store revision copied into this snapshot. */
  std::uint64_t revision() const noexcept { return revision_; }
  /** Captured rows in canonical entity-ID order; valid until the next snapshot. */
  const SnapshotRow* rows() const noexcept { return rows_.data(); }
 private:
  friend class StateStore;
  std::vector<SnapshotRow> rows_;
  std::size_t count_{};
  std::uint64_t revision_{};
};

/**
 * Preallocated staging area for an update cohort. Handles and prepared batches
 * are scoped to the StateStore instance that produced them and its lifetime.
 */
class StateBatch {
 public:
  /** Allocate scratch for at most capacity rows. */
  explicit StateBatch(std::size_t capacity);
  /** Batch row count after gather; rows correspond to handles(). */
  std::size_t size() const noexcept { return count_; }
  /** Capacity fixed at construction. */
  std::size_t capacity() const noexcept { return capacity_; }
  /** Read-only gathered handles, in batch row order. */
  const Handle* handles() const noexcept { return handles_.get(); }
  /** Mutable scratch only, rows [0, size()); changes reach live state only through commit. */
  MutableStateColumns columns() noexcept;
  /** Read-only profile metadata for valid batch rows. */
  const std::size_t* profile_indices() const noexcept { return profile_indices_.get(); }
  /** Read-only mode metadata for valid batch rows. */
  const EntityMode* modes() const noexcept { return modes_.get(); }
 private:
  friend class StateStore;
  std::size_t capacity_{};
  std::size_t count_{};
  std::unique_ptr<Handle[]> handles_;
  std::unique_ptr<std::size_t[]> profile_indices_;
  std::unique_ptr<EntityMode[]> modes_;
  std::unique_ptr<double[]> numbers_;
  std::unique_ptr<std::int64_t[]> times_;
  const void* owner_{};
  std::uint64_t revision_{};
  bool prepared_{};
  void copy_out(std::size_t row, PhysicalState& state, DesiredMotion& desired) const;
};

/**
 * Single-owner bounded entity store with transactional staged updates.
 * Calls must be serialized by the owner; the class provides no synchronization.
 * A StateColumns view points into live arrays and expires when the store is
 * destroyed or moved. Snapshot rows are independent copies.
 */
class StateStore {
 public:
  /**
   * Allocate fixed slot and profile-index bounds.
   * @param capacity Maximum occupied entities; numerical arrays never grow.
   * @param profile_count Number of entries in the configured model registry.
   * @throws std::invalid_argument when either bound is zero.
   */
  StateStore(std::size_t capacity, std::size_t profile_count);
  ~StateStore();
  /** Moving transfers ownership; the moved-from store may only be assigned or destroyed. */
  StateStore(StateStore&&) noexcept;
  StateStore& operator=(StateStore&&) noexcept;
  StateStore(const StateStore&) = delete;
  StateStore& operator=(const StateStore&) = delete;

  /**
   * Insert an entity into the lowest free slot.
   * @param id Nonempty platform identity, unique within this store.
   * @param profile_index Index in [0, configured profile count).
   * @param mode Physical motion class; disabled remains occupied.
   * @param state Finite physical state with nonnegative source nanoseconds.
   * @param desired Finite desired motion; the model validates envelope bounds.
   * @return Handle scoped to this store lifetime.
   * @throws std::invalid_argument for invalid state, mode, profile, or duplicate ID.
   * @throws std::length_error when full.
   */
  Handle spawn(std::string id, std::size_t profile_index, EntityMode mode,
               const PhysicalState& state, const DesiredMotion& desired);
  /** Remove a current handle; returns false for stale or out-of-range handles. */
  bool erase(Handle handle);
  /** Remove all entities and invalidate all previously issued handles. */
  void clear();
  /** Find an ID without copying it. */
  std::optional<Handle> find(const std::string& id) const;
  /** Change mode for a current handle; false means stale handle or invalid mode. */
  bool set_mode(Handle handle, EntityMode mode);
  /** Copy state for a valid handle; returns false if stale. */
  bool read_state(Handle handle, PhysicalState& state, DesiredMotion& desired) const;
  /** Current entity count. */
  std::size_t size() const noexcept;
  /** Fixed entity bound. */
  std::size_t capacity() const noexcept;
  /** Read-only live columns; views expire on store destruction or move. */
  StateColumns columns() const noexcept;
  /**
   * Gather explicit current handles into batch scratch, preserving input order.
   * @param handles Array of count handles from this store; null is valid only for zero count.
   * @param count Number of selected rows.
   * @param batch Reusable destination; valid column/metadata rows are [0, size()).
   * @throws std::length_error if count exceeds batch capacity.
   * @throws std::invalid_argument for null input with nonzero count, stale handles,
   *         or duplicates. Failure invalidates the batch.
   */
  void gather(const Handle* handles, std::size_t count, StateBatch& batch) const;
  /**
   * Gather matching rows in canonical ID order; failure invalidates the batch.
   * @param profile_index Index in the caller's configured model registry.
   * @param mode Exact motion class to select.
   * @param batch Reusable destination with capacity for the entire selected cohort.
   * @throws std::length_error if the matching cohort exceeds batch capacity.
   * @throws std::invalid_argument for an invalid profile or mode.
   */
  void gather_profile(std::size_t profile_index, EntityMode mode, StateBatch& batch) const;
  /**
   * Validate all staged rows, then copy numeric values live in one transaction.
   * @param batch Prepared scratch; successful commit consumes its preparation.
   * Caller must finish any parallel scratch writes before committing.
   * @throws std::invalid_argument for unprepared, foreign, stale, duplicate, invalid,
   *         or decreasing-time rows. A failed commit leaves live values unchanged.
   */
  void commit(StateBatch& batch);
  /**
   * Copy committed rows in canonical ID order into a fixed-capacity snapshot.
   * @param snapshot Reusable owning destination; do not refill while readers use it.
   * Snapshot contents and revision remain unchanged until this method is called again.
   * A copying failure invalidates the destination row count.
   * @throws std::length_error if snapshot capacity is smaller than store size.
   */
  void snapshot(Snapshot& snapshot) const;
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flight_models
