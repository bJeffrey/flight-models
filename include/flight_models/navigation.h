/** @file
 * Ideal test navigation state copied from committed physical snapshots.
 */
#pragma once

#include "flight_models/state_store.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace flight_models {

/**
 * Reported ECEF state and covariance for one navigation sample.
 * The initial IdealNavigationBatch producer requires explicit ideal_test
 * selection and returns zero covariance for its exact-copy assumption. The PRD
 * boundary accepts this record independently of truth; nonzero covariance may
 * be used in adapter fixtures but does not establish a nonideal error model.
 */
struct NavigationSolution {
  math::Vector3 position_ecef_m{}; /**< ECEF position, metres. */
  math::Vector3 velocity_ecef_mps{}; /**< ECEF velocity, metres/second. */
  math::Matrix3 body_frd_to_ecef{}; /**< Row-major body-FRD to ECEF DCM. */
  math::Covariance6 covariance_ecef{}; /**< Row-major [position, velocity] ECEF covariance; zero for ideal_test. */
  std::int64_t source_time_ns{}; /**< Simulation validity time, nanoseconds. */
};

/**
 * Bounded reusable batch of ideal navigation samples from owning snapshots.
 * Numeric state is copied into separate reusable columns; later physical-state
 * changes cannot alter published solutions. Calls are single-owner and must
 * be serialized. Entity ID references expire on the next successful generate()
 * or destruction; returned solution values remain independent owning copies.
 */
class IdealNavigationBatch {
 public:
  /**
   * Allocate a fixed row capacity and require explicit model selection.
   * @param capacity Maximum number of enabled snapshot rows.
   * @param selected_model Must be exactly "ideal_test".
   * @throws std::invalid_argument for zero capacity or any other model.
   * @throws std::length_error if numeric storage size is not representable.
   */
  IdealNavigationBatch(std::size_t capacity, const std::string& selected_model);

  /**
   * Copy enabled fixed and moving rows in canonical snapshot order.
   * Disabled rows are omitted. The previous batch remains published if input
   * validation, allocation, or capacity checking fails.
   * @param snapshot Owning physical-state snapshot; no live-store fallback occurs.
   * @throws std::invalid_argument for invalid identity, negative time, nonfinite
   *         state, or a matrix that is not a proper orthonormal DCM.
   * @throws std::length_error if enabled rows exceed fixed capacity.
   */
  void generate(const Snapshot& snapshot);

  /** Number of samples in the last successful generation. */
  std::size_t size() const noexcept { return count_; }
  /**
   * Return the copied entity identifier.
   * @throws std::out_of_range when row is outside [0, size()).
   */
  const std::string& entity_id(std::size_t row) const;
  /**
   * Return an owning copy of one solution, including its zero covariance.
   * @throws std::out_of_range when row is outside [0, size()).
   */
  NavigationSolution solution(std::size_t row) const;

 private:
  std::size_t capacity_;
  std::size_t count_{};
  std::vector<double> numbers_;
  std::vector<double> staged_numbers_;
  std::vector<std::int64_t> times_;
  std::vector<std::int64_t> staged_times_;
  std::vector<std::string> ids_;
  std::vector<std::string> staged_ids_;
};

}  // namespace flight_models
