/** @file
 * Reported navigation in ECEF, with optional synthetic bias and Gaussian errors.
 */
#pragma once

#include "flight_models/state_store.h"
#include "flight_models/motion_profile.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace flight_models {

/**
 * Reported ECEF state and covariance for one navigation sample.
 * Covariance is ECEF [position, velocity], row-major; the configured model
 * reports independent per-axis uncertainty and excludes fixed bias.
 */
struct NavigationSolution {
  math::Vector3 position_ecef_m{}; /**< ECEF position, metres. */
  math::Vector3 velocity_ecef_mps{}; /**< ECEF velocity, metres/second. */
  math::Matrix3 body_frd_to_ecef{}; /**< Row-major body-FRD to ECEF DCM. */
  math::Covariance6 covariance_ecef{}; /**< Row-major [position, velocity] ECEF covariance; zero for ideal_test. */
  std::int64_t source_time_ns{}; /**< Simulation validity time, nanoseconds. */
};

/**
 * Copy one physical sample as a zero-covariance ideal_test navigation solution.
 * @param state Completed physical state in ECEF metres and metres/second, with
 *        body-FRD attitude expressed as a proper rotation into ECEF.
 * @param selected_model Must be exactly "ideal_test"; no alternate error model
 *        is inferred or synthesized.
 * @return Owning ECEF position, velocity, attitude and source time; covariance
 *         is zero under the exact-copy ideal assumption.
 * @throws std::invalid_argument for another model or invalid physical state.
 */
[[nodiscard]] NavigationSolution ideal_navigation_solution(
    const PhysicalState& state, const std::string& selected_model);

/** Reusable validated navigation model with identity/time-keyed sampling. */
class NavigationModel {
 public:
  /**
   * Validate configuration and cache enabled terms/covariance once.
   * @param config Explicit model, ECEF bias vectors, sigmas and seed; copied.
   * @throws std::invalid_argument For an unsupported model, invalid numerical
   *         settings, nonzero ideal_test errors, or unrepresentable variance.
   */
  explicit NavigationModel(const NavigationConfig& config);
  /**
   * Produce reported ECEF state without modifying the physical input.
   * @param state Physical ECEF position/velocity, attitude and validity time.
   * @param entity_id Nonempty stable identity used with seed/time for sampling.
   * @return Owning reported state and covariance; attitude/time are unchanged.
   *         Repeated keys reproduce samples within the same toolchain/build.
   * @throws std::invalid_argument For invalid physical state or empty identity.
   * @throws std::overflow_error If perturbation produces nonfinite output.
   */
  [[nodiscard]] NavigationSolution solution(const PhysicalState& state, const std::string& entity_id) const;
  /** Return the fixed covariance shared by every solution from this model. */
  const math::Covariance6& covariance() const noexcept { return covariance_; }
  /** True when all bias and noise terms are zero. */
  bool is_zero_error() const noexcept { return zero_error_; }
 private:
  NavigationConfig config_;
  double position_variance_{};
  double velocity_variance_{};
  bool position_bias_enabled_{};
  bool velocity_bias_enabled_{};
  bool position_noise_enabled_{};
  bool velocity_noise_enabled_{};
  bool zero_error_{};
  math::Covariance6 covariance_{};
};

/**
 * Fixed-capacity transactional batch over owning snapshots.
 * Numeric values use reusable structure-of-arrays columns. Calls must be
 * serialized; IDs remain valid until the next successful generate().
 */
class NavigationBatch {
 public:
  /**
   * Allocate reusable SoA buffers and validate configuration once.
   * @param capacity Maximum enabled rows, positive and fixed for this batch.
   * @param config Navigation error settings, copied into the batch model.
   * @throws std::invalid_argument For zero capacity or invalid configuration.
   * @throws std::length_error If the numeric storage size is not representable.
   */
  NavigationBatch(std::size_t capacity, const NavigationConfig& config);
  /**
   * Stage enabled rows in snapshot order and publish after every row succeeds.
   * @param snapshot Owning physical rows; disabled entities are omitted.
   * @throws std::length_error If enabled rows exceed the fixed capacity.
   * @throws std::invalid_argument For an invalid input state/identity.
   * @throws std::overflow_error If perturbation produces nonfinite output.
   * All failures leave the previous published batch intact.
   */
  void generate(const Snapshot& snapshot);
  /** Number of rows in the last successful generation. */
  std::size_t size() const noexcept { return count_; }
  /** Return the copied ID; throws out_of_range for an invalid row. */
  const std::string& entity_id(std::size_t row) const;
  /** Return an owning solution copy; throws out_of_range for an invalid row. */
  NavigationSolution solution(std::size_t row) const;
 private:
  std::size_t capacity_;
  NavigationModel model_;
  std::size_t count_{};
  std::vector<double> numbers_, staged_numbers_;
  std::vector<std::int64_t> times_, staged_times_;
  std::vector<std::string> ids_, staged_ids_;
};

}  // namespace flight_models
