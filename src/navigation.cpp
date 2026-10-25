#include "flight_models/navigation.h"
#include "math/coordinates/geometry.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>

namespace flight_models {
namespace {

bool finite(math::Vector3 value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

// Hash fixed-width little-endian identity inputs so streams do not depend on
// std::hash implementations or on whether the other error group is enabled.
std::uint64_t sample_seed(std::uint64_t seed, const std::string& id,
                          std::int64_t time_ns, std::uint64_t stream) noexcept {
  std::uint64_t hash = 1469598103934665603ULL ^ seed ^ stream;
  const auto mix = [&hash](std::uint8_t byte) {
    hash = (hash ^ byte) * 1099511628211ULL;
  };
  const auto time_bits = static_cast<std::uint64_t>(time_ns);
  for (unsigned i = 0; i < 8; ++i) mix(static_cast<std::uint8_t>(time_bits >> (8 * i)));
  for (unsigned char byte : id) mix(byte);
  return hash;
}

void add(math::Vector3& value, const std::array<double, 3>& offset) noexcept {
  value.x += offset[0];
  value.y += offset[1];
  value.z += offset[2];
}

void write_columns(double* columns, std::size_t capacity, std::size_t row,
                   math::Vector3 position, math::Vector3 velocity,
                   const math::Matrix3& attitude) {
  const math::Vector3 vectors[] = {position, velocity};
  for (std::size_t vector = 0; vector < 2; ++vector) {
    columns[(vector * 3) * capacity + row] = vectors[vector].x;
    columns[(vector * 3 + 1) * capacity + row] = vectors[vector].y;
    columns[(vector * 3 + 2) * capacity + row] = vectors[vector].z;
  }
  for (std::size_t component = 0; component < 9; ++component)
    columns[(6 + component) * capacity + row] = attitude[component];
}

}  // namespace

NavigationModel::NavigationModel(const NavigationConfig& config) : config_(config) {
  validate_navigation_config(config);

  // Resolve configuration once; disabled terms never enter arithmetic or RNG paths.
  const std::array<double, 3> zero{};
  position_bias_enabled_ = config.position_bias_ecef_m != zero;
  velocity_bias_enabled_ = config.velocity_bias_ecef_mps != zero;
  position_noise_enabled_ = config.position_sigma_m > 0;
  velocity_noise_enabled_ = config.velocity_sigma_mps > 0;
  zero_error_ = !position_bias_enabled_ && !velocity_bias_enabled_ &&
      !position_noise_enabled_ && !velocity_noise_enabled_;

  position_variance_ = config.position_sigma_m * config.position_sigma_m;
  velocity_variance_ = config.velocity_sigma_mps * config.velocity_sigma_mps;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    covariance_[axis * 6 + axis] = position_variance_;
    covariance_[(axis + 3) * 6 + axis + 3] = velocity_variance_;
  }
}

NavigationSolution NavigationModel::solution(const PhysicalState& state,
                                               const std::string& entity_id) const {
  if (entity_id.empty()) throw std::invalid_argument("navigation entity id must not be empty");
  NavigationSolution result = ideal_navigation_solution(state, "ideal_test");
  if (zero_error_) return result;

  if (position_bias_enabled_) add(result.position_ecef_m, config_.position_bias_ecef_m);
  if (velocity_bias_enabled_) add(result.velocity_ecef_mps, config_.velocity_bias_ecef_mps);

  if (position_noise_enabled_) {
    // normal_distribution reproducibility is scoped to the same standard-library
    // implementation and build; the engine and identity key remain explicit.
    std::mt19937_64 engine(sample_seed(config_.seed, entity_id, state.source_time_ns, 1));
    std::normal_distribution<double> normal(0, config_.position_sigma_m);
    result.position_ecef_m.x += normal(engine);
    result.position_ecef_m.y += normal(engine);
    result.position_ecef_m.z += normal(engine);
  }
  if (velocity_noise_enabled_) {
    std::mt19937_64 engine(sample_seed(config_.seed, entity_id, state.source_time_ns, 2));
    std::normal_distribution<double> normal(0, config_.velocity_sigma_mps);
    result.velocity_ecef_mps.x += normal(engine);
    result.velocity_ecef_mps.y += normal(engine);
    result.velocity_ecef_mps.z += normal(engine);
  }
  if (!finite(result.position_ecef_m) || !finite(result.velocity_ecef_mps))
    throw std::overflow_error("navigation sample overflow");
  result.covariance_ecef = covariance_;
  return result;
}

NavigationBatch::NavigationBatch(std::size_t capacity, const NavigationConfig& config)
    : capacity_(capacity), model_(config) {
  if (capacity == 0) throw std::invalid_argument("navigation capacity must be positive");
  if (capacity > std::numeric_limits<std::size_t>::max() / 15)
    throw std::length_error("navigation capacity is not representable");
  numbers_.resize(capacity * 15);
  staged_numbers_.resize(capacity * 15);
  times_.resize(capacity);
  staged_times_.resize(capacity);
  ids_.resize(capacity);
  staged_ids_.resize(capacity);
}

void NavigationBatch::generate(const Snapshot& snapshot) {
  const SnapshotRow* const rows = snapshot.rows();
  std::size_t enabled_count = 0;
  for (std::size_t row = 0; row < snapshot.size(); ++row) {
    if (rows[row].mode == EntityMode::disabled) continue;
    if (enabled_count == capacity_) throw std::length_error("navigation batch capacity exceeded");
    ++enabled_count;
  }

  std::size_t output_row = 0;
  for (std::size_t row = 0; row < snapshot.size(); ++row) {
    const SnapshotRow& source = rows[row];
    if (source.mode == EntityMode::disabled) continue;

    // The exact-copy path preserves ideal bits and avoids noise/covariance work.
    const PhysicalState& state = source.state;
    if (model_.is_zero_error()) {
      if (source.id.empty() || (source.mode != EntityMode::fixed && source.mode != EntityMode::moving) ||
          state.source_time_ns < 0 || !finite(state.position_ecef_m) || !finite(state.velocity_ecef_mps) ||
          !math::coordinates::is_proper_rotation(state.body_frd_to_ecef, 1e-6))
        throw std::invalid_argument("invalid physical state in navigation snapshot");
      write_columns(staged_numbers_.data(), capacity_, output_row,
                    state.position_ecef_m, state.velocity_ecef_mps, state.body_frd_to_ecef);
    } else {
      const auto solution = model_.solution(state, source.id);
      write_columns(staged_numbers_.data(), capacity_, output_row, solution.position_ecef_m,
                    solution.velocity_ecef_mps, state.body_frd_to_ecef);
    }
    staged_times_[output_row] = state.source_time_ns;
    staged_ids_[output_row] = source.id;
    ++output_row;
  }

  numbers_.swap(staged_numbers_);
  times_.swap(staged_times_);
  ids_.swap(staged_ids_);
  count_ = enabled_count;
}

const std::string& NavigationBatch::entity_id(std::size_t row) const {
  if (row >= count_) throw std::out_of_range("navigation row out of range");
  return ids_[row];
}

NavigationSolution NavigationBatch::solution(std::size_t row) const {
  if (row >= count_) throw std::out_of_range("navigation row out of range");
  NavigationSolution result;
  result.position_ecef_m = {numbers_[row], numbers_[capacity_ + row], numbers_[2 * capacity_ + row]};
  result.velocity_ecef_mps = {numbers_[3 * capacity_ + row], numbers_[4 * capacity_ + row],
                              numbers_[5 * capacity_ + row]};
  for (std::size_t component = 0; component < 9; ++component)
    result.body_frd_to_ecef[component] = numbers_[(6 + component) * capacity_ + row];
  result.source_time_ns = times_[row];
  result.covariance_ecef = model_.covariance();
  return result;
}

NavigationSolution ideal_navigation_solution(const PhysicalState& state,
                                               const std::string& selected_model) {
  if (selected_model != "ideal_test")
    throw std::invalid_argument("navigation model must be explicitly selected as ideal_test");
  if (state.source_time_ns < 0 || !finite(state.position_ecef_m) ||
      !finite(state.velocity_ecef_mps) ||
      !math::coordinates::is_proper_rotation(state.body_frd_to_ecef, 1e-6))
    throw std::invalid_argument("invalid physical state for ideal navigation solution");

  NavigationSolution result;
  result.position_ecef_m = state.position_ecef_m;
  result.velocity_ecef_mps = state.velocity_ecef_mps;
  result.body_frd_to_ecef = state.body_frd_to_ecef;
  result.source_time_ns = state.source_time_ns;
  return result;
}

}  // namespace flight_models
