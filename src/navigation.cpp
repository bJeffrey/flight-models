#include "flight_models/navigation.h"
#include "math/coordinates/geometry.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace flight_models {
namespace {

bool finite(math::Vector3 value) noexcept {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}


}  // namespace

NavigationSolution ideal_navigation_solution(const PhysicalState& state,
                                               const std::string& selected_model) {
  if (selected_model != "ideal_test") {
    throw std::invalid_argument("navigation model must be explicitly selected as ideal_test");
  }
  if (state.source_time_ns < 0 || !finite(state.position_ecef_m) ||
      !finite(state.velocity_ecef_mps) ||
      !math::coordinates::is_proper_rotation(state.body_frd_to_ecef, 1e-6)) {
    throw std::invalid_argument("invalid physical state for ideal navigation solution");
  }
  NavigationSolution result;
  result.position_ecef_m = state.position_ecef_m;
  result.velocity_ecef_mps = state.velocity_ecef_mps;
  result.body_frd_to_ecef = state.body_frd_to_ecef;
  result.source_time_ns = state.source_time_ns;
  return result;
}

IdealNavigationBatch::IdealNavigationBatch(std::size_t capacity,
                                           const std::string& selected_model)
    : capacity_(capacity) {
  if (selected_model != "ideal_test") {
    throw std::invalid_argument("navigation model must be explicitly selected as ideal_test");
  }
  if (capacity == 0) throw std::invalid_argument("navigation capacity must be positive");
  if (capacity > std::numeric_limits<std::size_t>::max() / 15) {
    throw std::length_error("navigation capacity is not representable");
  }
  numbers_.resize(capacity * 15);
  staged_numbers_.resize(capacity * 15);
  times_.resize(capacity);
  staged_times_.resize(capacity);
  ids_.resize(capacity);
  staged_ids_.resize(capacity);
}

void IdealNavigationBatch::generate(const Snapshot& snapshot) {
  const SnapshotRow* const rows = snapshot.rows();
  // Validate the full enabled set before staging any report rows.
  std::size_t enabled_count = 0;
  for (std::size_t row = 0; row < snapshot.size(); ++row) {
    if (rows[row].mode == EntityMode::disabled) continue;
    if (enabled_count == capacity_) throw std::length_error("navigation batch capacity exceeded");
    const SnapshotRow& source = rows[row];
    const PhysicalState& state = source.state;
    if (source.id.empty() ||
        (source.mode != EntityMode::fixed && source.mode != EntityMode::moving) ||
        state.source_time_ns < 0 || !finite(state.position_ecef_m) ||
        !finite(state.velocity_ecef_mps) || !math::coordinates::is_proper_rotation(state.body_frd_to_ecef, 1e-6)) {
      throw std::invalid_argument("invalid physical state in navigation snapshot");
    }
    ++enabled_count;
  }

  // Stage owning copies; string allocation failure leaves the published batch intact.
  std::size_t output_row = 0;
  for (std::size_t row = 0; row < snapshot.size(); ++row) {
    const SnapshotRow& source = rows[row];
    if (source.mode == EntityMode::disabled) continue;
    const PhysicalState& state = source.state;
    const math::Vector3 vectors[] = {state.position_ecef_m, state.velocity_ecef_mps};
    for (std::size_t vector = 0; vector < 2; ++vector) {
      const math::Vector3 value = vectors[vector];
      staged_numbers_[(vector * 3) * capacity_ + output_row] = value.x;
      staged_numbers_[(vector * 3 + 1) * capacity_ + output_row] = value.y;
      staged_numbers_[(vector * 3 + 2) * capacity_ + output_row] = value.z;
    }
    for (std::size_t component = 0; component < 9; ++component) {
      staged_numbers_[(6 + component) * capacity_ + output_row] = state.body_frd_to_ecef[component];
    }
    staged_times_[output_row] = state.source_time_ns;
    staged_ids_[output_row] = source.id;
    ++output_row;
  }

  // Publish all columns together only after the complete copy succeeds.
  numbers_.swap(staged_numbers_);
  times_.swap(staged_times_);
  ids_.swap(staged_ids_);
  count_ = enabled_count;
}

const std::string& IdealNavigationBatch::entity_id(std::size_t row) const {
  if (row >= count_) throw std::out_of_range("navigation row out of range");
  return ids_[row];
}

NavigationSolution IdealNavigationBatch::solution(std::size_t row) const {
  if (row >= count_) throw std::out_of_range("navigation row out of range");
  NavigationSolution result;
  result.position_ecef_m = {numbers_[row], numbers_[capacity_ + row], numbers_[2 * capacity_ + row]};
  result.velocity_ecef_mps = {numbers_[3 * capacity_ + row], numbers_[4 * capacity_ + row],
                              numbers_[5 * capacity_ + row]};
  for (std::size_t component = 0; component < 9; ++component) {
    result.body_frd_to_ecef[component] = numbers_[(6 + component) * capacity_ + row];
  }
  result.source_time_ns = times_[row];
  return result;
}

}  // namespace flight_models
