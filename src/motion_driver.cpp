#include "flight_models/motion_driver.h"

#include "flight_models/bounded_motion.h"
#include "math/coordinates/geodetic.hpp"
#include "math/coordinates/geometry.hpp"
#include "math/coordinates/line_of_sight.hpp"
#include "math/geometry/swept.hpp"
#include "math/numerical/angle_wrapping.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace flight_models {
namespace {

using math::Vector3;

bool finite(const Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

/** Evaluate route bearing using the shared ECEF/local LOS convention. */
double bearing_to(Vector3 from, Vector3 to) {
  const auto location = math::coordinates::ecef_to_geodetic(from);
  const auto rotation = math::coordinates::ecef_to_enu_rotation(location);
  return math::coordinates::line_of_sight_aer(from, to, rotation).azimuth_rad;
}

/** Find the next zero-anchored deadline without overflowing scenario time. */
std::int64_t next_deadline(std::int64_t now, std::int64_t period, std::int64_t terminal) {
  const std::int64_t anchor = now - now % period;
  if (anchor > std::numeric_limits<std::int64_t>::max() - period) return terminal;
  return std::min(terminal, anchor + period);
}

}  // namespace

struct MotionDriver::Impl {
  struct ProgramState {
    EntityProgram program;
    std::vector<double> waypoint_altitudes_m; // Derived once from immutable ECEF route inputs.
    std::size_t command_index{};
    std::size_t waypoint_index{};
  };

  StateStore& store;
  std::size_t profile_index;
  MotionProfile profile;
  DriverLimits limits;
  std::vector<ProgramState> programs;
  std::vector<Handle> cohort;
  std::vector<std::size_t> row_program;
  StateBatch staged;
  StateBatch probe;
  std::vector<std::int64_t> substeps;
  std::vector<std::int64_t> probe_substeps;
  std::vector<std::int64_t> probe_contact_times;
  std::vector<TimedStateSample> output_samples;
  std::vector<TimedStateSample> provisional_samples;
  std::vector<WaypointTransition> output_transitions;
  std::vector<WaypointTransition> provisional_transitions;
  std::vector<std::size_t> provisional_commands;
  std::vector<std::size_t> provisional_waypoints;
  std::int64_t now{};
  std::uint64_t membership_revision{};
  bool initialized{};

  Impl(StateStore& source, std::size_t selected_profile, MotionProfile selected,
       DriverLimits bounds, std::vector<EntityProgram> supplied)
      : store(source), profile_index(selected_profile), profile(std::move(selected)), limits(bounds),
        row_program(source.size(), std::numeric_limits<std::size_t>::max()),
        staged(source.size()), probe(source.size()), substeps(source.size(), 0), probe_substeps(source.size(), 0),
        probe_contact_times(source.size(), std::numeric_limits<std::int64_t>::max()),
        membership_revision(source.membership_revision()) {
    validate_profile();
    if (profile_index >= source.profile_count()) throw std::invalid_argument("profile index is out of range");
    if (limits.max_samples == 0 || limits.max_events == 0 || limits.max_transitions == 0) {
      throw std::invalid_argument("driver work limits must be positive");
    }
    Snapshot snapshot(source.size());
    source.snapshot(snapshot);
    std::unordered_map<std::size_t, const SnapshotRow*> by_slot;
    by_slot.reserve(snapshot.size());
    cohort.reserve(snapshot.size());
    std::size_t cohort_size = 0;
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
      const SnapshotRow& row = snapshot.rows()[i];
      if (row.profile_index == profile_index && row.mode != EntityMode::disabled) {
        ++cohort_size;
        cohort.push_back(row.handle);
        by_slot.emplace(row.handle.slot, &row);
        if (row.state.source_time_ns != 0) throw std::invalid_argument("driver requires a fresh zero-time cohort");
      }
    }
    if (supplied.size() != cohort_size) throw std::invalid_argument("programs must cover the enabled profile cohort");
    programs.reserve(supplied.size());
    std::unordered_map<std::size_t, std::size_t> program_by_slot;
    program_by_slot.reserve(supplied.size());
    std::size_t command_count = 0;
    std::size_t waypoint_count = 0;
    for (EntityProgram& program : supplied) {
      const auto it = by_slot.find(program.entity.slot);
      if (it == by_slot.end() || it->second->handle != program.entity ||
          !program_by_slot.emplace(program.entity.slot, programs.size()).second) {
        throw std::invalid_argument("program has stale, duplicate, disabled, or foreign entity handle");
      }
      const SnapshotRow& row = *it->second;
      if (row.mode == EntityMode::fixed && (!program.waypoints.empty() || !program.commands.empty())) {
        throw std::invalid_argument("fixed entities cannot have routes or commands");
      }
      if (!program.waypoints.empty() && !program.commands.empty()) {
        throw std::invalid_argument("route and timed commands cannot be combined");
      }
      if (program.commands.size() > limits.max_commands - command_count ||
          program.waypoints.size() > limits.max_waypoints - waypoint_count) {
        throw std::length_error("driver command or waypoint bound exceeded");
      }
      command_count += program.commands.size();
      waypoint_count += program.waypoints.size();
      std::unordered_set<std::uint64_t> sequences;
      for (std::size_t i = 0; i < program.commands.size(); ++i) {
        const auto& command = program.commands[i];
        if (command.time_ns < 0 ||
            !std::isfinite(command.desired.heading_rad) ||
            !std::isfinite(command.desired.horizontal_speed_mps) ||
            !std::isfinite(command.desired.altitude_hae_m) ||
            command.desired.horizontal_speed_mps < profile.flight_envelope.min_speed_mps ||
            command.desired.horizontal_speed_mps > profile.flight_envelope.max_speed_mps ||
            command.desired.altitude_hae_m < profile.flight_envelope.min_altitude_hae_m ||
            command.desired.altitude_hae_m > profile.flight_envelope.max_altitude_hae_m ||
            !sequences.insert(command.sequence).second ||
            (i != 0 && std::tie(program.commands[i - 1].time_ns, program.commands[i - 1].sequence) >=
                          std::tie(command.time_ns, command.sequence))) {
          throw std::invalid_argument("commands must be valid and strictly ordered by time and sequence");
        }
      }
      std::vector<double> waypoint_altitudes;
      waypoint_altitudes.reserve(program.waypoints.size());
      for (const Waypoint& waypoint : program.waypoints) {
        if (!finite(waypoint.position_ecef_m)) throw std::invalid_argument("waypoint must be finite");
        const auto geo = math::coordinates::ecef_to_geodetic(waypoint.position_ecef_m);
        if (geo.altitude_m < profile.flight_envelope.min_altitude_hae_m ||
            geo.altitude_m > profile.flight_envelope.max_altitude_hae_m) {
          throw std::invalid_argument("waypoint HAE is outside the profile envelope");
        }
        waypoint_altitudes.push_back(geo.altitude_m);
      }
      programs.push_back({std::move(program), std::move(waypoint_altitudes), 0, 0});
    }

    // Snapshot is canonical ID order; it fixes batch rows and event tie ordering.
    for (std::size_t row = 0; row < cohort.size(); ++row) {
      row_program[row] = program_by_slot.at(cohort[row].slot);
    }
    provisional_commands.resize(programs.size());
    provisional_waypoints.resize(programs.size());
    provisional_samples.reserve(limits.max_samples);
    output_samples.reserve(limits.max_samples);
    provisional_transitions.reserve(limits.max_transitions);
    output_transitions.reserve(limits.max_transitions);
  }

  void validate_profile() const {
    const auto& e = profile.flight_envelope;
    const auto& p = profile.integration;
    if (p.max_substep_ns <= 0 || p.max_interval_ns <= 0 || p.max_substeps_per_interval <= 0 ||
        p.max_substep_ns > p.max_interval_ns || profile.publication.dis_period_ns <= 0 ||
        profile.publication.prd_period_ns <= 0 || !std::isfinite(profile.waypoint_capture_radius_m) ||
        profile.waypoint_capture_radius_m <= 0 || !std::isfinite(e.min_speed_mps) ||
        !std::isfinite(e.max_speed_mps) || !std::isfinite(e.max_acceleration_mps2) ||
        !std::isfinite(e.max_deceleration_mps2) || !std::isfinite(e.max_turn_rate_radps) ||
        !std::isfinite(e.max_climb_rate_mps) || !std::isfinite(e.max_descent_rate_mps) ||
        !std::isfinite(e.min_altitude_hae_m) || !std::isfinite(e.max_altitude_hae_m) ||
        e.min_speed_mps <= 0 || e.max_speed_mps < e.min_speed_mps ||
        e.max_acceleration_mps2 <= 0 || e.max_deceleration_mps2 <= 0 || e.max_turn_rate_radps <= 0 ||
        e.max_climb_rate_mps < 0 || e.max_descent_rate_mps < 0 ||
        e.max_altitude_hae_m < e.min_altitude_hae_m) {
      throw std::invalid_argument("invalid driver motion profile");
    }
  }

  void apply_due_commands(std::int64_t at, bool provisional = false) {
    MutableStateColumns columns = staged.columns();
    for (std::size_t row = 0; row < staged.size(); ++row) {
      ProgramState& p = programs[row_program[row]];
      std::size_t& cursor = provisional ? provisional_commands[row_program[row]] : p.command_index;
      while (cursor < p.program.commands.size() && p.program.commands[cursor].time_ns == at) {
        const DesiredMotion d = p.program.commands[cursor].desired;
        columns.heading_rad[row] = d.heading_rad;
        columns.horizontal_speed_mps[row] = d.horizontal_speed_mps;
        columns.altitude_hae_m[row] = d.altitude_hae_m;
        ++cursor;
      }
    }
  }

  void refresh_guidance() {
    MutableStateColumns columns = staged.columns();
    for (std::size_t row = 0; row < staged.size(); ++row) {
      ProgramState& p = programs[row_program[row]];
      const std::size_t waypoint_index = provisional_waypoints.empty()
          ? p.waypoint_index : provisional_waypoints[row_program[row]];
      if (waypoint_index < p.program.waypoints.size()) {
        const Vector3 current{columns.position_ecef_m.x[row], columns.position_ecef_m.y[row], columns.position_ecef_m.z[row]};
        const Waypoint& target = p.program.waypoints[waypoint_index];
        columns.heading_rad[row] = bearing_to(current, target.position_ecef_m);
        columns.altitude_hae_m[row] = p.waypoint_altitudes_m[waypoint_index];
      }
    }
  }

  void emit_samples(std::int64_t at, bool initial) {
    const bool physical_due = initial || at % profile.publication.dis_period_ns == 0;
    const bool navigation_due = initial || at % profile.publication.prd_period_ns == 0;
    for (int pass = 0; pass < 2; ++pass) {
      if ((pass == 0 && !physical_due) || (pass == 1 && !navigation_due)) continue;
      for (std::size_t row = 0; row < staged.size(); ++row) {
        const ProgramState& program = programs[row_program[row]];
        if (pass == 1 && !program.program.navigation_reporting) continue;
        if (provisional_samples.size() == limits.max_samples) throw std::length_error("sample bound exceeded");
        PhysicalState state;
        DesiredMotion desired;
        staged.read_state(row, state, desired);
        provisional_samples.push_back({program.program.entity,
            pass == 0 ? SampleKind::physical : SampleKind::navigation, state, desired});
      }
    }
  }
};

MotionDriver::MotionDriver(StateStore& store, std::size_t profile_index, MotionProfile profile,
                           DriverLimits limits, std::vector<EntityProgram> programs)
    : impl_(std::make_unique<Impl>(store, profile_index, std::move(profile), limits, std::move(programs))) {}
MotionDriver::~MotionDriver() = default;
MotionDriver::MotionDriver(MotionDriver&&) noexcept = default;
MotionDriver& MotionDriver::operator=(MotionDriver&&) noexcept = default;

void MotionDriver::advance_to(std::int64_t time_ns) {
  Impl& x = *impl_;
  if (x.store.membership_revision() != x.membership_revision) {
    throw std::invalid_argument("store cohort changed during motion driver lifetime");
  }
  if (time_ns < x.now || time_ns < 0 || time_ns - x.now > x.profile.integration.max_interval_ns) {
    throw std::invalid_argument("invalid driver advance interval");
  }
  if (x.initialized && time_ns == x.now) return;
  x.store.gather(x.cohort.data(), x.cohort.size(), x.staged);
  x.provisional_samples.clear();
  x.provisional_transitions.clear();
  for (std::size_t i = 0; i < x.programs.size(); ++i) {
    x.provisional_commands[i] = x.programs[i].command_index;
    x.provisional_waypoints[i] = x.programs[i].waypoint_index;
  }
  std::fill(x.substeps.begin(), x.substeps.end(), 0);
  std::int64_t current = x.now;
  std::size_t events = 0;
  auto count_event = [&]() {
    if (++events > x.limits.max_events) throw std::length_error("event bound exceeded");
  };
  // The implementation stages cursors separately, then publishes them only after store commit.
  if (!x.initialized) {
    count_event();
    advance_motion_batch(x.staged, x.profile, 0, x.substeps.data(), x.substeps.size());
    x.apply_due_commands(0, true);
    for (std::size_t row = 0; row < x.staged.size(); ++row) {
      const std::size_t pi = x.row_program[row];
      Impl::ProgramState& p = x.programs[pi];
      std::size_t& wi = x.provisional_waypoints[pi];
      if (x.staged.modes()[row] != EntityMode::moving || wi >= p.program.waypoints.size()) continue;
      PhysicalState state; DesiredMotion desired;
      x.staged.read_state(row, state, desired);
      while (wi < p.program.waypoints.size() &&
             math::geometry::first_segment_sphere_contact(
                 state.position_ecef_m, state.position_ecef_m,
                 p.program.waypoints[wi].position_ecef_m,
                 x.profile.waypoint_capture_radius_m) != std::nullopt) {
        count_event();
        if (x.provisional_transitions.size() == x.limits.max_transitions)
          throw std::length_error("initial waypoint transition bound exceeded");
        x.provisional_transitions.push_back({p.program.entity, wi, 0});
        ++wi;
      }
    }
    x.refresh_guidance();
    const MutableStateColumns columns = x.staged.columns();
    for (std::size_t row = 0; row < x.staged.size(); ++row) {
      if (columns.horizontal_speed_mps[row] < x.profile.flight_envelope.min_speed_mps ||
          columns.horizontal_speed_mps[row] > x.profile.flight_envelope.max_speed_mps ||
          columns.altitude_hae_m[row] < x.profile.flight_envelope.min_altitude_hae_m ||
          columns.altitude_hae_m[row] > x.profile.flight_envelope.max_altitude_hae_m) {
        throw std::invalid_argument("initial desired motion is outside the profile envelope");
      }
    }
    refresh_motion_controls(x.staged, x.profile);
    x.emit_samples(0, true);
  }
  while (current < time_ns) {
    const std::int64_t remaining = time_ns - current;
    std::int64_t boundary = x.profile.integration.max_substep_ns >= remaining
        ? time_ns : current + x.profile.integration.max_substep_ns;
    for (std::size_t row = 0; row < x.staged.size(); ++row) {
      const Impl::ProgramState& p = x.programs[x.row_program[row]];
      const std::size_t ci = x.provisional_commands[x.row_program[row]];
      if (ci < p.program.commands.size() && p.program.commands[ci].time_ns > current)
        boundary = std::min(boundary, p.program.commands[ci].time_ns);
    }
    const std::int64_t dis = next_deadline(current, x.profile.publication.dis_period_ns, time_ns);
    const std::int64_t prd = next_deadline(current, x.profile.publication.prd_period_ns, time_ns);
    boundary = std::min(boundary, std::min(dis, prd));
    count_event();

    x.probe.copy_from(x.staged);
    std::fill(x.probe_substeps.begin(), x.probe_substeps.end(), 0);
    advance_motion_batch(x.probe, x.profile, boundary, x.probe_substeps.data(), x.probe_substeps.size());
    for (std::size_t r = 0; r < x.substeps.size(); ++r) {
      if (x.probe_substeps[r] > x.profile.integration.max_substeps_per_interval - x.substeps[r]) {
        throw std::invalid_argument("motion substep budget exceeded across event probes");
      }
      x.substeps[r] += x.probe_substeps[r];
    }
    std::int64_t contact_time = boundary;
    bool found_contact = false;
    std::fill(x.probe_contact_times.begin(), x.probe_contact_times.end(),
              std::numeric_limits<std::int64_t>::max());
    for (std::size_t row = 0; row < x.staged.size(); ++row) {
      const Impl::ProgramState& p = x.programs[x.row_program[row]];
      const std::size_t wi = x.provisional_waypoints[x.row_program[row]];
      if (wi >= p.program.waypoints.size() || x.staged.modes()[row] != EntityMode::moving) continue;
      PhysicalState a, b; DesiredMotion da, db;
      x.staged.read_state(row, a, da); x.probe.read_state(row, b, db);
      const auto fraction = math::geometry::first_segment_sphere_contact(
          a.position_ecef_m, b.position_ecef_m, p.program.waypoints[wi].position_ecef_m,
          x.profile.waypoint_capture_radius_m);
      if (!fraction) continue;
      const std::int64_t duration = boundary - current;
      const std::int64_t offset = static_cast<std::int64_t>(std::ceil(*fraction * static_cast<double>(duration)));
      const std::int64_t candidate = std::min(boundary, current + offset);
      x.probe_contact_times[row] = candidate;
      if (!found_contact || candidate < contact_time) { contact_time = candidate; found_contact = true; }
    }
    if (found_contact && contact_time < boundary) {
      advance_motion_batch(x.staged, x.profile, contact_time, x.substeps.data(), x.substeps.size());
      current = contact_time;
    } else {
      x.staged.copy_from(x.probe);
      current = boundary;
    }
    if (found_contact && current == contact_time) {
      for (std::size_t row = 0; row < x.staged.size(); ++row) {
        const std::size_t pi = x.row_program[row];
        Impl::ProgramState& p = x.programs[pi];
        std::size_t& wi = x.provisional_waypoints[pi];
        if (wi >= p.program.waypoints.size() || x.probe_contact_times[row] != current) continue;
        PhysicalState state; DesiredMotion desired;
        x.staged.read_state(row, state, desired);
        bool swept_capture = true;
        while (wi < p.program.waypoints.size()) {
          const Waypoint& waypoint = p.program.waypoints[wi];
          if (!swept_capture && math::geometry::first_segment_sphere_contact(
                  state.position_ecef_m, state.position_ecef_m, waypoint.position_ecef_m,
                  x.profile.waypoint_capture_radius_m) == std::nullopt) break;
          count_event();
          if (x.provisional_transitions.size() == x.limits.max_transitions)
            throw std::length_error("waypoint transition bound exceeded");
          x.provisional_transitions.push_back({p.program.entity, wi, current});
          ++wi;
          swept_capture = false;
        }
        if (wi == p.program.waypoints.size()) {
          const double achieved_heading = bearing_to(state.position_ecef_m,
              {state.position_ecef_m.x + state.velocity_ecef_mps.x,
               state.position_ecef_m.y + state.velocity_ecef_mps.y,
               state.position_ecef_m.z + state.velocity_ecef_mps.z});
          desired.heading_rad = achieved_heading;
          x.staged.write_state(row, state, desired);
        }
      }
    }
    // Sampling source times advance for fixed rows too; their pose and controls stay fixed.
    for (std::size_t row = 0; row < x.staged.size(); ++row) {
      if (x.staged.modes()[row] == EntityMode::fixed) {
        PhysicalState state; DesiredMotion desired;
        x.staged.read_state(row, state, desired);
        state.source_time_ns = current;
        x.staged.write_state(row, state, desired);
      }
    }
    x.apply_due_commands(current, true);
    x.refresh_guidance();
    refresh_motion_controls(x.staged, x.profile);
    if (current % x.profile.publication.dis_period_ns == 0 ||
        current % x.profile.publication.prd_period_ns == 0) {
      x.emit_samples(current, false);
    }
    if (current == boundary && current < time_ns) continue;
  }
  x.store.commit(x.staged);
  for (std::size_t i = 0; i < x.programs.size(); ++i) {
    x.programs[i].command_index = x.provisional_commands[i];
    x.programs[i].waypoint_index = x.provisional_waypoints[i];
  }
  x.now = time_ns;
  x.initialized = true;
  x.output_samples.swap(x.provisional_samples);
  x.output_transitions.swap(x.provisional_transitions);
}

std::int64_t MotionDriver::time_ns() const noexcept { return impl_->now; }
const std::vector<TimedStateSample>& MotionDriver::samples() const noexcept { return impl_->output_samples; }
const std::vector<WaypointTransition>& MotionDriver::transitions() const noexcept { return impl_->output_transitions; }
std::size_t MotionDriver::waypoint_progress(Handle entity) const {
  for (const auto& program : impl_->programs) if (program.program.entity == entity) return program.waypoint_index;
  throw std::invalid_argument("entity is not in this motion driver cohort");
}

}  // namespace flight_models
