/** @file
 * Transactional advancement for the bounded synthetic point-motion model.
 */
#pragma once

#include "flight_models/motion_profile.h"
#include "flight_models/state_store.h"

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace flight_models {

/**
 * Advance all moving entities bound to one profile to a common simulation time.
 *
 * @param store Single-owner live state store; its live cohort changes only after
 *        every selected entity has integrated successfully.
 * @param scratch Reusable batch with capacity for the entire moving cohort.
 *        Workspace contents are unspecified after failure; gather again before
 *        using it elsewhere. Callers must serialize access with store mutation.
 * @param profile_index Profile registry index used to select the cohort.
 * @param profile Validated immutable motion limits and integration policy.
 * @param end_time_ns Common endpoint in scenario nanoseconds.
 * @throws std::invalid_argument for invalid policy, incoherent state, invalid
 *         desired motion, invalid time, or numerical failure.
 * @throws std::length_error if the cohort exceeds scratch capacity.
 *         Work-budget exhaustion is reported as std::invalid_argument.
 *
 * Desired motion remains unchanged. The midpoint integrator splits at control
 * saturation events and uses WGS-84 geodesy for local-frame rotation and HAE.
 * Endpoint acceleration and body angular rate are analytic derivatives of the
 * bounded controls plus local-frame transport; control derivatives are zero
 * immediately after a target has been captured. Climb/descent rate changes at
 * control onset and altitude capture without a vertical-acceleration model;
 * endpoint derivatives are right-hand regular values away from those jumps,
 * not an impulse model for the command boundaries.
 */
void advance_motion(StateStore& store, StateBatch& scratch,
                    std::size_t profile_index, const MotionProfile& profile,
                    std::int64_t end_time_ns);

/**
 * Integrate moving rows already staged in a batch without committing them.
 * @param scratch Gathered rows; moving rows are integrated, other modes are unchanged.
 * @param profile Validated motion limits and numerical policy.
 * @param end_time_ns Common endpoint in scenario nanoseconds.
 * @param cumulative_substeps Optional per-row in/out work totals for this overall interval;
 *        null uses a local per-call counter, as needed by the legacy wrapper.
 * @param count_capacity Number of counters available when the pointer is nonnull.
 * @throws std::invalid_argument for invalid policy, state, target, time, numerical
 *         failure, or exhausted cumulative work budget.
 * @throws std::length_error when the counter array is too small.
 */
void advance_motion_batch(StateBatch& scratch, const MotionProfile& profile,
                          std::int64_t end_time_ns,
                          std::int64_t* cumulative_substeps,
                          std::size_t count_capacity);

/**
 * Refresh right-hand control-derived velocity and derivatives after a target change
 * at the current event time. Vertical-rate changes follow the synthetic model's
 * explicit instantaneous transition; no vertical acceleration is modeled.
 */
void refresh_motion_controls(StateBatch& scratch, const MotionProfile& profile);

}  // namespace flight_models
