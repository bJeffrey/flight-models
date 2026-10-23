# Physical State And Simulation Time Contract

This reference fixes the local state/time conventions for FM-CONTRACT-1. The
[implementation plan](implementation_plan.md) owns work status. JSON files under
`contracts/` describe test specimens, not another runtime network protocol.
Production adapters must consume the existing Protobuf/UCI/DIS contracts.

## Physical state and identity

| Item | Selected convention |
| --- | --- |
| Position | WGS-84 Earth-centered Earth-fixed (ECEF) metres, float64 inside the core. Geodetic altitude is height above ellipsoid (HAE), never implicitly MSL. |
| Velocity / acceleration | ECEF derivatives in m/s and m/s². Preserve float64 internally; apply DIS float32 rounding only in the adapter. |
| Attitude | Row-major 3×3 proper rotation from body forward/right/down to ECEF. A column is one body axis expressed in ECEF. Heading/yaw conversions belong at adapter/guidance boundaries. |
| Angular velocity | Body forward/right/down components in rad/s; distinct from Euler-angle derivatives. |
| Time | Signed int64 elapsed scenario nanoseconds, beginning at zero. A physical snapshot is coherent at one source time, run and timeline epoch. |
| Identity | Explicit platform instance, DIS site/application/entity triplet, platform SystemID UUID and navigation-source SubsystemID UUID mapping supplied by scenario integration. Never derive an operational identity from an array slot, aircraft type or marking. |
| Storage | Stable identity-to-slot mapping with generation counters for reused slots. Spawn/remove/restore cannot make an old handle refer to a different entity. No slot identity crosses an adapter. |
| Publication | Adapters read completed immutable/versioned snapshots, not partially updated numerical columns. Source time remains separate from encoding/publication time. |

The physical-state specimen fixes frame, identity and time shape; it is not a
complete aircraft model or navigation solution. Model-profile selection, guidance
limits and navigation error evolution remain implementation prerequisites.
The nominal specimen is synthetic: an equatorial point at HAE 1000 m moving east
at 200 m/s, with level body axes. Its values are test inputs, not aircraft limits.

Use implemented shared math in `math/include/math/coordinates/geodetic.hpp` for
geodetic/ECEF/ENU conversion and NED rotation; `math/core/types.hpp` defines the
row-major matrix convention. The similarly named legacy
`include/coordinates/{quaternions,rotations}.hpp` and
`include/tracking_math/{covariance,fusion_primitives}.hpp` are empty placeholders,
not reusable implementations. Numerical backends are standard C++, Eigen and
Intel MKL behind a stable developer-facing API, following the AST reference in
the owning plan. No backend is yet implemented/qualified in flight-models.
Keep efficient small-update and large-batch paths separate from backend choice;
benchmark their crossover when adding actual kernels. No quaternion API is
selected by this contract.

## Clock and scheduling

The wire input remains `ai-bm-sim/schemas/sim_time.proto::SimTimeTick` on the
simulation control plane. The specimen projects only fields needed to verify
advancement; omitting rate/mode is deliberate because domain motion does not
need wall pacing or FTRT awareness.

- Interpret advancing ticks as the interval `(t_us - dt_us, t_us]`, not an
  integration step starting at `t_us`. Convert microseconds to nanoseconds with
  checked integer multiplication; a coarse clock does not quantize internal
  scheduled events or fine measurement offsets.
- Advance only once for a contiguous `(run_id, timeline_epoch, tick_id)`.
  Initial startup is at zero; joining an already-advanced run requires a coherent
  restored state/input cursor, not integration from an arbitrary first tick.
  Identical replay is idempotent. A missing interval or conflicting replay
  suspends advancement until recovered; never integrate a guessed gap.
- Heartbeats and pause/resume announcements have `dt_us=0` and do not advance
  the integration boundary. Pause allows already-running work to complete;
  future interval admission stops. No ordinary per-tick acknowledgement is added.
- Scenario actions use elapsed time: a turn due at 30 seconds remains due at
  elapsed 30 seconds. Split integration at scheduled events and configured
  substep boundaries; separate integration, DIS, PRD and collection cadences.
- Map `scenario_epoch_utc + elapsed` only when formatting standard absolute
  timestamps. UCI header creation time and navigation validity time are distinct;
  delayed forwarding preserves the original header and source timestamps.
- An authorized checkpoint restore quiesces the old epoch, restores state,
  action execution/cursors and random-stream state, then commits a strictly new
  epoch with tick ID zero. Restored elapsed time is retained. A clock message
  alone does not authorize restore or prove checkpoint application. Old-epoch
  work must not run concurrently with new-epoch work.

The clock specimen starts at zero, advances to 30 seconds, pauses/resumes,
advances to 60 seconds, restores to 30 seconds in a new epoch, then continues.
`initialize`, `announce` and `restore` are fixture labels, not new wire commands.
The test-only reference checker is retired in favor of production time-adapter
calls at FM-RUNTIME-1; the same fixtures remain regression inputs.

## Sensor inputs and source readiness

Sensor-models needs physical DIS for targets **and sensor hosts**, plus reported
PRD navigation for the configured receivers. DIS supports propagation/physical
geometry; PRD supplies reported receiver state, validity time and covariance.
A target does not need a PRD merely to be sensed. Missing receiver navigation
must not be filled from truth. Ideal-navigation tests require an explicitly
selected ideal profile; they do not establish realistic navigation accuracy.

Flight-models publishes each reporting platform's PRD to that platform's local
ASB. A sensor simulation process serving several platforms may connect to their
local ASBs while retaining per-platform state/output ownership. This does not
make every PRD available to operational consumers: selected offboard delivery
still passes through comms-models. DIS truth stays outside modeled impairments.

The existing sensor input adapter accepts absolute DIS hour-relative timestamps
and aligns them to the nearest hour around authoritative scenario UTC within a
configured skew strictly below 30 minutes. It rejects relative mode. Preserve
that contract and qualify adapter rounding/wrap independently of fine TDOA
precision; DIS timestamp resolution is not a TDOA accuracy requirement.

Before a collection, readiness must establish the configured target/receiver
set, actual physical samples covering its required source times, receiver PRD
validity/covariance and any required emitter-source state. A tick, silence or a
cross-channel publication watermark does not prove UDP arrival. Apply bounded
admission, freshness/prediction and missing-source policy per collection; suppress
an incomplete collection rather than substituting truth or waiting without bound.

DIS has no run/epoch field. Current sensor timestamp alignment attaches its
current processing epoch; it cannot independently identify a delayed packet from
an earlier epoch. New cross-service epoch/restore handling is deferred until after SuperCell
deprecation. SIM-TIME-FM-1 will then select and test transport-incarnation fencing
and source-epoch correlation; this is not a prerequisite for replacing SuperCell
in fresh isolated runs. Do not add private fields to DIS/UCI or claim a clock-only reset
solves this. Initial live comparisons must use separate backend runs with
isolated transport/identities and one authoritative truth publisher per entity.

## Remaining integration decisions

The [adapter inventory](adapter_contract.md) records the accepted route-planning
consumer mapping: the header SystemID is the platform, and either PositionSource
SystemID or SubsystemID is the contributing navigation source. Correlated adapter
and dispatcher tests now cover the EGI choice, covariance and source time.
Reported yaw remains required; flight-models must emit modeled navigation
orientation. SuperCell's yaw-less PRD is not silently converted using ground
course. This closes the local consumer mapping gap, not runtime interoperability
or realistic navigation-model acceptance.

Do not select an authorization carrier or emission PDU by analogy. Route
applicability/plan-version/control permission must be reconciled with the owning
mission/execution services, and emitter-source transfer must be audited with
sensor-models. These gates remain open in FM-CONTRACT-1; no new source protocol,
error model or runtime producer is introduced by this slice.
