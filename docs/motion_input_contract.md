# Initial Bounded Motion And Ordinary-Run Inputs

This is the initial transport-independent contract for FM-SOA-1/FM-DYNAMICS-1.
The [owning plan](implementation_plan.md) records acceptance and remaining
operational gates. Configuration is
[`synthetic_fixed_wing.json`](../config/profiles/synthetic_fixed_wing.json),
validated against [`motion_profile.schema.json`](../config/motion_profile.schema.json)
and the C++ loader's cross-field checks. Duration/work-count fields use exact
integer JSON tokens, not floating-point tokens such as `1.0`. There are no
implicit tuning defaults.

## Model and limits

The selected model is synthetic bounded point kinematics without wind. Speed
means horizontal ground speed; climb/descent are independent HAE rates. The
limits do not describe total airspeed, stall, thrust, lift, bank authority,
terrain clearance or a qualified aircraft. Initial envelope values match the
existing generic route-planning test profile:
`route-planning/config/vehicle_profiles/generic_fixed_wing.json`.
Flight-models owns execution limits; the planner consumes a corresponding
planning subset and must not allow a wider envelope for the same installation.
This checkpoint does not migrate that cross-repository configuration binding.

| Quantity | Initial synthetic setting |
| --- | --- |
| Horizontal ground speed | 92.6–205.8 m/s |
| Acceleration / deceleration magnitude | 4 m/s² each |
| Heading rate magnitude | 0.05235987755982989 rad/s (3°/s) |
| Climb / descent magnitude | 30 m/s each |
| Altitude | 300–15000 m WGS-84 HAE |
| Waypoint capture radius | 150 m, 3D distance |
| Maximum numerical substep | 10 ms scenario time |
| Maximum admitted clock interval | 2 s scenario time |
| Maximum substeps per interval | 4096, including extra event splits |
| DIS / PRD sampling periods | 100 ms / 200 ms scenario time |
| Navigation | Explicit `ideal_test`: exact reported state, zero covariance |

Capture, integration, work-budget and publication values are synthetic test
settings. They are not inferred sensor specifications or performance claims.
Ideal navigation is a bounded initial test profile; nonideal error modeling
remains FM-NAV-PRD-1 before realistic navigation acceptance.

## Core inputs and guidance

The core receives coherent initial physical state, the selected profile, elapsed
intervals and desired-motion inputs already admitted by the VMS/guidance layer.
It receives no UCI, DIS, wall-clock rate, permission payload or simulation truth
from an operational consumer. Test fixtures may provide desired commands directly;
this does not create a new operational command/authorization protocol.

Initial state and desired speed/altitude must be finite and within the selected
profile. Unsupported/out-of-envelope targets are rejected explicitly. Admitted
commands change desired state; achieved speed, altitude and heading evolve under
rate bounds, without overshoot or instantaneous setpoint application. Heading
is clockwise from true north in radians. Normalize with the implemented shared
`math::numerical` angle wrapping; select the shortest turn, with a clockwise
choice at an exact half-turn. Zero-wind kinematic body yaw follows the modeled
heading; pitch follows climb angle and roll is not modeled (zero). This is not
a banked aerodynamic model.

Waypoints are ordered WGS-84 HAE points or points in a named ENU frame with its
explicit geodetic origin. Convert through shared geodesy into canonical ECEF;
reject missing origins and unsupported MSL/relative datums. Capture uses the
3D radius and the swept integrated path, so stepping past a waypoint cannot skip
capture. Emit transitions in waypoint order. Detailed-route arrival windows,
loiter and unsupported path constraints must be preserved for VMS admission and
rejected until implemented, rather than ignored or reported as successful.
Their operational adapter/control semantics remain a later integration gate;
they do not block transport-free SoA and motion kernels.

A scheduled desired-state change at elapsed 30 s applies at 30 s. Advance to that
boundary under the old command, apply the change, then advance subsequent time
under the new command. At a coincident deadline, apply due commands and guidance
transitions before sampling outputs. Position/velocity stay continuous; any
instantaneous control/acceleration change is represented by the post-event
snapshot. Simultaneous commands for one entity require the upstream ordered
command sequence; conflicting command identities are rejected before execution.

## Integration and publication

Consume the existing authoritative SimTimeTick interval `(t-dt,t]`. Split at the
maximum substep, scheduled command, waypoint transition and output deadlines.
The initial numerical method is explicit midpoint integration with rate-limited
control evolution; qualify its error at aligned elapsed times against smaller
substeps and analytical straight/turning fixtures in FM-DYNAMICS-1.
Integration, DIS and PRD periods need not divide one another or the external
clock interval. Never round an event to the next tick or replace all due samples
with the latest state. Subdivision and event-generation work is bounded by the
configured substep budget; interval rejection/failure must not publish a partially
completed interval as though it succeeded.

Create a coherent initial snapshot at elapsed zero, then sample DIS and PRD on
their independent schedules. Only configured reporting platforms produce local
PRDs; DIS truth covers the configured physical entities, including targets that
have no PRD. Platform installations/reporting membership come from ai-bm-sim's
scenario, not from the fact that an entity occupies a SoA slot. Sending a local
PRD does not implicitly share it offboard: selected operational traffic uses
comms-models.

Adapters retain snapshot source times and query the configured simulation time
provider for message-header creation time. Source validity and creation time
are distinct. Domain code does not inspect FTRT rate. DIS encoding precision is
qualified separately from the internal integer-nanosecond source time. Reuse
existing run/epoch/tick fields; new epoch/restore handling remains deferred until
after SuperCell deprecation.

## Sensor ordinary-run admission

Sensor-models owns its receiver/collection profile and input admission settings.
Flight-models does not introduce a second copy of those settings. Existing
references are `sensor-models/config/runtime/runtime_config.xsd`, the selected
runtime XML, and `tests/cpp/realtime_input_coordinator_test.cpp`.

For each due collection, use the configured target/receiver set and the actual
admitted registry state:

- Physical DIS samples cover target and receiver-host geometry with source times
  no later than the collection, subject to the selected freshness/prediction
  bounds. A target's PRD is not required merely to sense it.
- Reported receiver PRD supplies navigation validity and covariance. Missing PRD
  is not replaced with the receiver's physical truth, even in a shared simulator.
- Required emitter settings/source events must be present under the selected
  existing sensor profile. No emitter-source ownership or carrier transfer is
  performed by this motion slice.

Clock receipt and publication completion do not prove UDP arrival. Eligible
inputs are ordered by source time and the existing deterministic source ranking;
future inputs remain bounded, duplicates/late inputs follow the sensor-owned
policy, and queue overflow fails explicitly rather than silently evicting inputs.
For the initial profile, use the existing `complete_without_output` behavior for
an incomplete target/receiver collection; complete ready sets independently.
There is no new all-platform barrier, unbounded wait, watermark protocol or
ordinary tick acknowledgement. This policy suppresses an incomplete collection;
it does not claim missing truth was delivered. Log readiness/suppression counters
and include them in end-to-end workload/rate acceptance so FTRT is not qualified
by kernel timing alone.

Initial migration runs start fresh with one physical publisher per entity.
Existing pause allows admitted work to finish. Live rewind/restore and new DIS
epoch correlation are outside this slice. Deferred authorization, emitter and
nonideal-navigation contracts still gate their operational implementations;
they do not gate the independent bounded SoA/dynamics implementation.
