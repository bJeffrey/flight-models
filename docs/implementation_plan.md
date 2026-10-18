# SuperCell Replacement Implementation Plan

## Selected Direction (2026-10-10)

Replace SuperCell in the active ecosystem with a focused `flight-models` runtime.
Preserve the current tabular MARL policy and existing operational interfaces.
SuperCell remains a selectable migration/reference backend until replacement
acceptance; do not remove it or start two publishers for the same platform.
This repository is initially empty apart from planning documentation.

The ecosystem master plan owns cross-repository gates. This document owns only
flight-models implementation and local acceptance. Follow the workspace
[engineering and delegation policy](/home/jeffs/git/AGENTS.md).

## Rationale And Evidence Limits

SuperCell's [integration plan](/home/jeffs/git/ams-gra-hello-world-sk-sim-supercell/docs/ftrt_dis_uci_integration_plan.md#acceptance)
records approximately 208 ms wall time per 200 ms scenario tick in 2026-08-29
runs; attempted 8x execution achieved about 0.77x overall. The efficiency task
is still open. This documents a runtime problem, not a completed attribution to
JSBSim computation versus synchronous TCP/control/output work.

The fast 2026-09-24/223621 and 2026-10-04/123130 captures select `Kinematic` for
all five platforms in their generated SuperCell TOMLs. They do not demonstrate
JSBSim performance recovery. That backend instantaneously applies heading and
altitude setpoints, holds configured speed and ignores throttle. Bundled JSBSim
aliases are C172-derived; compatibility with the policy's requested 350/320-knot
conditions remains unaccepted. Replacement is justified by required bounded
motion, model suitability, ownership and measurable scaling, not by an assumed
OpenDIS speedup. Existing code is reusable evidence, not a runtime dependency.

## Responsibilities And Internal Boundaries

| Module | Owns |
| --- | --- |
| Simulated VMS / guidance | Addressed, authorized route activation; waypoint progression; desired heading/speed/altitude; route-version and execution-status tracking. |
| Dynamics core | Deterministic achieved motion with configured vehicle limits and explicit state/time/units/frames. No CAL, DIS, process lifecycle or mission decisions. |
| Emitter-source model | Batched simulated transmitter state and source-event scheduling for scenario entities; source-only truth, separate from propagation, receiver processing and operational observations. |
| Navigation model | Reported navigation solution and covariance, separate from physical truth; explicit error profile and EGI/source provenance. |
| DIS adapter | Map achieved physical state to standard EntityStatePdu, serialize and publish over configured UDP. |
| UCI adapter | Consume detailed RoutePlan and the agreed authorization/control contract; publish PositionReportDetailed and RoutePlanExecutionStatus. |
| Runtime/time adapter | Validated configuration, lifecycle, health, simulation event ordering and output scheduling under ai-bm-sim time authority. |

`route-planning` retains route search, intermediate geometry, feasibility,
arrival constraints and route-change/splice policy. `single-mission-manager`
owns accepted plan/version and per-mission lifecycle. `multi-mission-manager`
owns portfolio admission and scheduling. `execution-authority` owns permission
to activate/continue and revocation. Guidance must enforce the applicable vehicle
limits locally; external permission never bypasses those limits. The exact
authorization carrier is pending the ecosystem contract review, not an invented
UCI field or implicitly approved protobuf.

OpenDIS C++ is the intended DIS codec dependency, subject to the build and
interoperability gate below. It is linked inside the runtime, not a service:

```text
Detailed route + matching execution permission
  -> simulated VMS/guidance -> bounded dynamics -> canonical physical state
  -> DIS adapter -> linked OpenDIS EntityStatePdu serialization -> UDP consumers
                       |
Canonical physical state -> navigation model -> UCI PositionReportDetailed
Simulated VMS state --------------------------> UCI RoutePlanExecutionStatus
```

OpenDIS does not provide dynamics, waypoint guidance, authorization, clocks or
UCI. No JSON/protobuf/network hop exists between the dynamics core and OpenDIS.
Keep codec-specific types inside the DIS adapter. Reuse convention-controlled
frame mathematics from the shared `math` repository. An optional embedded JSBSim
backend may follow later; full aerodynamics is not required for the first slice.

## Batched SoA Architecture (2026-10-10)

User-required implementation direction: use structure-of-arrays (SoA) storage and
batched kernels for all scenario entities' kinematics and emitter-source updates.
Keep stable entity/platform/emitter identities in an explicit index mapping;
array slots are never external identities. Group work by compatible model/profile
and use active masks or compact batches for fixed, flying, disabled and removed
entities. Stable ordering and defined slot reuse must survive spawn/remove/reset.

Separate arrays hold positions, velocities, attitude, desired controls, vehicle
limits and navigation-error state. Emitter arrays hold platform attachment,
transmitter identity, enabled/mode state, source parameters and event phase/time
as required by the accepted source model. Use reusable capacity and scratch
buffers; do not rebuild an object graph, allocate per entity per tick, or perform
per-entity TCP property exchanges in the core hot path. Keep variable-sized route
and emission-event data in indexed storage outside the dense numerical columns.

Vectorization and parallelism are implementation tools, not proof of performance.
Define one configured worker/resource budget, deterministic partitioning and
entity/emitter-keyed random streams so reordering or worker-count changes do not
silently change navigation noise or emitter events. Preserve physical event times
within a step; several events per entity may be due. Do not collapse required
events to one latest value or materialize unbounded pulse/IQ streams when a
declared metadata profile suffices. Bound buffers and specify overflow behavior.

Materialize canonical records and OpenDIS/UCI objects only at adapter boundaries.
Publication workers consume coherent, versioned state snapshots; they must not
read half-updated SoA columns. Record source time and output age independently of
send time. Benchmark numerical kernels, event generation, encoding and delivery
separately, with scalar/reference parity and the same fidelity/workload.

### Emitter Boundary

`flight-models` owns the target batched emitter-source state/schedule attached to
simulated entities. This does not move RF propagation, light-time solving,
receiver antenna/detection physics, noise, PDW/IQ synthesis or UCI observation
generation out of `sensor-models`. Audit existing sensor-owned emitter settings
and algorithms before transfer; reuse them or leave them in place until an
explicit contract migration, rather than implementing two source models.

ai-bm-sim selects scenario platform/emitter identities and service-owned profiles;
it does not introduce a second set of emitter parameters. Define the authoritative
source for each parameter, enable/mode transition and event. Emitter state is
simulation-only truth, never an actor observation or operational target identity.
The carrier for source state/events remains undecided: audit the selected standard
DIS emission/transmitter PDUs before choosing a narrowly scoped separate protocol.
EntityStatePdu alone must not be assumed to carry arbitrary emission semantics.
No new emission carrier or raw-IQ publication is approved by this plan.

## PRD Ownership And Navigation Boundary (2026-10-10)

**Decision: flight-models creates simulated UCI PositionReportDetailed.** No new
navigation repository or PRD bridge is required for this migration. Its navigation
module transforms achieved physical state into the modeled onboard navigation
solution; its UCI adapter maps that solution to schema-valid PRD. OpenDIS is not
involved. Keep this module transport-independent so an independently maintained
navigation model could replace it later without changing consumers.

```text
Physical state SoA -> DIS adapter -> physical EntityStatePdu
                  -> navigation/error-model SoA -> UCI adapter -> PRD
Emitter-source SoA -> agreed truth carrier -> sensor-models physical generation
```

The PRD contract defines source measurement time, navigation solution state,
position/velocity, units/frames/altitude reference, full required covariance blocks,
platform SystemID and navigation/EGI SubsystemID. Freeze these mappings against
the installed UCI schema and existing consumer contracts. Navigation errors and
covariance must reflect the declared model; neither arbitrary placeholder accuracy
nor silently substituted physical truth is acceptable. An explicitly selected
ideal-navigation test profile may use exact state and zero covariance, labeled in
configuration/provenance; it does not establish realistic navigation acceptance.

Generate local navigation for each simulated platform, but publish/share PRDs only
for the configured reporting set. Red physical truth does not become a friendly
operational PRD merely because it exists in the same SoA. Local own-platform
navigation feeds local services; off-platform delivery follows comms-models policy.
Retain platform identity and source timestamp through impairment/reordering;
missing or stale PRDs cannot be replaced with backend physical state. A standalone
PRD relay that bypasses recipient routing is not part of the target topology.

## Configuration And Motion Contract

Schema-validated, versioned profiles must define speed/altitude envelopes,
acceleration/deceleration, turn-rate and climb/descent-rate limits, guidance
behavior, waypoint capture rules, integration/substep policy, output cadences,
navigation errors, endpoints, failure behavior and resource budgets. Units are
explicit; there are no hidden tuning defaults. The initial frame convention and
altitude reference must be agreed with route-planning and existing consumers.

Commands change desired state, not achieved state. A 45-degree heading change
takes time under the configured turn limit; climbing/descending and changing
speed obey their respective limits. Define shortest-turn/wrap behavior,
overshoot handling and zero/invalid-dt rejection. Unsupported timing or route
constraints reject explicitly rather than silently teleporting or claiming
arrival feasibility. Speed, acceleration and climb limits must be mutually
consistent with the selected definition of air/ground speed and wind profile.

Exactly one simulation clock owns advancement. Scenario time drives motion,
validity and publication; monotonic wall time measures scheduling/health. Fixed
scenario-step results must be independent of wall pacing. Preserve source times,
identities and epoch/reset semantics; never stamp stale state with a new time.

## Implementation Checklist

- [ ] **FM-CONTRACT-1 — flight-models:** inventory reusable SuperCell adapters,
  route-planning contracts and shared math. Freeze canonical vehicle state,
  model profile, guidance API and adapter field/source table, including DIS
  identity/time/frame/dead-reckoning semantics and PRD navigation covariance.
  Include SoA index/lifecycle rules, emitter-source ownership and carrier audit,
  navigation profile semantics and configured PRD reporting membership.
  Requires existing boundary docs; deliver schema-valid nominal and rejection
  fixtures. Validate units, correlation and lifecycle cases. Stop before runtime
  implementation; coordinate authorization mapping with ECO-FM-CONTRACT-1.
- [ ] **FM-SOA-1 — flight-models:** after FM-CONTRACT-1, establish typed SoA
  columns, stable identity/index mapping, model batches, coherent snapshots and
  bounded scratch/event storage. Validate active masks, slot reuse, spawn/remove,
  reset and deterministic ordering with scalar/reference fixtures. Stop before
  introducing unvalidated vectorized model behavior or transport dependencies.
- [ ] **FM-DYNAMICS-1 — flight-models:** after FM-SOA-1, implement the transport-independent
  bounded kinematic core and waypoint guidance from FM-CONTRACT-1. Verify straight
  motion, finite-radius turns, heading wrap, acceleration/deceleration, climb and
  descent, limit saturation and waypoint progression. Compare identical elapsed
  scenario time across declared substep policies with explicit tolerances.
  Stop at local deterministic model acceptance, without aerodynamic claims.
- [ ] **FM-EMITTER-1 — flight-models:** after source ownership/carrier review and
  FM-SOA-1, implement batched emitter-source evolution/events using accepted
  profiles. Validate same event times/content under supported pacing, partitioning
  and worker counts, plus enable/disable/reset and buffer limits. Compare with
  retained source-model fixtures; do not duplicate sensor-models propagation or
  receiver processing. Stop before cross-service emission acceptance.
- [ ] **FM-NAV-PRD-1 — flight-models:** after FM-CONTRACT-1 and FM-SOA-1, implement
  batched navigation/error state and the PRD encoder. Validate configured error
  statistics/covariance consistency, reproducible entity-keyed noise, source-time
  and platform/EGI identity mapping, reporting membership and strict UCI schema
  conformance. Validate ideal and nonideal profiles separately. Stop before live
  CAL/recipient-routing acceptance; runtime wiring remains FM-RUNTIME-1.
- [ ] **FM-DIS-1 — flight-models:** pin an OpenDIS revision/license record and
  build it in the maintained container environment, with the selected DIS version
  and examples disabled. Implement the adapter after FM-CONTRACT-1. Validate
  identity, source timestamp encoding/precision/wrap, ECEF position/velocity,
  orientation and dead-reckoning frames, lifecycle and packet lengths using
  independent decoding and existing sensor-models-compatible fixtures. Exercise
  the selected version explicitly: inspected upstream tests link OpenDIS6.
  Stop before claiming live interoperability or performance gains.
- [ ] **FM-RUNTIME-1 — flight-models:** after core, codec, emitter and navigation
  checks, add the clock,
  route ingress and output adapters. Preserve command idempotency, plan version,
  applicability, cancellation, permission expiry/revocation and reset behavior.
  Emit execution feedback from achieved behavior, not publication success.
  Distinguish physical DIS state from modeled navigation PRD. Validate missing,
  stale, duplicate, reordered and unauthorized inputs with focused mocked ports.
  Stop before live ecosystem acceptance.
- [ ] **FM-PERF-1 — flight-models:** provide a bounded benchmark wrapper for
  fixed workloads at 1/4/16/64 platforms with declared emitter counts/event rates,
  separating SoA guidance/dynamics, emitter events, navigation, serialization/
  publication and receiver latency. Record allocations, memory, worker budget,
  profile/revision/hardware,
  warmup, steady-state median/p95/p99, achieved rate, queues/drops and error counts.
  Compare equivalent fidelity backends; never label Kinematic timing as JSBSim.
  Establish required rates/tolerances before acceptance. Validate wrapper smoke;
  leave compute-heavy runs user-run and record capacity as unvalidated until then.

## Migration Completion And Checkpoint

Local tests are necessary but do not retire SuperCell. Ecosystem acceptance must
show the current policy repeatedly producing coarse intents, detailed routes,
committed/authorized execution, achieved motion and subsequent observations, with
correct execution feedback and truth isolation. Test pause/step/reset, supported
pacing, cancellation, revoked authority and partial platform failures. Display
and sensor consumers must preserve the existing meaning of their inputs.

Only one selected backend may publish authoritative physical state for any
platform identity. Run comparisons separately or on isolated networks/identities;
duplicate authoritative publishers are never a migration strategy.

Checkpoint 2026-10-10: this plan and docs index establish the replacement scope,
SoA batching requirement and flight-models-owned navigation/PRD generation;
no implementation or runtime acceptance is claimed. Next executable local slice:
FM-CONTRACT-1, coordinated with ECO-FM-CONTRACT-1. SuperCell remains available;
the master plan owns selection, live comparison and eventual retirement.
