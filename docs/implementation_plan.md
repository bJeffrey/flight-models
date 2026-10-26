# SuperCell Replacement Implementation Plan

## Selected Direction (2026-10-10)

Replace SuperCell in the active ecosystem with a focused `flight-models` runtime.
Preserve the current tabular MARL policy and existing operational interfaces.
SuperCell remains a selectable migration/reference backend until replacement
acceptance; do not remove it or start two publishers for the same platform.
The initial repository contained planning documentation only. The first contract
slice below adds test-only C++ validation; no runtime is implemented.

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

### Numerical backends and population-aware execution (2026-10-23)

User-selected backend candidates are Eigen, Intel MKL and standard C++. Follow
AST's abstraction pattern: stable backend-independent domain/data-view APIs with
one numerical implementation selected by the maintained CMake/container build.
Flight-model developers do not include backend-specific types or select a
library at each call site. SoA storage, identity/slot mapping and physical/model
semantics remain independent of the selected numerical backend. Keep flight code
independent of AST's tracker/plugin implementation; reusable mathematical kernels
belong in shared math when actual consumers justify them.

AST references inspected for this decision:

- `ast/include/plugins/propagator/la.h` and `la_batch_rw.h`: stable matrix/batch
  and sparse-subset APIs; implementations live under
  `ast/src/plugins/propagator/la/`. CMake and Docker's `BACKEND` build argument
  choose `std`, `eigen` or `mkl`; selection is not per-call runtime dispatch.
- `ast/src/plugins/filter/passive_pf/family_pf_backend.cpp`: serial scoring for
  the promoted 1,200-particle workload because measured OpenMP dispatch cost
  outweighed its benefit. Sparse covariance subset implementations likewise
  retain serial paths for small candidate sets.
- `ast/include/plugins/filter/passive_pf/tempered_smc.h` and corresponding source:
  configured batched rejuvenation/likelihood blocks. The migration-map benchmark
  records scalar/batch comparisons at 4,096 particles; these are batching
  evidence, not a measured flight-model crossover.

The inspected AST paths do not implement an automatic update-population
threshold. Flight-models must explicitly support different execution paths for
small and large workloads: direct indexed/scalar updates on the SoA for small
sets, and compatible contiguous batches for sufficiently large sets. Backend and
scalar/batch path are separate choices; standard C++ may implement batch kernels,
and a selected Eigen/MKL backend must retain an efficient small-work path.

Count actual active/due ownship and target updates per compatible kernel/model
cohort, not total scenario entities or allocated capacity. Preserve stable
identity/order when compacting or gathering subsets, and reuse packing/scratch
storage. Include gathering, scatter and dispatch costs when comparing paths.
Both paths implement the same model and event schedule; identities/order remain
exact, while numerical results meet declared tolerances.

Calibrate any crossover using flight-model workloads on the selected backend and
container CPU/thread budget. Put the selected threshold/resource policy in
version-controlled execution configuration, separate from vehicle motion limits;
record its benchmark provenance. No threshold is selected by this documentation
and no AST particle count/block width is imported as one. Qualify small-count
latency as well as large-count throughput and bound backend/internal threading
within the same process resource budget.

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

### Shared buses and flight timing (2026-10-21)

`ai-bm-sim` remains the sole scenario-clock authority. Flight-models consumes the
authoritative Protobuf/NATS `SimTimeTick` run/epoch/tick contract and the existing
scenario interval; it does not create per-entity clocks or integrate motion from
wall time. Elapsed scenario time starts at zero: a turn at 30 s remains an
elapsed-time command. Map to the scenario UTC origin only for standard absolute
DIS/UCI timestamp fields. A restored epoch retains its restored elapsed time and
action schedule. Runtime integration belongs to FM-CONTRACT-1/FM-RUNTIME-1 and follows
the [shared bus and timing decision](/home/jeffs/git/ai-bm-sim/docs/architecture/ecosystem_message_architecture.md#shared-buses-and-flight-timing-2026-10-21).
Preserve the event-driven clock: global tick acknowledgements are not a target
requirement, and `SimStepRequest`/`SimStepCompletion` remain limited to the
explicit regression harness.

## Implementation Checklist

- [ ] **FM-CONTRACT-1 — flight-models:** inventory reusable SuperCell adapters,
  route-planning contracts and shared math. Freeze canonical vehicle state,
  model profile, guidance API and adapter field/source table, including DIS
  identity/time/frame/dead-reckoning semantics and PRD navigation covariance.
  Include SoA index/lifecycle rules, emitter-source ownership and carrier audit,
  navigation profile semantics and configured PRD reporting membership. Freeze
  the `SimTimeTick` run/epoch/tick and scenario-interval mapping here; inspect
  shared `math` primitives before selecting matrix/vector optimization libraries.
  Coordinate ordinary-run source-completeness/readiness with sensor-models and
  SIM-TIME-FM-1; a tick or separate marker cannot prove DIS/UDP arrival. Preserve
  existing run/epoch/tick fields; new cross-service epoch/restore handling is
  deferred until after SuperCell deprecation and does not block this gate.
  Requires existing boundary docs; deliver schema-valid nominal and rejection
  fixtures. Validate units, correlation and lifecycle cases. Stop before runtime
  implementation; coordinate authorization mapping with ECO-FM-CONTRACT-1.
- [x] **FM-SOA-1 — flight-models:** after the bounded state/profile portion of
  FM-CONTRACT-1 accepted below, establish typed SoA
  columns, stable identity/index mapping, backend-independent data views,
  small indexed-update and large batch access, coherent snapshots and
  bounded scratch/event storage. Validate active masks, slot reuse, spawn/remove,
  reset and deterministic ordering with scalar/reference fixtures. Stop before
  introducing unvalidated vectorized model behavior or transport dependencies.
- [x] **FM-DYNAMICS-1 — flight-models:** after FM-SOA-1, implement the transport-independent
  bounded kinematic core and waypoint guidance from FM-CONTRACT-1. Verify straight
  motion, finite-radius turns, heading wrap, acceleration/deceleration, climb and
  descent, limit saturation and waypoint progression. Compare identical elapsed
  scenario time across declared substep policies with explicit tolerances.
  Stop at local deterministic model acceptance, without aerodynamic claims.
  Initial scalar motion and the bounded waypoint/event driver pass local model
  acceptance below. Live program admission, manual-versus-route arbitration and
  runtime throughput remain their operational integration gates.
- [ ] **FM-EMITTER-1 — flight-models:** after source ownership/carrier review and
  FM-SOA-1, implement batched emitter-source evolution/events using accepted
  profiles. Validate same event times/content under supported pacing, partitioning
  and worker counts, plus enable/disable/reset and buffer limits. Compare with
  retained source-model fixtures; do not duplicate sensor-models propagation or
  receiver processing. Stop before cross-service emission acceptance.
- [x] **FM-NAV-PRD-1 — flight-models:** after FM-CONTRACT-1 and FM-SOA-1, implement
  batched navigation/error state and the PRD encoder. Validate configured error
  statistics/covariance consistency, reproducible entity-keyed noise, source-time
  and platform/EGI identity mapping, reporting membership and strict UCI schema
  conformance. Validate ideal and nonideal profiles separately. Stop before live
  CAL/recipient-routing acceptance; runtime wiring remains FM-RUNTIME-1.
  The ideal and configured synthetic ECEF bias/Gaussian models now pass local
  statistics, deterministic sampling, batch, runtime and live UCI admission
  checks; see the 2026-10-25 error-model checkpoint below. Equipment-specific
  accuracy qualification remains open; synthetic settings and the retained
  covariance fixture are not an equipment accuracy specification.
- [x] **FM-DIS-1 — flight-models:** pin an OpenDIS revision/license record and
  build it in the maintained container environment, with the selected DIS version
  and examples disabled. Implement the adapter after FM-CONTRACT-1. Validate
  identity, source timestamp encoding/precision/wrap, ECEF position/velocity,
  orientation and dead-reckoning frames, lifecycle and packet lengths using
  independent decoding and existing sensor-models-compatible fixtures. Exercise
  the selected version explicitly: inspected upstream tests link OpenDIS6.
  Stop before claiming live interoperability or performance gains. Local
  OpenDIS7 mapping and existing sensor decoder acceptance are recorded below;
  fixed/moving/disabled publication rules are covered. Live spawn/removal and
  transport publication remain FM-RUNTIME-1.
- [ ] **FM-RUNTIME-1 — flight-models:** after core, DIS adapter, emitter and navigation
  checks, add the clock,
  route ingress and output adapters. Preserve command idempotency, plan version,
  applicability, cancellation, permission expiry/revocation and reset behavior.
  Emit execution feedback from achieved behavior, not publication success.
  Distinguish physical DIS state from modeled navigation PRD. Validate missing,
  stale, duplicate, reordered and unauthorized inputs with focused mocked ports.
  Stop before live ecosystem acceptance.
  The fresh-run clock/output process below now passes isolated NATS/two-ASB/UDP
  acceptance. Normal startup, operational controls, emitter carriers and full
  sensor/comms/scenario integration still keep this checkbox open.
- [ ] **FM-PERF-1 — flight-models:** provide a bounded benchmark wrapper for
  configurable workloads: 1/4/16/64-platform smoke cases followed by declared
  representative large-scenario counts, with emitter counts/event rates,
  separating SoA guidance/dynamics, emitter events, navigation, serialization/
  publication and receiver latency. Record allocations, memory, worker budget,
  profile/revision/hardware,
  warmup, steady-state median/p95/p99, achieved rate, queues/drops and error counts.
  Compare scalar/reference and matrix/vector kernels under declared tolerances
  and thread budgets; inspect shared math and AST batching practice before adding
  another numerical dependency. Kernel speed alone is not end-to-end FTRT.
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

Checkpoint 2026-10-21: flight-models owns scalable physical kinematics/truth,
including batched SoA state and benchmark-led use of existing matrix/math
primitives, plus the linked OpenDIS EntityState UDP codec, modeled navigation
error and UCI PRD published to the owning platform's ASB for OWP consumption.
Comms-models owns
offboard dissemination; flight-models does not add flight communications logic.
The PRD is reported navigation, not a renamed physical truth state. The shared
bus and timing decision above is authoritative; the wire contract remains part
of FM-CONTRACT-1. See the [planning index](README.md).


### Initial executable contract checkpoint (2026-10-23)

FM-CONTRACT-1 is in progress. The [state/time contract](state_time_contract.md)
fixes local ECEF/SI physical state, row-major body-FRD orientation, elapsed-time
scheduling and existing SimTimeTick interval/lifecycle interpretation. Its schemas
and nominal specimens describe acceptance inputs, not a new runtime protocol.
The C++17 checker covers the known equatorial level-east basis, invalid rotations
and identities, 30-second scheduling, pause/resume, duplicate delivery, restore
boundary and stale-epoch/interval/overflow rejection. This is test-only reference
logic: replace its clock checker with production adapter calls at FM-RUNTIME-1
while retaining the same specimen inputs. It does not prove runtime quiescence
or restored action-state behavior.

The [adapter audit](adapter_contract.md) records reusable SuperCell DIS/UCI,
route-planning and sensor consumer evidence. Implemented shared geodetic/frame
math is available; several similarly named legacy math headers are empty and
must not be mistaken for implementations. Matrix backend selection remains tied
to real batched-kernel validation. Python was not added to this repository.

Sensor-models consumes DIS physical truth for targets and sensor hosts plus PRD
reported navigation/covariance for the configured receivers. Local PRDs use the
owning platform ASB; selected offboard reports use comms-models. Physical truth
must not replace missing navigation. The shared sensor simulation process may
serve several platform ASBs without granting operational services global PRD
access.

Changed paths: `contracts/`, `tests/fixtures/`, `tests/cpp/`, isolated
`tests/sleet/`, Docker/CMake/Compose and build/test wrappers, root README and the
two contract references. No dynamics, OpenDIS adapter, transport producer or
normal scenario topology was added.

Remaining contract work is concrete: route-planning's PRD source-identity/yaw
expectations differ from the existing SuperCell producer; receiving a tick does
not prove DIS source completeness. Resolve ordinary-run readiness and PRD
compatibility with correlated consumer fixtures and SIM-TIME-FM-1. DIS old-epoch
packet handling is deferred until after SuperCell deprecation. Detailed-route guidance/profile,
navigation-error model, authorization mapping and emitter-source/carrier audit
also remain open; this checkpoint does not close FM-CONTRACT-1 or ECO-FM-CONTRACT-1.

Exact next bounded slice: reconcile the PRD consumer contract with a platform
header SystemID, EGI SubsystemID and reported orientation, preserving full NED
covariance/source-time handling. Add consumer regression cases for the existing
and target specimens, then freeze the remaining flight-model profile and causal
truth-input contract before SoA/runtime implementation. Cross-service epoch
handling is not a prerequisite for that work.


Validation: `./bootstrap.sh` builds the maintained toolchain; `./test.sh` compiles
C++17 with warnings as errors and passes the local contract cases
([log](../logs/20261023/contracts/contract-tests-final.log)).
`./test_sleet.sh logs/20261023/uci-schema-contract-final` passes real UCI 2.5
schema admission for nominal SystemID and EGI-shaped SubsystemID PRDs, and
rejects the missing mandatory header timestamp
([probe log](../logs/20261023/uci-schema-contract-final/schema.log),
[ASB log](../logs/20261023/uci-schema-contract-final/services.log)).
This proves schema admission, not consumer navigation interoperability or a
realistic navigation model. The isolated ASB/network is removed after the test.
Focused independent review findings on basis direction, restore pause boundaries
and integer bounds were resolved. No OpenDIS build/interoperability or throughput
claim is made.


### SuperCell deprecation sequencing correction (2026-10-23)

User decision: finish replacing/deprecating SuperCell before implementing new
cross-service epoch handling. Preserve existing run/epoch/tick fields and existing
validated behavior; do not implement DIS epoch correlation or live rewind/restore
as a migration prerequisite. Use fresh isolated runs for replacement acceptance.
The first executable contract's restore cases remain bounded reference fixtures.

The immediate next slice remains PRD consumer compatibility (platform SystemID,
EGI SubsystemID, reported orientation, covariance and source time). Then finish
the bounded model/guidance profile and ordinary-run input contract, implement
FM-SOA-1 and FM-DYNAMICS-1, and add FM-DIS-1/OpenDIS plus FM-NAV-PRD-1. Proceed
through runtime/scenario integration and measured acceptance to SuperCell
retirement. Complete the deferred epoch/restore work afterward. The ecosystem
master remains the sole cross-repository checklist.


### PRD consumer compatibility checkpoint (2026-10-23)

Route-planning's existing UCI adapter now uses header SystemID as the platform
key and accepts either SystemID or SubsystemID for the navigation contributor.
Its adapter/dispatcher cases cover a distinct EGI, later publication time,
component validity, reported yaw distinct from ground course, and complete
covariance including cross terms. The standard-shaped SubsystemID fixture matches
the prior real-Sleet schema-accepted specimen. See the
[owning implementation/evidence](/home/jeffs/git/route-planning/docs/development_plan.md#flight-models-prd-consumer-compatibility-2026-10-23).

The local consumer prerequisite is complete. SuperCell still omits yaw; the
replacement flight-models PRD producer must include modeled navigation
orientation. This work adds no truth/ground-course substitute, new source-ID
assurance gate or flight runtime, and does not retire SuperCell.

Exact next bounded slice: finish the initial bounded motion/guidance profile and
ordinary-run input contract in FM-CONTRACT-1: speed/acceleration/turn/climb limits,
route/frame/altitude and event/substep conventions, separate DIS/PRD cadences,
reporting membership and local sensor input readiness. Use explicitly synthetic
or ideal test profiles where appropriate rather than inventing aircraft or
navigation performance claims. Then implement FM-SOA-1 and bounded dynamics;
OpenDIS and modeled PRD adapters follow as recorded above. New epoch/restore
handling remains deferred until after SuperCell deprecation.


### Bounded model and ordinary-run input checkpoint (2026-10-23)

The local state/profile contract required by FM-SOA-1 is accepted. Added the
[initial motion/input contract](motion_input_contract.md), version-controlled
`config/profiles/synthetic_fixed_wing.json` and strict
`config/motion_profile.schema.json`, with typed C++ loading through
`include/flight_models/motion_profile.h` and `src/motion_profile.cpp`.
The configuration library is transport-independent and builds without a process
runtime. There are no hidden tuning defaults or alternative development-phase
configurations.

The initial model is explicitly synthetic, no-wind bounded kinematics. Its
execution envelope matches the existing generic route-planning test envelope;
copying those values does not qualify an aircraft or implement the eventual
planner-subset distribution. Ground-horizontal speed and independent HAE rates,
shortest-turn/clockwise tie semantics, swept waypoint capture, scheduled-event
ordering, explicit midpoint integration and bounded work are now specified.
DIS and PRD schedules remain independent of each other and clock/substep spacing.
Navigation is explicitly `ideal_test`; nonideal modeling remains FM-NAV-PRD-1.

Ordinary-run sensor admission uses actual admitted target/receiver geometry and
receiver PRD under sensor-owned freshness/queue settings. The initial selected
missing-input behavior is the existing `complete_without_output`: suppress the
incomplete collection and count it, while ready sets proceed. No clock receipt
or publication marker is treated as delivery proof; no new watermark protocol,
all-platform barrier or unbounded wait is introduced. End-to-end readiness and
suppression counters remain part of runtime/performance acceptance. Sensor
runtime clock-adapter integration is not implemented by this contract slice.

Validation uses the maintained `./test.sh` Docker workflow: both the new motion
profile loader test and existing state/time contract test pass, with C++17
warnings treated as errors
([log](../logs/20261023/motion-profile-tests-final.log)). Cases include unknown
model/navigation/fields, missing settings, zero/negative values, ordered bounds,
independent cadences, exact/ceil work budgets, exact integer-token requirements,
INT64_MAX retention and unsigned overflow rejection. Independent focused review
found no material defect. No dynamics, OpenDIS codec or throughput result is
claimed; UCI schema fixtures were unchanged and their prior validation remains
applicable.

Exact next development slice is **FM-SOA-1**: contiguous typed numerical columns,
stable platform identity/slot generation mapping, model batches, coherent snapshot
extraction and bounded scratch storage. Validate spawn/remove/slot reuse and
fresh-run reset deterministically before implementing motion kernels. Detailed
route constraints, operational authorization and emitter-source transfer still
gate their respective adapters/live replacement, not transport-free storage.
The complete FM-CONTRACT-1/ECO-FM-CONTRACT-1 gates remain open for those integration
contracts. New epoch/restore work remains deferred until after SuperCell
deprecation.


### AST backend/batching reference checkpoint (2026-10-23)

The SoA slice now explicitly preserves the user-required numerical abstraction
and small/large update distinction; see the backend/population subsection above.
This is a design refinement for FM-SOA-1 and subsequent kernels, not implemented
flight-model backend support or benchmark acceptance. AST's code, rather than its
stale `docs/la_overview.txt` build/timing summary, supplies the concrete reference.
Next remains FM-SOA-1 storage/views/lifecycle, followed by the motion kernels and
measured backend/path qualification. Epoch work remains deferred.


### Bounded SoA storage checkpoint (2026-10-23)

FM-SOA-1's initial physical-state storage is implemented in
`include/flight_models/state_store.h` and `src/state_store.cpp`; see the
[storage contract](state_storage.md). It has separate numerical component columns,
packed row-major attitude, desired controls/source times, active/mode/profile
metadata and generation handles. Shared types come from `math::math` through the
maintained container mount; the developer API exposes no numerical-backend types.
Read-only indexed live views and reusable packed StateBatch scratch support small
explicit sets and full compatible cohorts without imposing a crossover threshold.

Staged updates validate before changing live values. Revisions invalidate stale
prepared work, failed gathers invalidate their destination, and snapshots own
canonical ID-ordered rows with the original source times. Slot reuse and fresh
clear cannot alias old handles; a preallocated free-slot heap avoids quadratic
startup slot scans. Mode filtering distinguishes occupied fixed/moving/disabled
entities. Calls are single-owner serialized; publishers consume independent
snapshots rather than borrowing changing columns.

`config/storage_policy{,.schema}.json` records explicit example capacities and
profile registry order. Tests validate that policy, while a future runtime adapter
will perform actual deployment loading. There are no hidden constructor bounds.
Profile limits remain outside the hot per-entity columns. Emitter-event and
nonideal-navigation storage is added with its owning module when those consumers
exist; no speculative source-event schema was introduced here.

Changed paths: state-store public API/source, storage policy/schema, state-store
behavior tests plus test-only allocation probe, CMake/shared-math Compose mount,
README/index and storage reference. No dynamics kernel, Eigen/MKL backend,
scalar/batch threshold, OpenDIS adapter or runtime was added.

Validation: maintained `./test.sh` builds C++17 with warnings as errors and passes
all three test executables ([log](../logs/20261023/soa/tests.log)). Storage cases
exercise capacity/profile bounds, duplicate identity, stale generations through
remove/reuse/clear, lowest-free-slot order, profile/mode metadata, all-or-nothing
batch rejection, decreasing source-time rejection, stale/foreign batches,
retained snapshots, untouched source times and 256-row scalar/batch data
agreement. A test-only allocator rejects heap allocation during warmed gather,
commit and snapshot cycles with long identity strings. Lifecycle/initial-string
allocations are outside that assertion; this is not a throughput claim.
Independent review found no material correctness defect; its cold-population
scan concern was removed with the reserved free-slot heap.

Exact next slice: **FM-DYNAMICS-1**, implementing and testing bounded no-wind
motion/waypoint kernels against this state/profile contract. Keep the same API
for standard C++, Eigen and MKL implementation choices; qualify backend and
population crossover with actual kernels and packing costs before configuring
production dispatch. New epoch/restore work remains deferred until after
SuperCell deprecation.

### Initial bounded motion kernel checkpoint (2026-10-23)

`include/flight_models/bounded_motion.h` and `src/bounded_motion.cpp` add
`advance_motion`: a transport-independent scalar implementation over reusable
SoA cohort scratch. Moving entities for one profile advance from their own source
times to a common scenario-nanosecond endpoint. Fixed, disabled and other-profile
entities retain their state and source times. One commit publishes the cohort
only after every row succeeds; invalid inputs, insufficient scratch and exhausted
integration budgets leave live state unchanged.

The kernel uses shared WGS-84 geodesy and angle wrapping, bounded midpoint
integration with control-capture splits, shortest-path heading changes with a
clockwise half-turn tie, horizontal acceleration/deceleration and independent
HAE climb/descent. Body attitude follows heading and climb angle with zero roll.
Endpoint acceleration and angular velocity include local-frame transport rather
than assuming constant-speed motion has zero ECEF acceleration. Source timestamps
and regular step boundaries remain integer nanoseconds; control-capture splits
use relative double seconds within each step. This is synthetic point kinematics,
without an aerodynamic or vertical-acceleration model.

This checkpoint does **not** complete FM-DYNAMICS-1. Waypoint capture/progression,
scheduled desired-command application, independent publication deadlines and
their bounded event storage are not implemented. In particular, this kernel
holds the admitted desired controls constant throughout each call; a future
event driver must split calls at actual command/output boundaries. Do not wire
it as a replacement runtime or claim DIS/PRD acceptance yet. Eigen/MKL execution,
indexed sparse dispatch and measured population thresholds also remain pending.

Changed paths: bounded-motion public header/source, analytical/convergence and
failure tests, CMake target, top-level test description, README/index and this
owning plan. The ecosystem master lane links to this checkpoint.

Validation: maintained `./test.sh` builds C++17 with warnings as errors and passes
all four test executables ([log](../logs/20261023/dynamics/tests.log)). New cases
cover analytical equatorial travel, local constant-rate turns (1 cm tangent-plane
tolerance over 200 m), clockwise half-turns and heading wrap, speed/HAE target
capture, asymmetric climb/descent bounds, excluded cohorts, coherent initial
attitude, zero-duration preservation and atomic rejection. The combined
turn/acceleration/climb fixture agrees within 1 mm in ECEF at 10 s between 10 ms
and 1 ms substeps; aligned 1 s and 250 ms input partitions agree within 10 μm.
These are local numerical-fixture tolerances, not sensor/range-difference accuracy
requirements. Exact 200-step work budgets and off-grid control captures also pass.
No throughput or backend-crossover claim is made. Focused independent review
identified the reproduced vertical-rate reprojection roundoff issue; it was
fixed and subsequent accounting/derivative review found no blockers.

The shared-math extraction, flight-models adoption and scalar optimization
requested after this checkpoint are now complete; see the acceptance checkpoint
below. FM-DYNAMICS-1's waypoint guidance/scheduled-event driver remains open.
Guidance still requires swept 3D capture, ordered transitions, exact scenario-time
command/output boundaries and atomic failure on event-budget exhaustion.
Backend/population qualification and the owning OpenDIS/PRD adapter gates remain
pending. New epoch/restore work stays deferred until after SuperCell deprecation.

### Coordinate conversion and native message frames (2026-10-23)

Accepted strategy: preserve canonical ECEF physical truth and track/platform
state, while retaining native measurement observables. Geodetic positions and
local NED/ENU bases are derived geometry, not competing authoritative state.
The [state/time contract](state_time_contract.md#coordinate-conversion-boundaries)
records message-boundary, precision and cache-validity rules. Existing mappings
are recorded in the [adapter inventory](adapter_contract.md),
[sensor interface](/home/jeffs/git/sensor-models/docs/reference/uci_sensor_interface.md)
and [AST UCI plan](/home/jeffs/git/ast/docs/ast_uci_interface_plan.md).

Static inspection of the initial scalar kernel finds two ECEF-to-geodetic
conversions, one geodetic-to-ECEF conversion, and three NED-to-ECEF matrix
constructions/vector rotations per integration segment. Intermediate endpoint
velocities are calculated even when only the final value is consumed. Repeated
initial/final frame construction also exists outside the loop. These are code
operation counts, not measured bottlenecks or accepted throughput evidence.

The subsequent ownership audit found WGS-84 eccentricity/prime-vertical-radius
formulas duplicated between `src/bounded_motion.cpp` and
`math/src/coordinates/geodetic.cpp`. Flight-models also implements body-FRD/NED
attitude construction, matrix composition/validation and local-frame transport
without shared APIs. Math already supplies geodetic/ECEF conversions, local-frame
rotations, vector rotation/transpose and angle wrapping; keep using those.
Its legacy rotation/quaternion/vector/ellipsoid headers are empty placeholders,
not implementations to call. Small scalar expressions alone do not justify a
new abstraction; extraction should address reusable, convention-controlled work.

- [x] **Math prerequisite:** add narrowly scoped, backend-independent primitives
  under `math/include/math/` for the required attitude construction/composition,
  proper-rotation validation, WGS-84 curvature geometry and local-frame transport.
  Reuse the curvature calculation inside math's existing geodesy rather than
  introducing another copy. Document row-major ordering, FRD/NED/ECEF conventions,
  radians/metres, singularities, explicit validation tolerances and error behavior.
  Keep aircraft limits and control policy out of math. Validate with independent
  known-axis/rotation cases, reflection/non-orthogonal rejection, ellipsoid
  reference cases and transport/derivative checks under math's maintained tests.
- [x] **Flight-models adoption:** after the shared primitives pass, replace local
  reusable calculations and use existing shared constants. Keep speed/altitude
  target evolution, control captures, work budgets, integration scheduling and
  model-specific derivative assembly in flight-models. Preserve a pre-change
  baseline artifact; run analytical/convergence and frame/derivative regressions
  plus focused independent review. This adoption is a separate bounded slice
  following the math prerequisite, not a second implementation of its formulas.
- [x] After shared-math adoption, optimize the scalar kernel before waypoint/event
  expansion: construct endpoint velocity only when needed, reuse geometry at the
  same state/time, and evaluate replacing the altitude-correction forward
  conversion with a shared, tested ellipsoid-normal correction. Preserve necessary
  midpoint geometry, HAE control, proper attitude and frame-transport derivatives;
  do not remove conversions merely to reduce a count.
- [ ] Audit the DIS truth → sensor geometry → OMR → AST path and the separate
  navigation → PRD → consumer path against actual native fields. Identify each
  required position, vector, attitude and covariance transformation and repeated
  derivation. Record findings here; implementation in sensor-models/AST belongs
  to their existing owning plans as separate slices. Comms forwards message
  content without coordinate transformations.
- [x] Validate the optimized scalar implementation against the existing
  analytical/convergence fixtures and declared numerical tolerances. Cover
  longitude wrap, supported latitude extremes, altitude bounds, climb/descent
  and turn derivatives. Benchmark small and large populations with identical
  workloads, warmup and repeated runs; record conversion counts and steady-state
  timing in dated logs. Use the preserved baseline for the comparison. Qualify
  backend/batch dispatch only after this scalar baseline.

Completion means tested shared-math ownership, flight-models adoption, a reviewed
conversion inventory, validated scalar changes and reproducible before/after
evidence. No end-to-end conversion minimization or
truth-accuracy improvement is claimed by accepting this strategy alone. DIS
wire quantization and nonideal PRD/OMR accuracy remain their adapter/model gates.

### Shared-math adoption and scalar conversion checkpoint (2026-10-23)

Shared primitives now live in `math/include/math/coordinates/{geometry,geodetic}.hpp`
and their source files: FRD/NED attitude construction, rotation composition and
validation, cross products, WGS-84 curvature radii, local-frame geometry,
frame-transport rates and ellipsoid-normal altitude replacement. Existing
geodetic conversions reuse the curvature implementation. Aircraft/control limits
and model-specific derivative assembly remain in flight-models. A frame is
derived for one source state; altitude-only correction updates its origin HAE
without changing its latitude/longitude basis. Geographic-pole transport retains
the finite selected-meridian case for exactly zero east velocity and explicitly
rejects singular eastward motion.

Flight-models' scalar kernel now reuses that geometry for heading extraction,
coherence validation, prediction and endpoint output. It retains the two
ECEF-to-geodetic evaluations required by the current midpoint/altitude scheme;
it replaces each segment's forward conversion with a shared normal correction
and constructs endpoint velocity once per advancing row. Attitude, velocity and
derivatives use the same final frame. Per segment the static operation counts
fall from one to zero geodetic-to-ECEF conversions, and from three to two local
rotation constructions and vector rotations; fixed per-row work is separate.
Canonical stored position/velocity/acceleration remain float64 ECEF.

Validation/artifacts: the maintained Docker image builds both repositories with
warnings as errors. The standalone math suite and flight-models suite each have
four passing test executables. Math tests cover independent axis/sign/composition
cases, invalid rotations, curvature, frame transport including polar cases and
normal correction. Flight regressions retain the analytical and substep fixtures
and add longitude wrap at ±85°/equator and altitude bounds, plus endpoint
velocity/acceleration versus neighboring trajectory samples. The adoption-only
kernel and test log were preserved before optimization. See dated
[math tests](../logs/20261023/coordinates/math-tests.log),
[adoption tests](../logs/20261023/coordinates/adoption-tests.log) and
[optimized tests](../logs/20261023/coordinates/optimized-tests.log).
Independent review checked mathematical conventions and the cached-frame
integration sequence; validation and polar-compatibility findings were resolved.

The original working-tree kernel and shared math source are preserved under
`logs/20261023/coordinates/baseline/`, with source hashes and original git IDs.
The benchmark's versioned workload/schema and validated synthetic profile are
identical for baseline and candidate. Release builds run in the cached offline
Docker image, with one warmup and five measured repeats per population. Both
execution orders were run, producing ten timing samples per implementation.
Commands apply at the start of their due intervals, with continuous absolute
simulation phase across warmup/repeats; the harness requires an interval-aligned
command period. Timings include command gather/commit and motion
gather/integration/commit, excluding initialization and CSV output.

| Moving entities | Intervals per repeat | Baseline median updates/s | Optimized median updates/s | Median change |
| --- | --- | --- | --- | --- |
| 1 | 10000 | 89630 | 95752 | +6.8% |
| 32 | 1000 | 102880 | 109747 | +6.7% |
| 1000 | 30 | 104067 | 110946 | +6.6% |
| 10000 | 12 | 102845 | 109769 | +6.7% |

These are per-entity interval updates, each normally containing ten 10 ms
substeps, with additional control captures as needed. The one-entity timing
ranges overlap; medians describe this machine/workload, not a guaranteed speedup.
Each population has a different repeat duration, so cross-population values do
not select a dispatch crossover. Full ranges/raw samples, compiler/CPU/image
identity, source hashes and exact commands are in the
[comparison](../logs/20261023/coordinates/comparison-final.txt) and
[metadata](../logs/20261023/coordinates/benchmark-metadata.txt).

All 11033 final rows match in identity, time and desired commands; repeated runs
of each implementation produce byte-identical final rows. Maximum differences
from the original are 0.441 mm ECEF position, 1.11e-9 m/s velocity, 3.45e-11 m/s²
acceleration, 1.03e-11 DCM Frobenius norm and 1.87e-15 rad/s body angular rate.
These compare implementations, not physical truth against external ground truth,
and introduce no new sensor/range-difference accuracy requirement. Full rows and
source manifests are retained beside the results. Earlier unqualified build and
command-timing runs are explicitly superseded by these final artifacts.

Changed paths: shared math public headers/source and coordinate tests/CMake/docs;
flight-models kernel, geometry regressions, optional benchmark target,
`benchmarks/{motion_benchmark.cpp,workload.json,workload.schema.json}`,
`benchmark.sh` and owning references. Reproduce the current configured benchmark
with `./benchmark.sh`; the metadata records frozen-baseline comparison commands.
The requested shared-math/adoption/optimization slice is complete. This measures
the scalar path, not Eigen/MKL, parallel execution or full scenario throughput.

The subsequent waypoint/event slice is now accepted in the checkpoint below.
The cross-service DIS/PRD/OMR conversion inventory remains a separate
pending task before adapter acceptance; numerical backend/population dispatch
also remains unqualified. New epoch/restore handling stays deferred until after
SuperCell deprecation.

### Bounded waypoint and event-driver acceptance (2026-10-23)

FM-DYNAMICS-1's initial transport-free model is accepted locally.
`MotionDriver` consumes typed entity programs supplied directly by deterministic
tests: ordered ECEF waypoints or ordered timed desired-motion commands, plus
navigation-reporting membership. It introduces no scenario format or operational
command protocol. See the [input contract](motion_input_contract.md#initial-waypointevent-driver-slice).
WGS-84/explicit-origin ENU admission uses shared math; a new shared Cartesian
segment/sphere primitive owns swept contact geometry. Guidance uses the existing
shared line-of-sight bearing and caches immutable waypoint HAE once at admission.
ECEF remains the stored physical frame.

The driver stages an entire interval in reusable SoA batches. Commands, route
cursors, initial events and independent physical/navigation sample deadlines
become visible only after one successful store commit. Construction has no
observable motion/event effects; the first advance, including `advance_to(0)`,
validates and processes initial state transactionally. Same-time commands use
their supplied sequence order; overlapping waypoints drain in route order before
coincident samples. Capture uses a numerical swept chord with upward rounding
of at most one nanosecond, not exact continuous aerodynamic intersection.
Post-command samples carry refreshed control derivatives, retaining the existing
explicit synthetic vertical-rate discontinuities.

Admission and output/event bounds are explicit `DriverLimits`, with a
schema-validated small fixture policy in `config/event_policy{,.schema}.json`.
Deployment sizing/loading is not implemented here. Cumulative per-entity
numerical budgets include all event segments and probes/reintegration. Failed
advances retain committed store values, time, cursors and prior outputs. Fixed
entities retain pose while advancing sample validity; disabled and other-profile
entities are excluded. A membership revision detects structural cohort changes;
this driver supports a fresh, fixed cohort/program set, not live spawn or route
replacement. Mixed route/manual-command programs fail explicitly until VMS
arbitration is specified. Typed navigation-cadence samples are inputs to the later
navigation model/encoder, not accepted operational PRDs.

Changed paths: driver public header/source; staged kernel and StateBatch helpers;
store membership revision; driver fixture policy/schema; driver/math geometry
tests and CMake; shared swept-contact header/source; README/index and references.
Public APIs document units, ownership, sample lifetimes and failure boundaries.
Program lookup is linear at admission rather than a quadratic entity search.

Validation: maintained Docker Release builds with warnings as errors pass all
five flight-models and five shared-math test executables. Evidence:
[flight tests](../logs/20261023/events/flight-tests.log) and
[math tests](../logs/20261023/events/math-tests.log). Cases include 70/110 ms
independent cadences, exact 30 s command application, same-time command order,
post-event derivatives, 0.25 m waypoint capture with 20 ms steps at 100 m/s,
coincident captures, guided turns, failed initial/interval transactions and retry,
output/transition/aggregate-step limits, invalid initial moving states and cohort
lifecycle changes. Nonaligned input partitions preserve publication times and
agree within 1 mm in the tested trajectory. Guided 10 ms and 1 ms substep runs
agree within the same fixture tolerance at 2 s. A warmed allocation guard covers
event processing and the unchanged scalar entry point. Focused review findings
on initial validation, cascade ordering and lifecycle boundaries are resolved.

The existing scalar benchmark was rerun once with its unchanged workload and
Release configuration: medians were 95407, 107967, 107056 and 107256 entity
updates/s for populations 1, 32, 1000 and 10000
([raw samples](../logs/20261023/events/scalar-benchmark.txt)). These are within
about 0.4–3.5% of the prior optimized medians; one check does not establish a
statistical regression or qualify the new event driver. No backend crossover,
event-driver workload/FTRT or scenario acceptance is claimed.

Next at this checkpoint (completed in the subsequent adapter checkpoint):
**FM-DIS-1**, pin/build OpenDIS in the
maintained container and implement/independently decode EntityState packets from
typed ECEF snapshots, checking identity, time precision/wrap, orientation and
dead-reckoning conventions. Reuse the existing adapter contract and complete its
relevant conversion inventory. Navigation/error/PRD and emitter ownership gates
then feed runtime/scenario integration before SuperCell retirement. New epoch
and restore handling remain deferred until after deprecation.

### OpenDIS EntityState adapter checkpoint (2026-10-23)

FM-DIS-1's local adapter slice is accepted. The maintained container pins and
verifies OpenDIS C++ revision `d1288ace9ea9a5630c10d9d458c01be889833345`,
builds static DIS7 only, and retains license/source notices. OpenDIS owns binary
serialization/deserialization. Flight-models only maps completed physical truth
and explicit metadata into its EntityState PDU. The
[field mapping and dependency record](adapter_contract.md#flight-models-opendis-boundary-2026-10-23)
documents the build workaround, precision and supported fields.

Changed paths: `include/flight_models/dis_entity_state.h`,
`src/dis_entity_state.cpp`, the two DIS tests, CMake/Docker/Compose, and existing
README/index/adapter references. The required reusable DCM-to-Z-Y-X extraction
lives in `math/include/math/coordinates/geometry.hpp` and its implementation,
with direct shared-math tests. It validates proper rotations, preserves both
vertical-attitude signs, and uses deterministic zero roll at gimbal lock.
Position and world-frame vectors retain canonical ECEF; no geodetic position
conversion is added. Only the wire-required attitude representation and float32
fields change. No runtime, custom codec or new epoch handling was added.

Validation: `./bootstrap.sh` provisioned the pinned dependency. `./test.sh` passes
all seven flight-models CTest executables in the offline Release container,
including OpenDIS decoding and the existing sensor-models decoder/alignment code
compiled as test-only sources. The retained SuperCell v7 fixture supplies the
consumer-compatible identity/type/kinematics specimen. Cases cover 144-byte
packets, identity/type/marking, ECEF values, DR 1/4, zero fixed derivatives,
disabled/invalid inputs, ASCII bounds, non-hour-aligned epoch origins, hourly
wrap, and the int64 elapsed-time limit. Sensor alignment agrees within 1677 ns;
this is DIS wire quantization, not a fine TDOA requirement. All five shared-math
executables pass, including direct inverse-rotation reconstruction near and at
both gimbal singularities. Focused independent review found no material blockers.

Evidence: [bootstrap](../logs/20261023/dis/bootstrap.log),
[flight tests](../logs/20261023/dis/flight-tests.log),
[math tests](../logs/20261023/dis/math-tests.log).
Reproduce math checks in the maintained image with:

```sh
docker compose run --pull never --rm -T dev sh -ec '
  cmake -S /math -B /src/build/math-coordinate-check -G Ninja -DCMAKE_BUILD_TYPE=Release -DMATH_BUILD_TESTS=ON
  cmake --build /src/build/math-coordinate-check
  ctest --test-dir /src/build/math-coordinate-check --output-on-failure
'
```

Unresolved: metadata is supplied through the typed API, not yet loaded from a
normal scenario; live DIS publication, lifecycle/removal, PRD production, emitter
source ownership/carriers, and end-to-end consumer/frame acceptance remain their
integration gates. No throughput or live interoperability claim is made.

Next at this checkpoint (ideal portion implemented in the subsequent checkpoint):
**FM-NAV-PRD-1**, establish the typed navigation/error
state and ideal/nonideal reporting contract using the existing PRD fixture and
accepted timing/accuracy references, then implement and validate the navigation
model/PRD adapter. Preserve physical truth separation and source/creation times;
do not expose physical truth as a nonideal navigation fallback. Emitter ownership
and runtime/scenario integration still precede SuperCell retirement. New epoch
and restore handling stay deferred until after deprecation.

### Ideal navigation and generated PRD checkpoint (2026-10-23)

The approved `ideal_test` portion of FM-NAV-PRD-1 is accepted. No approved numeric
nonideal navigation profile was found in the owning contract; the retained PRD
fixture's covariance is a schema/mapping specimen, not equipment accuracy. This
slice therefore implements the explicitly approved ideal model and general
reported-state wire boundary without inventing error values or qualifying
nonideal performance. FM-NAV-PRD-1 stays open for that model and its statistics.

Changed paths: `include/flight_models/navigation.h`, `src/navigation.cpp`,
`include/flight_models/position_report.h`, `src/position_report.cpp`, their two
focused tests, CMake, and the existing Sleet probe/Compose/wrapper and references.
The ideal model copies enabled physical snapshot rows into separately owned
reusable SoA columns. It requires explicit `ideal_test`, retains canonical IDs
and source times, excludes disabled rows, and generates zero covariance. Staged
publication preserves the prior batch on invalid input or capacity failure;
returned solutions remain independent of later physical mutations. There is no
implicit truth fallback for absent navigation. No numerical backend or throughput
qualification is claimed.

The PRD encoder takes only reported navigation plus explicit platform SystemID,
EGI SubsystemID, security and solution-state metadata. It performs one reported
ECEF-to-geodetic position conversion and derives the report's local NED frame.
Shared math owns attitude, vector and full covariance transformations; no new
mathematical implementation was added. All nine position/velocity cross entries
are preserved. A nonzero-covariance reported-state fixture exercises this mapping
without claiming nonideal-model behavior. The encoder expects model-validated
symmetric PSD covariance and checks finite input/output. Runtime reporting-set
selection remains separate from generating local navigation for each platform.

Simulation time formatting is explicit: supplied scenario Unix epoch plus elapsed
integer nanoseconds, formatted as UCI's existing Zulu xs:dateTime with nine
fractional digits. Header creation time and position/velocity/orientation validity
time remain distinct. The model consumes source time already present in its
input; no new explicit advancement input, wall-clock read, FTRT flag or epoch
recovery behavior is added. The installed UCI 2.5 XSD `DateTimeType` permits this
precision, and the actual Sleet schema accepts the generated output.

Validation: `./test_sleet.sh logs/20261023/navigation/schema` builds/tests in the
maintained offline Release container. All nine CTest executables pass. The
wrapper then generates a PRD from the actual ideal model/encoder and publishes it
to an isolated Sleet ASB: both PositionSource choices pass, and a missing header
timestamp fails as expected. The ASB/network are removed afterward. Unit checks
cover separate state ownership, canonical ordering, disabled omission, zero
covariance, invalid DCM and capacity rollback, fixture frame/covariance mapping,
source versus creation nanoseconds, calendar rollover, negative Unix origins and
int64 timestamp overflow. Focused independent review found no material blockers.

Evidence: [full tests](../logs/20261023/navigation/schema/build-test.log),
[generated PRD](../logs/20261023/navigation/schema/generated-prd.json),
[Sleet results](../logs/20261023/navigation/schema/schema.log), and
[cleanup](../logs/20261023/navigation/schema/cleanup.log).
The normal short commands remain `./test.sh` and `./test_sleet.sh`.

Remaining gates: approved nonideal error-profile semantics/values, reproducible
entity-keyed noise and covariance consistency; operational identity/reporting-set
configuration; runtime clock, route/control ingress and DIS/PRD publication;
emitter-source ownership/carriers; consumer/system performance and retirement
acceptance. Ideal mode is explicitly test-only and must remain selected/labeled
when used for initial scenario acceptance; it cannot establish nonideal accuracy.

Exact next bounded SuperCell-migration slice: reconcile **FM-CONTRACT-1 runtime
configuration** with the existing scenario/startup adapters: platform/DIS/EGI
identity, configured local navigation/reporting membership, physical DIS output
and per-platform ASB endpoints, plus authoritative clock input. Reuse the approved
ideal profile for the initial runtime test and keep nonideal acceptance open.
Audit retained emitter source/carrier ownership before moving its behavior. That
configuration contract feeds FM-RUNTIME-1 clock/output wiring; full operational
control/emitter and live acceptance remain prerequisites to retirement. New epoch
and restore handling remain deferred until after SuperCell deprecation.

### Fresh-run execution core and scenario bindings (2026-10-23)

This budget-bounded slice implements the fresh-run execution core and scenario
binding generator. It does **not** implement a network process or replace normal
SuperCell startup. Scope was reduced at the user's 14%-remaining update to finish
and validate existing work while aiming to stop near 10%; remaining usage is not
visible to the agent. No measured token/cost saving is claimed.

Changed paths: `include/flight_models/runtime.h`, `src/runtime.cpp`,
`tests/cpp/runtime_test.cpp`, the scalar ideal-navigation helper and CMake;
`ai-bm-sim/tools/generate_flight_models_runtime_inputs.py` and its focused tests.
The generator reuses ai-bm-sim's existing scenario loader/topology normalizer,
requires complete explicit scenario-instance bindings, and preserves scenario
locations, epoch and duration. Unique runtime IDs use scenario instances rather
than repeated platform-template entity IDs. DIS identities, platform/EGI UUIDs,
ASB URLs and topics are explicit bindings; missing or duplicate mappings fail.
It does not depend on SuperCell identity export. No radio/channel definitions move.

The C++ core loads its explicit motion profile/resource bounds, initializes ECEF
state using shared math, and owns one fresh fixed cohort. Direct elapsed-time
advances produce scheduled physical DIS for each entity and PRDs only for the
configured reporting set. Every intermediate sample retains its validity time;
PRD creation uses the advance boundary. Small individual navigation samples use
the shared ideal helper without a temporary store. Epoch origin only formats
absolute timestamps; new recovery handling remains deferred. Unsupported live
commands/waypoints fail explicitly. Encoding failure poisons the runtime because
motion has already committed; retries cannot claim transactional publication.
No new backend or throughput qualification is claimed.

Validation: `./test.sh` passes all ten flight-models CTest executables in the
maintained offline Release container. The runtime test checks initial samples,
100/200 ms cadence, partitioned versus direct advancement, independent red DIS
versus configured PRD reporting, delayed creation versus validity, and invalid
capacity/profile/unsupported commands. Evidence:
[flight tests](../logs/20261023/runtime/tests.log).
Generator validation: from ai-bm-sim,
`PYTHONPATH=. pytest -q tests/test_generate_flight_models_runtime_inputs.py`
passes all three tests, covering normalized locations, explicit bindings, epoch,
policy and ambiguity failures. Focused independent review found no material
blockers. No live runtime transport/system test was run in this slice.

Generator invocation (all paths explicit):

```sh
python tools/generate_flight_models_runtime_inputs.py \
  --scenario config/scenarios/bma_policy_4blue_80nmi.yaml \
  --bindings <platform-bindings.json> --policy <flight-runtime-policy.json> \
  --output logs/<date>/<run>/flight-models.json --run-id <run-id>
```

The policy supplies capacity, profile/schema paths, event limits and transport
settings (NATS URL/clock subject, DIS host/port, OWP service ID, clock poll and
operation timeouts). Bindings map scenario instance IDs to initial fixed/moving
mode, heading/speed, DIS identity/type/marking and optional PRD identity/security/
ASB/topic metadata. Scenario epoch/duration come from the selected scenario.
Transport fields are carried through but not consumed by this library. Complete
versioned deployment schema validation, policy/binding examples and path/mount
admission remain prerequisites to launching a runtime process.

Exact next slice: add deployment configuration/schema and a thin C++ process
around this core: consume the existing `sim.time.tick` protobuf on simulation
NATS, preserve fresh-run causal admission/pause semantics, send DIS directly via
UDP, and publish unchanged UCI PRDs to explicitly bound per-platform ASBs.
Current ecosystem startup still assumes one legacy ASB and SuperCell; it must
not silently substitute that topology for the accepted platform-local target.
Then add opt-in scenario startup and isolated transport acceptance before the
full emitter/control/sensor/communications/performance retirement gates.
SuperCell remains current/reference; new epoch/restore work remains deferred.

### Fresh-run clock and output process (2026-10-24)

The next FM-RUNTIME-1 slice is locally accepted. `flight-models-run` wraps the
existing domain runtime with the authoritative `sim.time.tick` protobuf input,
direct DIS UDP output and explicitly configured platform-local OWP publishers.
OpenDIS still owns serialization. No domain library acquires a transport or
wall-clock dependency, and no additional per-tick ACK protocol is introduced.
Normal ecosystem startup and SuperCell retirement remain pending.

Changed paths: process config/clock/transport public headers and sources,
`runtime_main.cpp`, deployment schema/policy, owning runtime Compose/run wrapper,
process unit tests and isolated smoke workflow; Docker/CMake/README/index.
The ai-bm-sim generator and its tests now retain required clock replay and pending
message/byte bounds. The existing core gained an explicit creation-time overload
for final-interval clipping. A live-test regression also corrected initial motion
admission: fixed ground rows bypass moving-flight minimum speed/HAE controls;
finite state validation and their prohibition on route/command programs remain.

Deployment uses `config/runtime.schema.json`, explicit identity/reporting/ASB
bindings, and versioned resource/time policies. Origins must agree as UTC text
and signed Unix ns, with exact integer checks and unique entity/DIS/reporting
identities. Relative profile paths resolve at the generated configuration file;
the container example uses `/src/config/...` paths. `config/runtime_policy.json`
is a synthetic three-platform example, not qualified deployment sizing. Input
location remains geodetic radians/HAE; the core converts it once to canonical
ECEF. Standalone wrapper launch requires an explicitly selected existing
simulation Docker network; it does not create platform ASBs implicitly.

Clock admission starts at nonadvancing time/tick zero. Paused/resume heartbeats
advance no domain work; advancing messages require contiguous IDs/times and a
matching run/origin/frozen initial timeline epoch. Accepted advancing replays are
idempotent within a configured finite history. Gaps, conflicts, expired replays,
changed epochs and late joins fail clearly rather than guessing missing motion.
Absolute creation-time representability is checked before processing state can
change. Rate/mode never enter domain advancement. A final overshooting interval
stops physical work at scenario end while retaining announced PRD creation time.
New epoch/restore handling remains deferred until after SuperCell deprecation.

Transport uses bounded NATS ingress and global pending PRD message/byte counts
across ASBs, reserved before asynchronous queue posting. Outbound WebSocket
writes are pipelined with ordered ACK retirement and configured oldest-ACK/write
deadlines. Incoming frame sizes are bounded by the byte policy. Subscription
registration is flushed before readiness. Connection loss, dropped clock input,
queue overflow and ASB rejection are fatal and visible during idle clock polling;
there is no implicit reconnect/redelivery. Stop requests finish active domain
work and drain admitted publications within the operation deadline. DIS goes
straight to its UDP destination; interplatform impairment remains comms-models'
responsibility. Publisher ACKs establish local ASB admission, not receiver work
completion. Initial plain-WebSocket DNS/IPv4 endpoints are supported; TLS/IPv6,
new recovery behavior and throughput qualification are not claimed.

Validation: `./test_process.sh logs/20261024/process-accepted` builds/tests in the
maintained offline Release image, then runs a unique isolated Compose project
with real NATS, two Sleet ASBs, the actual process and a finite observer. All
**12 CTest executables pass**. The process emits **15 independently decoded DIS
PDUs and six exact PRDs** for two reporting moving entities plus one fixed
nonreporting entity. The test checks five physical sample times per entity,
identity/force/static DR, platform-local report contents, accepted duplicate and
pause behavior, and 400 ms validity versus 450 ms final creation time. A separate
publication-rejection case exits with an error while no further clock arrives.
The child must exit normally at scenario end. Test cleanup removes only its
PID-scoped project. Functional acceptance is not a throughput/FTRT measurement.

Evidence: [12 tests](../logs/20261024/process-accepted/build-test.log),
[transport observer](../logs/20261024/process-accepted/compose.log),
[process counts](../logs/20261024/process-accepted/flight-process.log),
[rejection propagation](../logs/20261024/process-accepted/flight-process-forbidden-topic.log),
[cleanup](../logs/20261024/process-accepted/cleanup.log).
From ai-bm-sim, `PYTHONPATH=. pytest -q tests/test_generate_flight_models_runtime_inputs.py`
also passes all three generator tests. Independent review's UTC-creation bound
finding is resolved with an admission check/regression; its fixed-ground-row
review found the envelope correction appropriately scoped. No material review
issues remain.

Exact next slice: wire the owning runtime service and generated configuration
into **ai-bm-sim's opt-in normal scenario startup**, with explicit per-platform
ASB composition and authoritative clock startup ordering. Reconcile the legacy
SuperCell dynamics-stage bridge without adding an ordinary flight tick-ACK
barrier or running two truth producers for one identity. Then exercise actual
sensor-models DIS/PRD consumption and comms-models interplatform delivery in a
bounded system scenario. Operational control/route admission, emitter-source
ownership/carriers, nonideal navigation and performance/retirement gates remain
open. SuperCell stays current/reference until that acceptance is complete.

### Opt-in startup orchestration handoff (2026-10-25)

ai-bm-sim now implements the opt-in normal-entrypoint profile described by the
previous checkpoint. Its [owning startup checklist and acceptance command](../../ai-bm-sim/docs/development_plans/foundation_roadmap/runtime_messaging_and_replay_handling.md#opt-in-flight-models-startup-checkpoint-2026-10-25)
record generated per-platform ASBs, paused authoritative-clock startup,
time-zero observation and finite DIS/local PRD acceptance. Docker startup now
passes with 63 DIS datagrams, 11 PRDs per reporting platform, normal process exit
and isolated cleanup; the owning checkpoint links the accepted artifacts and
resolved observer-import issue. No flight-models source changed in this slice.
The legacy default still uses SuperCell. Sensor/comms, operational control/emitter
and performance retirement gates remain open. Exact next action: implement actual
sensor-models truth/navigation and clock consumption. New rewind/restore work
remains deferred.

Shared clock adoption checkpoint (2026-10-25): runtime_main now uses
`RuntimeSupport::sim_time` from ai-bm-sim's independently buildable library.
The local clock_admission implementation/header and duplicate common/sim_time
Protobuf generation were removed. Existing NATS/ASB transports and fresh-run
admission semantics are retained. CMake takes an explicit runtime-support source
path; Compose mounts only that library and the existing authoritative schemas.
Release build/12 CTests and the isolated live process smoke (15 DIS/six PRDs)
pass. Evidence and the next sensor integration slice are recorded in the
[owning shared-runtime checkpoint](/home/jeffs/git/ai-bm-sim/docs/development_plans/foundation_roadmap/runtime_messaging_and_replay_handling.md#shared-runtime-time-and-header-support-2026-10-25).
This is not operational control/emitter, new epoch/restore or SuperCell retirement
acceptance.

### Configured ECEF navigation error and disabled terms (2026-10-25)

FM-NAV-PRD-1 now accepts the bounded synthetic error model selected by the user:
fixed ECEF position/velocity biases plus independent zero-mean isotropic Gaussian
samples at report validity times. Physical motion/DIS, attitude and source time
are unchanged. The model adds no coordinate conversions; the existing PRD
boundary still converts reported state and rotates covariance once. ECEF 6x6
[position, velocity] covariance contains sigma squared on the respective axes,
zero cross terms, and no contribution from deterministic bias. Biases remain
uncorrected; this covariance describes the random component about the biased
mean, not total accuracy about physical truth.

Changed paths: typed profile schema/loader and navigation configuration,
`navigation.h/.cpp`, runtime model construction/call, configuration target link,
zero/nonzero example profiles, and navigation/profile/PRD/runtime tests. Public
API is `NavigationModel` plus fixed-capacity `NavigationBatch`, replacing the
ideal-only batch while retaining its staged 15-column SoA storage. Model flags
and covariance are cached once. Each zero bias vector bypasses addition; each
zero sigma bypasses its entire RNG/distribution/draw path independently. All-zero
settings use the exact scalar/SoA copy path even with `bias_gaussian` selected.

Motion profiles now require a six-field `navigation` object: model,
`position_bias_ecef_m`, `velocity_bias_ecef_mps`, `position_sigma_m`,
`velocity_sigma_mps`, and `seed`. `ideal_test` requires all errors zero.
The existing `synthetic_fixed_wing.json` remains the explicit zero-error default;
`synthetic_navigation_error.json` demonstrates small nonzero settings, labeled
synthetic rather than equipment-derived. Runtime settings select either by
`profile_path`; no runtime schema or ai-bm-sim generator behavior changes.
Seed tokens require exact nonnegative uint64 values; the typed loader checks the
range because Valijson numeric schema bounds cannot represent UINT64_MAX.

Samples use full-width `mt19937_64` with fixed byte-wise entity/time keys and
separate position/velocity streams; no mutable RNG state or wall time enters the
model. Results reproduce under identical configuration, identity, validity time
and toolchain/build regardless of batching, interval partitioning, duplicates or
processing order. `normal_distribution` does not promise cross-library bitwise
parity. White sample noise and profile-wide fixed biases are explicit synthetic
assumptions; no drift, correlations, correction or equipment calibration is
claimed. Numerical backend/crossover and throughput qualification remain open.

Validation: `./test.sh` passes all **12 CTest executables** in the maintained
Release/Werror Docker environment. Tests cover zero-error exactness/seed
invariance, bias-only output, independently enabled noise streams, key changes,
scalar/batch equality and order, transactional capacity failure, malformed
configuration/overflow, covariance mapping and runtime PRD changes with identical
physical DIS. A fixed-seed 12,000-sample check validates Gaussian mean/variance
within documented six-standard-error tolerances. Independent model/interface
review found no material issue.
Evidence: [build and tests](../logs/20261025/navigation-error/build-test.log).

Real Docker startup with the nonzero example passes through simulation NATS,
three ASBs, the real clock and C++ process: **63 DIS datagrams and 11 PRDs on each
reporting platform's local ASB**, with strict Sleet UCI admission, normal process
exit and isolated cleanup. Runtime unit tests separately prove the PRD value
changes and unchanged DIS bytes. Evidence: [accepted result](../../ai-bm-sim/logs/20261025/flight-startup-201239-3539d942/observer/result.json),
[process/ASB/cleanup log](../../ai-bm-sim/logs/20261025/flight-startup-201239-3539d942/launch.log),
[source and binary manifest](../../ai-bm-sim/logs/20261025/flight-startup-201239-3539d942/manifest.json).
The selected startup/policy inputs are under
`ai-bm-sim/logs/20261025/navigation-error-acceptance/`; reproduce with the existing
entrypoint and `--flight-models-profile` pointing to its `startup_profile.json`.

This completes the requested navigation-error addition, not SuperCell retirement.
Exact next development step remains actual sensor-models truth/navigation and
clock consumption; communications, operational control/emitter and workload
acceptance gates remain in their owning sequence. New rewind/restore stays deferred.
