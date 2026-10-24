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
- [ ] **FM-DYNAMICS-1 — flight-models:** after FM-SOA-1, implement the transport-independent
  bounded kinematic core and waypoint guidance from FM-CONTRACT-1. Verify straight
  motion, finite-radius turns, heading wrap, acceleration/deceleration, climb and
  descent, limit saturation and waypoint progression. Compare identical elapsed
  scenario time across declared substep policies with explicit tolerances.
  Stop at local deterministic model acceptance, without aerodynamic claims.
  The first scalar control/motion kernel is implemented in the checkpoint below;
  waypoint progression and command/output event boundaries remain unfinished.
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

Next implementation slice: FM-DYNAMICS-1 waypoint guidance and scheduled-event
boundaries. The cross-service DIS/PRD/OMR conversion inventory remains a separate
pending task before adapter acceptance; numerical backend/population dispatch
also remains unqualified. New epoch/restore handling stays deferred until after
SuperCell deprecation.
