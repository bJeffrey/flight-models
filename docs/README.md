# Flight Models Planning Index

## Active

- [SuperCell Replacement Implementation Plan](implementation_plan.md): owns
  simulated VMS/guidance, bounded vehicle dynamics, DIS/UCI adapters, deterministic
  time handling and local migration validation.

Cross-repository sequencing and retirement acceptance belong only to the
[BM Ecosystem Master Plan](/home/jeffs/git/ai-bm-sim/docs/architecture/bm_ecosystem_master_plan.md#supercell-replacement-with-flight-models-2026-10-10).
The [Ecosystem Message Architecture](/home/jeffs/git/ai-bm-sim/docs/architecture/ecosystem_message_architecture.md#flight-models-target-boundary-2026-10-10)
owns service boundaries and transport ownership; its [shared bus and timing
decision](/home/jeffs/git/ai-bm-sim/docs/architecture/ecosystem_message_architecture.md#shared-buses-and-flight-timing-2026-10-21)
records the current NATS and simulation-clock contract. The fresh-run runtime and
finite opt-in startup are implemented; full replacement acceptance remains open.

The first FM-CONTRACT-1 slice adds the [state/time contract](state_time_contract.md),
[existing adapter inventory](adapter_contract.md) and executable C++ contract/UCI
schema checks. See the [repository commands](../README.md) and latest checkpoint
in the owning implementation plan. Route-planning PRD identity/heading
compatibility now passes local tests. Live input integration and operational
control/emitter mappings keep the full contract gate open. New cross-service
epoch/restore handling follows SuperCell deprecation; it is not a prerequisite
for the replacement runtime.

The [bounded motion/input contract](motion_input_contract.md) and typed profile
loader establish the local motion prerequisites. Remaining operational
control/emitter contracts stay open for their adapters and live acceptance.

FM-SOA-1 follows the implementation plan's
[numerical backend and population policy](implementation_plan.md#numerical-backends-and-population-aware-execution-2026-10-23):
standard C++/Eigen/MKL remain behind a common API, and small indexed updates and
large batches require separate cost/qualification checks.


[SoA physical-state storage](state_storage.md) now implements FM-SOA-1 with indexed
views, reusable cohort staging and owning snapshots. FM-DYNAMICS-1's initial
scalar motion and waypoint/event model now passes local acceptance. Numerical
backend/dispatch qualification remains pending.

The [shared-math adoption and scalar optimization checkpoint](implementation_plan.md#shared-math-adoption-and-scalar-conversion-checkpoint-2026-10-23)
is complete, with passing math/flight tests and a frozen-baseline benchmark.
The [bounded waypoint/event-driver checkpoint](implementation_plan.md#bounded-waypoint-and-event-driver-acceptance-2026-10-23)
records the subsequent model acceptance. The
[OpenDIS EntityState adapter checkpoint](implementation_plan.md#opendis-entitystate-adapter-checkpoint-2026-10-23)
now records local serialization and existing-consumer decoding acceptance.
The [ideal navigation and generated PRD checkpoint](implementation_plan.md#ideal-navigation-and-generated-prd-checkpoint-2026-10-23)
now accepts separate batched ideal state and schema-tested PRD production.
The [configured ECEF navigation-error checkpoint](implementation_plan.md#configured-ecef-navigation-error-and-disabled-terms-2026-10-25)
adds synthetic position/velocity bias and Gaussian sampling with explicit zero
fast paths; equipment-specific accuracy qualification remains open.
The [fresh-run execution core and scenario bindings checkpoint](implementation_plan.md#fresh-run-execution-core-and-scenario-bindings-2026-10-23)
joins scheduled motion, DIS and configured PRDs under direct elapsed-time calls.
The [fresh-run clock/output process checkpoint](implementation_plan.md#fresh-run-clock-and-output-process-2026-10-24)
now records deployment-schema, authoritative clock, direct UDP and platform ASB
acceptance using an isolated process smoke test. Finite opt-in startup also
passes; operational controls/emitter contracts and full scenario acceptance
remain open.
The end-to-end DIS/PRD/OMR conversion inventory and runtime/throughput gates remain
pending; canonical ECEF truth and reported-navigation separation are preserved.
