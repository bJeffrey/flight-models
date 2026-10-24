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
records the current NATS and simulation-clock contract. No runtime is implemented yet.

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
views, reusable cohort staging and owning snapshots. The next local development
slice is FM-DYNAMICS-1. Its first scalar motion kernel is implemented; waypoint
and event-boundary integration remain open in the owning plan. Numerical
backend/dispatch qualification remains pending.

The [shared-math adoption and scalar optimization checkpoint](implementation_plan.md#shared-math-adoption-and-scalar-conversion-checkpoint-2026-10-23)
is complete, with passing math/flight tests and a frozen-baseline benchmark.
Waypoint/event implementation is next. The end-to-end DIS/PRD/OMR conversion
inventory and numerical backend/dispatch qualification remain pending; canonical
ECEF truth and reported-navigation separation are preserved.
