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
