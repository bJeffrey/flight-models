# Flight Models

C++17 flight-models migration work currently contains contract fixtures and
validation tools, a typed motion-profile library, bounded SoA state storage and
an initial scalar motion kernel, bounded waypoint/event driver, an OpenDIS7
EntityState adapter and separate ideal-test navigation/PRD generation. Runtime
transport adapters and nonideal navigation remain pending. Start from the
[planning index](docs/README.md).

Provision the maintained Docker toolchain once, then run offline contract tests:

```sh
./bootstrap.sh
./test.sh
```

The DIS adapter uses OpenDIS for serialization. Tests also feed its output through
the existing sensor-models decoder; the Docker workflow mounts the sibling
`sensor-models` repository read-only for these test sources (override its path
with `FLIGHT_SENSOR_REPO`). Production libraries do not depend on sensor-models.
See the [field mapping and dependency record](docs/adapter_contract.md#flight-models-opendis-boundary-2026-10-23).

With the existing Sleet image available locally, validate the PRD fixture against
its installed UCI 2.5 schema in an isolated ASB:

```sh
./test_sleet.sh
```

The latter builds/tests, generates a PRD through the ideal navigation model and
production encoder, checks both PositionSource identity choices and a
missing-header-timestamp rejection, saves dated logs, and tears down its ASB.
It exposes no host ports and does not start or modify a normal scenario.

The initial [synthetic motion profile](config/profiles/synthetic_fixed_wing.json)
uses explicit test limits, integration work bounds and separate DIS/PRD cadences.
See the [motion/input contract](docs/motion_input_contract.md) for guidance and
ordinary-run sensor admission semantics. Profile loading is available through
`flight_models::load_motion_profile`; the maintained test command checks both
profile validation and the existing state/time specimens.

[StateStore](docs/state_storage.md) supplies backend-independent indexed/cohort
views, staged commits and copied snapshots. `config/storage_policy.json` records
example resource bounds. The storage tests check lifecycle safety, scalar/batch
data agreement and warmed-path allocation behavior. The first
[`advance_motion`](include/flight_models/bounded_motion.h) kernel advances one
moving profile cohort transactionally with bounded midpoint steps. It supports
heading, horizontal speed and HAE targets. `MotionDriver` adds ordered waypoints,
timed commands and independent typed physical/navigation sample cadences over a
fixed fresh-run cohort. Samples are provisional until the whole interval commits;
they are not DIS/UCI messages. The [input contract](docs/motion_input_contract.md)
documents supported programs, explicit resource bounds and deferred VMS/runtime
behavior. Backend selection and population crossover remain unqualified.

`IdealNavigationBatch` copies completed snapshots into separate reusable SoA
columns. It requires explicit `ideal_test` selection, omits disabled entities,
and reports exact position/velocity/attitude with zero covariance. That is an
explicit test assumption, not a claim about navigation equipment accuracy.
[`encode_position_report_detailed`](include/flight_models/position_report.h)
accepts only reported navigation and explicit identity/security metadata. It
projects the report to WGS-84 HAE/NED using shared math and formats creation and
validity timestamps from authoritative simulation time. It reads no wall clock.
Runtime reporting membership, nonideal error profiles and live publication remain
open; see the [navigation checkpoint](docs/implementation_plan.md#ideal-navigation-and-generated-prd-checkpoint-2026-10-23).

Run the bounded scalar benchmark in the same cached Docker image:

```sh
./benchmark.sh
```

The schema-validated workload in `benchmarks/workload.json` covers small and
large populations using the validated synthetic motion profile. Release timings
include command updates, cohort gather, integration and commit; entity creation
and output are excluded. One warmup precedes five measured runs on a continuing
trajectory. Final state rows are written under the dated `logs/` directory.
This is a scalar-kernel benchmark, not a backend crossover or scenario/FTRT
qualification. The owning plan links the frozen baseline, comparison and accuracy
evidence.
