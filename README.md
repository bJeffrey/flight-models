# Flight Models

C++17 flight-models migration work currently contains contract fixtures and
validation tools, a typed motion-profile library, bounded SoA state storage and
an initial scalar motion kernel, bounded waypoint/event driver, an OpenDIS7
EntityState adapter and separate ideal-test navigation/PRD generation. A fresh-run
C++ process now connects the authoritative NATS clock, DIS UDP and configured
platform ASBs. Nonideal navigation and ecosystem cutover remain pending. Start from the
[planning index](docs/README.md).

The fresh-run `Runtime` library now joins initial scenario state, motion sampling,
DIS encoding and explicitly configured PRD reporting. `advance_to()` consumes
elapsed simulation nanoseconds and returns owned output records; it does not
open network connections. `flight-models-run` owns those connections. The ai-bm-sim generator
`tools/generate_flight_models_runtime_inputs.py` normalizes scenario locations
and joins explicit DIS/EGI/ASB bindings without the SuperCell exporter.
See the [current runtime checkpoint](docs/implementation_plan.md#fresh-run-execution-core-and-scenario-bindings-2026-10-23)
for the initial core acceptance and the
[process checkpoint](docs/implementation_plan.md#fresh-run-clock-and-output-process-2026-10-24)
for schema/transport acceptance and remaining startup integration work.

Provision the maintained Docker toolchain once, then run offline contract tests:

```sh
./bootstrap.sh
./test.sh
```

Run the isolated real-transport process check with cached NATS and Sleet images:

```sh
./test_process.sh
```

This starts a dedicated Compose project containing NATS, two ASBs and a finite
observer. It runs the actual process, decodes DIS through sensor-models and checks
platform-local PRDs. Artifacts go under a dated directory; cleanup removes only
that test project. This is functional acceptance, not throughput qualification.
Builds read the authoritative clock protobuf from sibling `ai-bm-sim` and reuse
the test-only OWP observer from sibling `comms-models`; production libraries do
not depend on communications routing. Repository locations can be overridden
with `AI_BM_SIM_REPO` and `FLIGHT_COMMS_REPO` in the dev Compose workflow.

For a generated deployment configuration and an existing simulation network:

```sh
FLIGHT_RUNTIME_NETWORK=<existing-network> ./run.sh <runtime.json>
```

Build with `./test.sh` first. The process validates
[`config/runtime.schema.json`](config/runtime.schema.json) before opening sockets.
[`config/runtime_policy.json`](config/runtime_policy.json) is a three-platform
synthetic policy example; scenario identity/EGI/ASB bindings remain explicit
ai-bm-sim inputs. The wrapper mounts the repository at `/src`, matching the
example profile paths. Normal ecosystem startup still selects SuperCell.

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

`NavigationModel` and `NavigationBatch` produce reported navigation separately
from physical truth. `ideal_test` is exact; `bias_gaussian` adds configured ECEF
position/velocity biases and independent isotropic Gaussian error. The fixed
biases do not enter covariance. The batch retains staged SoA storage and omits
disabled entities. These synthetic models do not qualify equipment accuracy.

The `navigation` object in a selected motion profile has explicit units/settings:

```json
"navigation": {
  "model": "bias_gaussian",
  "position_bias_ecef_m": [1.0, -2.0, 0.5],
  "velocity_bias_ecef_mps": [0.01, -0.02, 0.005],
  "position_sigma_m": 1.0,
  "velocity_sigma_mps": 0.05,
  "seed": 20261025
}
```

These example values are synthetic test choices. Select
`/src/config/profiles/synthetic_navigation_error.json` as the runtime policy's
`profile_path` to exercise them; the standard `synthetic_fixed_wing.json` keeps
all errors zero. Settings apply to the selected runtime profile's platforms;
identity/time keys separate their random samples. Position/velocity samples and
ECEF axes are independent, with a new draw at each report validity time. The seed,
entity ID and validity time make duplicates, batching, replay of the same inputs
and interval partitioning reproducible within the same toolchain/build. The C++
standard does not require `normal_distribution` bitwise parity across libraries.

Enabled flags and covariance are cached once. A zero bias vector skips addition;
each zero sigma independently skips RNG/distribution construction and draws.
All-zero errors take the exact-copy path even under `bias_gaussian`. No error
sampling or frame conversion is performed for disabled terms.
[`encode_position_report_detailed`](include/flight_models/position_report.h)
retains its existing reported-ECEF to WGS-84 HAE/NED conversion and covariance
rotation through shared math. Physical DIS, attitude and validity time remain
unchanged; velocity noise is sampled directly, not differentiated from position.
No correction or estimation loop is added. Covariance describes the random
component around the biased mean, not total accuracy about physical truth.


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
