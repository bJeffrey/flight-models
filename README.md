# Flight Models

C++17 flight-models migration work currently contains contract fixtures and
validation tools, a typed motion-profile library and bounded SoA state storage.
Dynamics, OpenDIS encoding and runtime adapters are not
implemented. Start from the [planning index](docs/README.md).

Provision the maintained Docker toolchain once, then run offline contract tests:

```sh
./bootstrap.sh
./test.sh
```

With the existing Sleet image available locally, validate the PRD fixture against
its installed UCI 2.5 schema in an isolated ASB:

```sh
./test_sleet.sh
```

The latter builds/tests, checks both PositionSource identity choices and a
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
data agreement and warmed-path allocation behavior; kernel performance remains
unqualified until dynamics/backends are implemented.
