# Flight Models

C++17 flight-models migration work currently contains contract fixtures and
validation tools. Dynamics, OpenDIS encoding and runtime adapters are not
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
