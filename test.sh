#!/bin/sh
# Contract fixtures only: no scenario processes, network services or dynamics.
set -eu
cd "$(dirname "$0")"
docker compose run --pull never --rm -T dev sh -ec '
  cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
  cmake --build build
  ctest --test-dir build --output-on-failure
'
