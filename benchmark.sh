#!/usr/bin/env bash
# Run the same configured source/workload in the cached, network-disabled dev image.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source_dir="${FLIGHT_MODELS_BENCH_SOURCE:-/src}"
math_dir="${FLIGHT_MATH_BENCH_SOURCE:-/math}"
build_dir="${FLIGHT_MODELS_BENCH_BUILD:-/tmp/flight-models-benchmark}"
workload="${FLIGHT_MODELS_BENCH_WORKLOAD:-benchmarks/workload.json}"
schema="${FLIGHT_MODELS_BENCH_SCHEMA:-benchmarks/workload.schema.json}"
states="${FLIGHT_MODELS_BENCH_STATES:-/src/logs/$(date -u +%Y%m%d)/coordinates/current/states.csv}"
profile="${FLIGHT_MODELS_BENCH_PROFILE:-config/profiles/synthetic_fixed_wing.json}"
profile_schema="${FLIGHT_MODELS_BENCH_PROFILE_SCHEMA:-config/motion_profile.schema.json}"
mkdir -p "$repo_dir/logs/$(date -u +%Y%m%d)/coordinates/current"
docker compose -f "$repo_dir/compose.yaml" run --rm --no-deps --pull never \
  -e SOURCE_DIR="$source_dir" -e MATH_DIR="$math_dir" -e BUILD_DIR="$build_dir" \
  -e WORKLOAD="$workload" -e WORKLOAD_SCHEMA="$schema" -e STATES="$states" \
  -e PROFILE="$profile" -e PROFILE_SCHEMA="$profile_schema" dev bash -lc '
    cmake -S "$SOURCE_DIR" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTING=OFF -DFLIGHT_MODELS_BUILD_BENCHMARK=ON \
      -DFLIGHT_MATH_SOURCE_DIR="$MATH_DIR" >/dev/null
    cmake --build "$BUILD_DIR" --target motion-benchmark --parallel 2 >/dev/null
    "$BUILD_DIR/motion-benchmark" "$SOURCE_DIR/$WORKLOAD" "$SOURCE_DIR/$WORKLOAD_SCHEMA" \
      "$SOURCE_DIR/$PROFILE" "$SOURCE_DIR/$PROFILE_SCHEMA" "$STATES"
  '
