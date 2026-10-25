#!/bin/sh
# Build with the maintained dev image, then run only the isolated transport smoke test.
set -eu
cd "$(dirname "$0")"
artifact_dir=${1:-"logs/$(date +%Y%m%d)/process-smoke-$$"}
mkdir -p "$artifact_dir"
artifact_dir=$(cd "$artifact_dir" && pwd)
export FLIGHT_PROCESS_ARTIFACTS="$artifact_dir"
project="flight-process-$$"
cleanup() {
  status=$?
  trap - EXIT INT TERM
  timeout 10s docker compose -p "$project" -f compose.process-tests.yaml down \
    --remove-orphans --timeout 2 >"$artifact_dir/cleanup.log" 2>&1 || true
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
./test.sh >"$artifact_dir/build-test.log" 2>&1 || {
  tail -n 40 "$artifact_dir/build-test.log"
  exit 1
}
timeout 60s docker compose -p "$project" -f compose.process-tests.yaml up --pull never \
  --abort-on-container-exit --exit-code-from process-smoke \
  >"$artifact_dir/compose.log" 2>&1 || {
  tail -n 100 "$artifact_dir/compose.log"
  exit 1
}
grep -E 'runtime process smoke passed|runtime-process-smoke:' "$artifact_dir/compose.log" || tail -n 10 "$artifact_dir/compose.log"
printf 'Process-smoke artifacts: %s\n' "$FLIGHT_PROCESS_ARTIFACTS"
