#!/bin/sh
# Build locally, then validate PRD specimens against an isolated real Sleet ASB.
set -eu
cd "$(dirname "$0")"
output=${1:-"logs/$(date +%Y%m%d)/uci-schema-$(date +%H%M%S)"}
mkdir -p "$output"
output=$(cd "$output" && pwd)
project="flight-contract-$$"
cleanup() {
  docker compose -p "$project" -f compose.schema-tests.yaml logs --no-color --tail=60 > "$output/services.log" 2>&1 || true
  docker compose -p "$project" -f compose.schema-tests.yaml down --remove-orphans > "$output/cleanup.log" 2>&1 || true
}
trap cleanup EXIT
./test.sh > "$output/build-test.log" 2>&1
# Test-only watchdog bounds health polling and synchronous probe I/O.
if timeout 60 docker compose -p "$project" -f compose.schema-tests.yaml run --pull never --rm -T schema-test > "$output/schema.log" 2>&1; then
  tail -n 1 "$output/schema.log"
else
  status=$?
  tail -n 15 "$output/schema.log" >&2
  exit "$status"
fi
printf 'Schema-test artifacts: %s\n' "$output"
