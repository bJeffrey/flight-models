#!/bin/sh
# Run one preconfigured fresh scenario; endpoints/network belong to orchestration.
set -eu
repo_dir=$(CDPATH= cd -- "$(dirname "$0")" && pwd)
if [ "$#" -ne 1 ]; then
  echo 'usage: FLIGHT_RUNTIME_NETWORK=<existing-network> ./run.sh <generated-runtime.json>' >&2
  exit 2
fi
# Resolve caller input before moving to the repository-owned Compose directory.
config_file=$(realpath "$1")
export FLIGHT_RUNTIME_CONFIG="$config_file"
cd "$repo_dir"
exec docker compose -f compose.runtime.yaml run --pull never --rm flight-models
