#!/bin/sh
# Provision the maintained contract-test toolchain; subsequent tests run offline.
set -eu
cd "$(dirname "$0")"
docker compose build dev
