# Test-only toolchain. Runtime and OpenDIS targets are added in their owning slices.
FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build nlohmann-json3-dev libvalijson-dev libboost-system-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
