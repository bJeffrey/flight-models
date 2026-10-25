# Maintained development/runtime toolchain; normal tests use cached dependencies.
FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build nlohmann-json3-dev libvalijson-dev libboost-system-dev \
    ca-certificates curl libnats-dev libprotobuf-dev protobuf-compiler pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Pin and verify the source archive. Bootstrap needs network access; ordinary
# builds/tests use this installed static DIS7 library with networking disabled.
RUN curl -fLsS https://codeload.github.com/open-dis/open-dis-cpp/tar.gz/d1288ace9ea9a5630c10d9d458c01be889833345 -o /tmp/opendis.tar.gz \
    && echo 'eb6bf63ea26299cd980c9965de5e754646ccb4404f11be8ca246bb46dd21ee2a  /tmp/opendis.tar.gz' | sha256sum -c - \
    && mkdir /opt/opendis-source \
    && tar -xzf /tmp/opendis.tar.gz --strip-components=1 -C /opt/opendis-source \
    && cmake -S /opt/opendis-source -B /tmp/opendis-build -G Ninja \
       -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
       -DBUILD_OPENDIS6=OFF -DBUILD_OPENDIS7=ON -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF \
    && cmake --build /tmp/opendis-build --parallel 2 \
    && cmake --install /tmp/opendis-build \
    && mkdir -p /usr/local/share/licenses/opendis \
    && cp /opt/opendis-source/LICENSE /usr/local/share/licenses/opendis/LICENSE \
    && rm -rf /tmp/opendis-build /tmp/opendis.tar.gz
WORKDIR /src
