# HORUS CLI — build and run image.
#
# HORUS shipped no Dockerfile. `tests/docker/Dockerfile.ubuntu` exists but is a
# test harness: it rebuilds the whole workspace and defaults its CMD to
# `cargo test`. This is the one for actually running the tool.
#
#   docker build -t horus .
#   docker run --rm -it -v "$PWD:/work" -w /work horus doctor
#
# Shared memory is how HORUS nodes talk, so a container that runs nodes needs a
# real /dev/shm — the default 64 MB is small for image or point-cloud topics:
#
#   docker run --rm -it --shm-size=1g -v "$PWD:/work" -w /work horus run
#
# For multi-process IPC across containers, share the namespace as well:
#
#   docker run --rm --ipc=host --shm-size=1g ... horus run

# ── Builder ────────────────────────────────────────────────────────────────
# Pinned to the workspace MSRV, so this image also checks that the floor is real
# rather than aspirational. A test asserts the two agree; they had already
# drifted once, when the MSRV moved to 1.90 and this line kept saying 1.92.
FROM rust:1.90-slim-bookworm AS builder

# libudev is needed by the hardware discovery path; pkg-config finds it.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libudev-dev \
        pkg-config \
        git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src

# Copy the manifests first so dependency compilation caches independently of
# source edits.
COPY Cargo.toml Cargo.lock ./
COPY horus/Cargo.toml                horus/Cargo.toml
COPY horus_core/Cargo.toml           horus_core/Cargo.toml
COPY horus_macros/Cargo.toml         horus_macros/Cargo.toml
COPY horus_cpp_macros/Cargo.toml     horus_cpp_macros/Cargo.toml
COPY horus_cpp/Cargo.toml            horus_cpp/Cargo.toml
COPY horus_manager/Cargo.toml        horus_manager/Cargo.toml
COPY horus_types/Cargo.toml          horus_types/Cargo.toml
COPY horus_net/Cargo.toml            horus_net/Cargo.toml
COPY horus_sys/Cargo.toml            horus_sys/Cargo.toml
COPY horus_py/Cargo.toml             horus_py/Cargo.toml
COPY benchmarks/Cargo.toml           benchmarks/Cargo.toml

COPY . .

RUN cargo build --release -p horus_manager --locked

# ── Runtime ────────────────────────────────────────────────────────────────
FROM debian:bookworm-slim AS runtime

# libudev1 is the runtime half of libudev-dev; ca-certificates is needed to
# reach the registry; git for git-sourced dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libudev1 \
        ca-certificates \
        git \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /src/target/release/horus /usr/local/bin/horus

# Ship the man page and completions the install script would have placed.
RUN mkdir -p /usr/local/share/man/man1 \
    && horus man > /usr/local/share/man/man1/horus.1 \
    && mkdir -p /usr/share/bash-completion/completions \
    && horus completion bash > /usr/share/bash-completion/completions/horus

WORKDIR /work
ENTRYPOINT ["horus"]
CMD ["--help"]
