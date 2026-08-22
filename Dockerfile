# HORUS in a container. Two images, because they answer different questions.
#
# HORUS shipped no Dockerfile. `tests/docker/Dockerfile.ubuntu` exists but is a
# test harness: it rebuilds the whole workspace and defaults its CMD to
# `cargo test`. These are the ones for using the tool.
#
# ── horus:cli — the default target ─────────────────────────────────────────
# The CLI and nothing else: 284 MB, no language toolchains. Inspect a system,
# validate a manifest, read logs, generate message types.
#
#   docker build -t horus:cli .
#   docker run --rm -v "$PWD:/work" -w /work horus:cli doctor
#   docker run --rm -v "$PWD:/work" -w /work horus:cli check
#
# It cannot build or run a project. `horus run` compiles the project, which
# needs cargo — the first version of this file advertised exactly that command
# against this image, and it fails:
#
#   horus hint [preflight] H060
#   Rust toolchain not installed.
#
# The error is correct and actionable, which is not the same as the example
# being correct. Use the dev image for that.
#
# ── horus:dev — build and run ──────────────────────────────────────────────
# Carries Rust, Python and CMake, so `horus new`, `horus build` and `horus run`
# all work. 3.98 GB against 284 MB, and that is the trade.
#
#   docker build --target dev -t horus:dev .
#   docker run --rm -it --shm-size=1g -v "$PWD:/work" -w /work horus:dev run
#
# Shared memory is how HORUS nodes talk, so a container that runs nodes needs a
# real /dev/shm — the default 64 MB is small for image or point-cloud topics,
# hence --shm-size above. For IPC across containers, share the namespace too:
#
#   docker run --rm --ipc=host --shm-size=1g ... horus:dev run
#
# ── Real-time scheduling needs a capability ────────────────────────────────
# Without it a container cannot set SCHED_FIFO, and HORUS degrades — correctly,
# and loudly, but it degrades:
#
#   [RT-thread] Could not set SCHED_FIFO: Permission denied: SCHED_FIFO
#   requires CAP_SYS_NICE or root (continuing with normal priority)
#
# So any container actually running real-time nodes wants:
#
#   docker run --rm --cap-add=SYS_NICE --shm-size=1g ... horus:dev run
#     -> [RT] Set RT priority 80
#
# Measuring real-time behaviour in a container without it measures the
# container, not HORUS.
#
# ── File ownership ─────────────────────────────────────────────────────────
# These run as root, so anything `horus new` or `horus build` writes into a
# bind-mounted directory is root-owned on the host. To keep ownership, pass your
# own uid and a writable CARGO_HOME (the image's belongs to root):
#
#   docker run --rm --user "$(id -u):$(id -g)" -e CARGO_HOME=/work/.cargo \
#     -v "$PWD:/work" -w /work horus:dev build

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

# ── Dev ────────────────────────────────────────────────────────────────────
# Everything `horus run` needs to compile a project in all three languages.
#
# Built from the builder stage so the Rust toolchain and the warm cargo registry
# are already there — a fresh `rustup` install here would download it twice.
FROM builder AS dev

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-venv \
        cmake \
        g++ \
        clang-format \
        clang-tidy \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /src/target/release/horus /usr/local/bin/horus

# The HORUS source tree stays at /src: `horus build` resolves horus_core and the
# C++ bindings from it, and without it a generated project cannot compile.
ENV HORUS_SOURCE=/src

RUN mkdir -p /usr/local/share/man/man1 \
    && horus man > /usr/local/share/man/man1/horus.1 \
    && mkdir -p /usr/share/bash-completion/completions \
    && horus completion bash > /usr/share/bash-completion/completions/horus

WORKDIR /work
ENTRYPOINT ["horus"]
CMD ["--help"]

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
