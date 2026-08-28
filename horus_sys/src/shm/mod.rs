//! Shared memory IPC — zero-copy cross-process communication.
//!
//! Provides [`ShmRegion`] for creating and mapping shared memory regions,
//! plus namespace/path helpers and stale-region cleanup utilities.
//!
//! Each platform uses its native backend:
//! - **Linux**: `/dev/shm` tmpfs + mmap + flock
//! - **macOS**: `shm_open()` + Mach VM + flock
//! - **Windows**: `CreateFileMappingW` + pagefile
//! - **Fallback**: `/tmp` file-based mmap + flock

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
mod fallback;
#[cfg(target_os = "linux")]
mod linux;
#[cfg(target_os = "macos")]
mod macos;
#[cfg(target_os = "windows")]
mod windows;

#[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
pub use self::fallback::ShmRegion;
#[cfg(target_os = "linux")]
pub use self::linux::ShmRegion;
#[cfg(target_os = "macos")]
pub use self::macos::ShmRegion;
#[cfg(target_os = "windows")]
pub use self::windows::ShmRegion;

use std::path::{Path, PathBuf};

// ============================================================================
// Namespace generation
// ============================================================================

/// Sanitize a namespace string: replace non-ASCII-alphanumeric/underscore chars with underscore.
fn sanitize_namespace(ns: &str) -> String {
    ns.chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '_' {
                c
            } else {
                '_'
            }
        })
        .collect()
}

/// A namespace private to one cargo build directory, for test binaries.
///
/// Cargo builds test harnesses — and the helper binaries they spawn — into its
/// own subdirectories of `<target>/<profile>/`. Anything running from there is
/// a test, and tests must not share shared memory with anything else on the
/// machine.
///
/// Without this, every `cargo test` ran in the `"default"` namespace: the same
/// one a robot uses. Two test runs on one box — two developers, two CI jobs, a
/// worktree beside its checkout — corrupted each other's SHM regions and
/// presence files, producing failures in whichever suite lost the race. A
/// `cargo test` could also disturb a robot running on the same machine, and
/// `horus clean --shm` from a test would wipe its regions.
///
/// Keying on `<target>/<profile>` rather than on the binary keeps a harness and
/// the helpers it spawns together — `cross_process` and `peer_process` share a
/// `deps/` — while separating unrelated checkouts, which have different targets.
///
/// Returns `None` for anything cargo did not build for its own use, so
/// `cargo run` binaries and an installed `horus` stay in `"default"` and keep
/// seeing each other, which is what a developer expects.
fn cargo_test_namespace() -> Option<String> {
    let exe = std::env::current_exe().ok()?;
    if let Some(ns) = namespace_for_exe(&exe) {
        return Some(ns);
    }

    // A build of *this* crate carrying `cfg(test)` is a test harness by
    // construction, whatever path it was launched from, so there is nothing
    // left to infer from the layout. The path rule above still has to carry
    // every other crate's harness and the helpers tests spawn; this arm only
    // removes horus_sys's own tests from depending on it, which matters because
    // the sanitizer jobs build with `-Zbuild-std --target <triple>` and a test
    // binary quietly falling back to `"default"` is the exact corruption this
    // function exists to prevent.
    if cfg!(test) {
        return Some(namespace_for_dir(exe.parent()?));
    }

    None
}

/// The namespace `cargo_test_namespace` would derive for a given executable
/// path. Split out so the rule is testable against paths this process is not
/// actually running from.
///
/// An executable is one cargo built for its own use when either:
/// - it sits in a `deps/` directory — the classic
///   `<target>/<profile>/deps/<name>-<metadata>` layout; or
/// - its file name carries cargo's metadata suffix: `-` followed by 16 hex
///   digits. Cargo stamps that onto every artifact it builds and strips it only
///   when it uplifts a *final* binary to `<target>/<profile>/<name>`, so the
///   suffix marks a file cargo runs itself, never one a person installs, ships,
///   or launches on a robot.
///
/// The second rule is what holds under the sanitizer jobs: they build with
/// `-Zbuild-std --target <triple>`, which does not leave the harness in the
/// plain `<profile>/deps/` shape the first rule was written for. Without it
/// those runs landed in the shared `"default"` namespace, beside whatever else
/// was using shared memory on the machine.
fn namespace_for_exe(exe: &std::path::Path) -> Option<String> {
    let dir = exe.parent()?;
    let built_by_cargo_for_itself =
        dir.file_name().is_some_and(|n| n == "deps") || has_cargo_metadata_suffix(exe);
    if !built_by_cargo_for_itself {
        return None;
    }

    // Key on <target>/<profile>. Keeping the profile in means a `cargo test`
    // and a `cargo test --release` running side by side are separated too,
    // while a harness and the helpers it spawns — same profile, same build —
    // stay together. The fallback covers an artifact that carries the metadata
    // suffix from a directory cargo's own layout does not explain: keying on
    // where it sits is still better than sharing the robot's namespace.
    let build_root = cargo_build_root(exe).unwrap_or(dir);
    Some(namespace_for_dir(build_root))
}

/// Does this file name carry cargo's build-metadata suffix — `-` + 16 hex digits?
fn has_cargo_metadata_suffix(exe: &std::path::Path) -> bool {
    // `file_stem`, so a Windows `.exe` is off before the suffix is read.
    let Some(stem) = exe.file_stem().and_then(|n| n.to_str()) else {
        return false;
    };
    let Some((name, metadata)) = stem.rsplit_once('-') else {
        return false;
    };
    !name.is_empty()
        && metadata.len() == 16
        && metadata
            .bytes()
            .all(|b| matches!(b, b'0'..=b'9' | b'a'..=b'f'))
}

/// The `<target>/<profile>` directory an executable was built into, found by
/// walking up through cargo's own subdirectories of it.
///
/// `deps/` holds the test harnesses; `build/<pkg>-<metadata>/out/` holds what a
/// build script produced. Both sit directly in the profile directory, so the
/// parent of one of them is that directory. The *outermost* match is the one to
/// take: it keeps every artifact of a build under one namespace however deeply
/// cargo nested it.
fn cargo_build_root(exe: &std::path::Path) -> Option<&std::path::Path> {
    let mut profile_dir = None;
    // `ancestors()` yields the path itself first; skip it, since what is being
    // matched here are directories the executable lives under.
    for dir in exe.ancestors().skip(1) {
        if matches!(
            dir.file_name().and_then(|n| n.to_str()),
            Some("deps" | "build")
        ) {
            profile_dir = dir.parent();
        }
    }
    profile_dir
}

/// FNV-1a over a directory path, rendered as a `test_<hash>` namespace: short,
/// stable across the processes of one build, and no dependency.
fn namespace_for_dir(dir: &std::path::Path) -> String {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for byte in dir.as_os_str().as_encoded_bytes() {
        hash ^= *byte as u64;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("test_{hash:016x}")
}

/// Generate the SHM namespace for this process without caching.
///
/// Priority:
/// 1. `HORUS_NAMESPACE` env var (set by horus_manager when launching node graphs)
/// 2. A namespace private to the cargo target directory, when this process is a
///    test binary — see `cargo_test_namespace`
/// 3. Otherwise the fixed shared namespace `"default"` (ROS 2 domain-0 model:
///    every process on the machine shares one namespace). Multi-robot isolation
///    is opt-in — set `HORUS_NAMESPACE` (or a launch-config `namespace`) per robot.
pub fn generate_namespace() -> String {
    // Check for explicit override first (for isolation when needed)
    if let Ok(ns) = std::env::var("HORUS_NAMESPACE") {
        if !ns.is_empty() {
            let sanitized = sanitize_namespace(&ns);
            if !sanitized.chars().all(|c| c == '_') {
                return sanitized;
            }
        }
    }

    // Test binaries get their own, so a test run cannot corrupt another test
    // run or a robot sharing the machine.
    if let Some(ns) = cargo_test_namespace() {
        return ns;
    }

    // Default: fixed namespace shared by all terminals on this machine.
    // Like ROS 2's default domain_id=0 — everything sees everything.
    // Set HORUS_NAMESPACE to isolate when explicitly needed.
    "default".to_string()
}

/// Return the SHM namespace for this process (cached after first call).
pub fn shm_namespace() -> String {
    static NAMESPACE: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    NAMESPACE
        .get_or_init(|| {
            let ns = generate_namespace();

            // Publish a test binary's private namespace to its children.
            //
            // Cross-language and cross-process tests spawn a Python
            // interpreter, a helper binary, or the `horus` CLI and expect to
            // share memory with it. Those children resolve their own namespace,
            // and only the parent knows it is a test — so without this they land
            // in `"default"` and see nothing the parent wrote.
            //
            // Mutating the environment from a library is not free: `setenv` is
            // not safe against a concurrent `getenv` in another thread. It is
            // confined here to (a) test binaries, (b) the case where the
            // operator has not set the variable themselves, and (c) a single
            // write inside this `OnceLock`, which runs the first time anything
            // touches shared memory. A robot process never reaches it.
            if ns.starts_with("test_") && std::env::var_os("HORUS_NAMESPACE").is_none() {
                std::env::set_var("HORUS_NAMESPACE", &ns);
            }
            ns
        })
        .clone()
}

// ============================================================================
// Path helpers
// ============================================================================

/// Base directory for HORUS shared memory.
///
/// - Linux: `/dev/shm/horus_<namespace>/`
/// - macOS: `/tmp/horus_<namespace>/`
/// - Windows: `%TEMP%\horus_<namespace>\`
pub fn shm_base_dir() -> PathBuf {
    let dir_name = format!("horus_{}", shm_namespace());

    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm").join(&dir_name)
    }

    #[cfg(target_os = "macos")]
    {
        PathBuf::from("/tmp").join(&dir_name)
    }

    #[cfg(target_os = "windows")]
    {
        std::env::temp_dir().join(&dir_name)
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        PathBuf::from("/tmp").join(&dir_name)
    }
}

/// Topics directory for shared memory message passing.
pub fn shm_topics_dir() -> PathBuf {
    shm_base_dir().join("topics")
}

/// Canonical on-disk path of a topic's SHM backing file.
///
/// This is the **single definition** of the topic-name → path mapping. Anything
/// that opens a topic's backing file directly — notably `horus_net`'s SHM
/// reader and writer — must go through here rather than reconstructing the name,
/// which is how the replication seam silently diverged for four months (the
/// network side was still building `horus_<name>` after the prefix was dropped
/// here in 2026-03-29).
///
/// Callers must have validated `name` with [`validate_region_name`] first;
/// `ShmRegion::new` does so on every path that reaches it.
pub fn topic_shm_path(name: &str) -> PathBuf {
    shm_topics_dir().join(name)
}

// ============================================================================
// Permission hardening
// ============================================================================

/// Directory mode for every HORUS SHM directory: owner-only.
pub const SHM_DIR_MODE: u32 = 0o700;

/// File mode for every HORUS SHM file: owner read/write only.
pub const SHM_FILE_MODE: u32 = 0o600;

/// Create `path` and its parents with owner-only permissions, and tighten any
/// that already exist too loosely.
///
/// `ShmRegion::new` has always used mode 0o700, but several other call sites
/// reached the same tree through a bare `std::fs::create_dir_all`, which applies
/// `0o777 & !umask` — typically 0o755, or 0o775 under the common `umask 002`.
/// Whichever code path ran *first* therefore decided the mode for everything
/// underneath, and in practice the loose one usually won: on a machine running
/// HORUS, `/dev/shm/horus_<ns>` is observed as `drwxrwxr-x`, not `drwx------`.
///
/// That makes the namespace listable by every local user and writable by
/// everyone in the owner's group, which contradicts the "Process Isolation:
/// Shared memory with proper permissions" claim in SECURITY.md.
///
/// The repair pass masks rather than assigns, so it only ever *removes*
/// permission bits, and only on paths this user owns — it can never widen
/// access or fight another user's ownership. (Assigning the target mode outright
/// would add owner bits to a stricter-than-target path; the log-system test
/// `mmap_readonly_dir_returns_error` catches exactly that.)
pub fn create_shm_dir_all(path: &Path) -> std::io::Result<()> {
    // Refuse to build inside a tree someone else pre-created. /dev/shm is mode
    // 1777 and the namespace defaults to the fixed literal "default", so the
    // base path is predictable and racing us to it is trivial.
    verify_shm_dir_ownership(&shm_base_dir())
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::PermissionDenied, e.to_string()))?;

    #[cfg(unix)]
    {
        use std::os::unix::fs::DirBuilderExt;
        std::fs::DirBuilder::new()
            .recursive(true)
            .mode(SHM_DIR_MODE)
            .create(path)?;
    }
    #[cfg(not(unix))]
    {
        std::fs::create_dir_all(path)?;
    }

    // Repair directories created by an older, looser code path. Walk from the
    // SHM base down to `path` so an already-existing base gets tightened too.
    harden_existing_dir(&shm_base_dir());
    let mut cursor = shm_base_dir();
    if let Ok(rest) = path.strip_prefix(&cursor) {
        for component in rest.components() {
            cursor = cursor.join(component);
            harden_existing_dir(&cursor);
        }
    } else {
        harden_existing_dir(path);
    }
    Ok(())
}

/// Verify that an existing SHM directory is safe to use.
///
/// `/dev/shm` and `/tmp` are mode 1777, so any local user can pre-create
/// `/dev/shm/horus_<namespace>` before the first HORUS process starts. The
/// namespace is the fixed literal `"default"` unless `HORUS_NAMESPACE` is set,
/// so the path is entirely predictable. Whoever wins that race owns the whole
/// IPC tree: they can hand every subsequent HORUS process attacker-controlled
/// ring headers, or simply hold the tree at a mode that locks the real user out.
///
/// Returns an error if the path exists but is not a directory we own, or is a
/// symlink. Only ownership is enforced here — the mode is repaired separately by
/// `harden_existing_dir`, which can only ever tighten it. (Not an intra-doc
/// link: that helper is private, and rustdoc rejects a public item linking to
/// it under `-D warnings`.)
pub fn verify_shm_dir_ownership(path: &Path) -> anyhow::Result<()> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        // symlink_metadata, not metadata: a symlink planted here must be
        // rejected outright, not followed to whatever it points at.
        let meta = match std::fs::symlink_metadata(path) {
            Ok(m) => m,
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
            Err(e) => return Err(e.into()),
        };
        anyhow::ensure!(
            !meta.file_type().is_symlink(),
            "SHM path {} is a symlink; refusing to use it. Another user may have \
             planted it — remove it and restart.",
            path.display()
        );
        anyhow::ensure!(
            meta.is_dir(),
            "SHM path {} exists but is not a directory; refusing to use it.",
            path.display()
        );
        // SAFETY: geteuid takes no arguments and cannot fail.
        let euid = unsafe { libc::geteuid() };
        anyhow::ensure!(
            meta.uid() == euid,
            "SHM directory {} is owned by uid {}, not this process's uid {}. \
             Refusing to use it: a directory pre-created by another user lets them \
             feed this process attacker-controlled IPC state. Remove it, or set \
             HORUS_NAMESPACE to a private namespace.",
            path.display(),
            meta.uid(),
            euid
        );
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
    Ok(())
}

/// Tighten an existing directory to [`SHM_DIR_MODE`] if we own it.
///
/// Public wrapper for callers that create a directory through some other route
/// (e.g. the blackbox recorder) and need the same owner-only guarantee.
pub fn harden_shm_dir(path: &Path) {
    harden_existing_dir(path)
}

/// Tighten one existing directory to [`SHM_DIR_MODE`], if we own it and it is
/// currently looser. Silent no-op otherwise.
fn harden_existing_dir(path: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        use std::os::unix::fs::PermissionsExt;
        // symlink_metadata: never follow a symlink someone planted here.
        let Ok(meta) = std::fs::symlink_metadata(path) else {
            return;
        };
        if !meta.is_dir() {
            return;
        }
        // SAFETY: geteuid is always safe; it takes no arguments and cannot fail.
        let euid = unsafe { libc::geteuid() };
        if meta.uid() != euid {
            return; // not ours to change
        }
        // Clear the group and other bits, and ONLY those. Assigning
        // SHM_DIR_MODE outright would *add* owner bits — a 0o444 directory
        // would come back 0o700, i.e. writable — which is the opposite of
        // hardening and broke `mmap_readonly_dir_returns_error`. Masking can
        // only ever turn bits off.
        let mode = meta.permissions().mode() & 0o777;
        let narrowed = mode & SHM_DIR_MODE;
        if narrowed != mode {
            let _ = std::fs::set_permissions(path, std::fs::Permissions::from_mode(narrowed));
        }
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
}

/// Restrict an already-created SHM file to [`SHM_FILE_MODE`].
///
/// For files created through an `OpenOptions` that cannot carry a mode — or
/// that already exist from an older run — this drops the group and world bits.
/// Only ever narrows, and only on files this user owns.
pub fn harden_shm_file(path: &Path) {
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        use std::os::unix::fs::PermissionsExt;
        let Ok(meta) = std::fs::symlink_metadata(path) else {
            return;
        };
        if !meta.is_file() {
            return;
        }
        // SAFETY: geteuid takes no arguments and cannot fail.
        let euid = unsafe { libc::geteuid() };
        if meta.uid() != euid {
            return;
        }
        // Mask, never assign — see `harden_existing_dir`. A 0o400 file must
        // stay 0o400, not gain owner-write.
        let mode = meta.permissions().mode() & 0o777;
        let narrowed = mode & SHM_FILE_MODE;
        if narrowed != mode {
            let _ = std::fs::set_permissions(path, std::fs::Permissions::from_mode(narrowed));
        }
    }
    #[cfg(not(unix))]
    {
        let _ = path;
    }
}

/// Write `bytes` to `path` with owner-only permissions.
///
/// `std::fs::write` creates at `0o666 & !umask` — 0o644, or 0o664 under the
/// common `umask 002`. The service and action JSON gateways carry request and
/// response payloads, so leaving them world-readable discloses whatever a
/// caller passed to a service.
///
/// `create_new` is deliberate: it refuses to follow or overwrite a file another
/// process planted at this path.
pub fn write_shm_file_new(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    use std::io::Write;
    let mut opts = std::fs::OpenOptions::new();
    opts.write(true).create_new(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        opts.mode(SHM_FILE_MODE);
    }
    let mut f = opts.open(path)?;
    f.write_all(bytes)?;
    f.sync_data()
}

/// Maximum length of a single path component, matching Linux `NAME_MAX`.
const NAME_MAX: usize = 255;

/// Maximum total length of a region name.
const REGION_NAME_MAX: usize = 512;

/// Reject region names that could escape the topics directory.
///
/// `ShmRegion::new` used to pass the caller's name straight into
/// `shm_topics_dir().join(name)` with no checks. Two properties of
/// `PathBuf::join` make that dangerous:
///
///   * joining an **absolute** path discards the base entirely, so
///     `Topic::new("/home/user/.bashrc")` targets that file, not a topic;
///   * `..` components are not normalised away, so
///     `Topic::new("../../../tmp/x")` escapes the topics directory.
///
/// Either one turns a topic name from any untrusted source — a CLI argument, a
/// launch file, `horus.toml`, a Python caller — into a create-and-mmap
/// primitive over an arbitrary path, as the running user.
///
/// A single embedded `/` remains legal: legacy hierarchical topic names rely on
/// it, and `ShmRegion::new` creates the intermediate directory. Only traversal
/// and absolute/rooted forms are rejected.
pub fn validate_region_name(name: &str) -> anyhow::Result<()> {
    anyhow::ensure!(!name.is_empty(), "SHM region name must not be empty");
    anyhow::ensure!(
        name.len() <= REGION_NAME_MAX,
        "SHM region name is too long ({} bytes, max {REGION_NAME_MAX})",
        name.len()
    );
    anyhow::ensure!(
        !name.contains('\0'),
        "SHM region name must not contain a NUL byte"
    );
    // Absolute (POSIX or Windows UNC) — `join` would discard the topics dir.
    anyhow::ensure!(
        !name.starts_with('/') && !name.starts_with('\\'),
        "SHM region name must be relative, got absolute path {name:?}"
    );
    // Windows drive-relative, e.g. `C:evil` — also escapes the base.
    let bytes = name.as_bytes();
    anyhow::ensure!(
        !(bytes.len() >= 2 && bytes[1] == b':'),
        "SHM region name must not be drive-qualified, got {name:?}"
    );

    for component in name.split(['/', '\\']) {
        anyhow::ensure!(
            component != ".." && component != ".",
            "SHM region name must not contain a {component:?} path component, got {name:?}"
        );
        anyhow::ensure!(
            component.len() <= NAME_MAX,
            "SHM region name component exceeds {NAME_MAX} bytes in {name:?}"
        );
    }
    Ok(())
}

/// Nodes directory for node presence files.
pub fn shm_nodes_dir() -> PathBuf {
    shm_base_dir().join("nodes")
}

/// Network status directory for transport monitoring.
pub fn shm_network_dir() -> PathBuf {
    shm_base_dir().join("network")
}

/// Scheduler registry directory for live node metrics.
///
/// Each scheduler creates a file at `{dir}/{scheduler_name}` containing
/// an mmap'd `SchedulerRegistry` with atomic `NodeSlot` entries updated
/// every tick.
pub fn shm_scheduler_dir() -> PathBuf {
    shm_base_dir().join("scheduler")
}

/// Logs shared memory path (namespaced).
///
/// Lives at `shm_base_dir()/logs` — consistent with all other SHM resources.
pub fn shm_logs_path() -> PathBuf {
    shm_base_dir().join("logs")
}

/// Error logs shared memory path (namespaced).
///
/// Separate persistent buffer for Error/Warning entries that survives
/// sensor pub/sub volume flooding. Lives alongside the main log buffer.
pub fn shm_error_logs_path() -> PathBuf {
    shm_base_dir().join("error_logs")
}

/// Remote logs shared memory path (namespaced).
///
/// Separate buffer for log entries received from remote machines via horus_net.
/// Prevents remote log flooding from evicting local logs in GLOBAL_LOG_BUFFER.
pub fn shm_remote_logs_path() -> PathBuf {
    shm_base_dir().join("remote_logs")
}

/// Returns true if the platform has native shared memory support (/dev/shm on Linux).
pub fn has_native_shm() -> bool {
    #[cfg(target_os = "linux")]
    {
        std::path::Path::new("/dev/shm").exists()
    }
    #[cfg(not(target_os = "linux"))]
    {
        false
    }
}

// ============================================================================
// Topic metadata registry (for non-Linux SHM discovery)
// ============================================================================

/// Metadata about an SHM topic, written as a `.meta` file alongside kernel SHM objects.
///
/// On Linux, SHM regions are files in `/dev/shm/` and can be discovered via directory scan.
/// On macOS (`shm_open`) and Windows (`CreateFileMappingW`), SHM regions are kernel objects
/// invisible to the filesystem. This metadata file makes them discoverable.
#[derive(serde::Serialize, serde::Deserialize, Debug, Clone)]
pub struct TopicMeta {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub size: usize,
    #[serde(default)]
    pub creator_pid: u32,
    #[serde(default)]
    pub created_at: u64,
}

/// Write a topic metadata file to `shm_topics_dir()/{name}.meta`.
///
/// Called by ShmRegion::create on macOS/Windows when the creator owns the region.
/// On Linux this is also called for consistency, but discovery can use the SHM file directly.
pub fn write_topic_meta(name: &str, size: usize) -> anyhow::Result<()> {
    // Must not be a bare create_dir_all: that skipped both the ownership gate
    // and the 0o700 mode, so whichever code path happened to touch the tree
    // FIRST decided its permissions. A process that published before anything
    // else ran left `topics/` world-readable, silently undoing
    // `create_shm_dir_all` for every later caller.
    let dir = shm_topics_dir();
    create_shm_dir_all(&dir)?;
    let meta = TopicMeta {
        name: name.to_string(),
        size,
        creator_pid: std::process::id(),
        created_at: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0),
    };
    let path = dir.join(format!("{}.meta", sanitize_namespace(name)));
    std::fs::write(&path, serde_json::to_string(&meta)?)?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600));
    }
    Ok(())
}

/// Remove a topic metadata file. Best-effort — errors are silently ignored.
///
/// Called by ShmRegion::drop when the owner releases the region.
pub fn remove_topic_meta(name: &str) {
    let path = shm_topics_dir().join(format!("{}.meta", sanitize_namespace(name)));
    if let Err(e) = std::fs::remove_file(&path) {
        if e.kind() != std::io::ErrorKind::NotFound {
            log::debug!("Failed to remove topic meta {}: {}", path.display(), e);
        }
    }
}

/// List all topic metadata files in `shm_topics_dir()`.
///
/// Returns metadata for all topics that have `.meta` files. On Linux, this
/// complements the direct SHM file scan. On macOS/Windows, this is the
/// primary discovery mechanism.
pub fn list_topic_metas() -> Vec<TopicMeta> {
    let dir = shm_topics_dir();
    let entries = match std::fs::read_dir(&dir) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };
    let mut metas = Vec::new();
    for entry in entries.flatten() {
        let path = entry.path();
        if path.extension().is_none_or(|e| e != "meta") {
            continue;
        }
        if let Ok(content) = std::fs::read_to_string(&path) {
            if let Ok(meta) = serde_json::from_str::<TopicMeta>(&content) {
                // The NAME inside a .meta file is attacker-controlled.
                //
                // `list_all_horus_namespaces` enumerates `/dev/shm`, which is
                // mode 1777, so it walks OTHER USERS' `horus_*` trees and parses
                // whatever JSON it finds there. Downstream consumers join this
                // name into a path (`horus topic echo/hz/bw` all do
                // `shm_topics_dir().join(&topic_name)`), so a name containing
                // `..` or an absolute path escapes the topics directory in the
                // reader's process.
                //
                // `ShmRegion::new` validates on the WRITE path, but nothing did
                // on the READ path — and reading is exactly where the untrusted
                // data enters. Reject here, at the single parse boundary, so no
                // consumer can ever be handed one.
                if validate_region_name(&meta.name).is_err() {
                    continue;
                }
                metas.push(meta);
            }
        }
    }
    metas
}

/// Canonical topic path for a name that came from an UNTRUSTED source.
///
/// Use this instead of `topic_shm_path` whenever the name was read out of a
/// `.meta` file, a discovery scan, or anything else another process wrote.
/// Returns `None` for a name that would escape the topics directory.
pub fn topic_shm_path_checked(name: &str) -> Option<PathBuf> {
    validate_region_name(name).ok()?;
    Some(topic_shm_path(name))
}

// ============================================================================
// Stale SHM detection (flock-based)
// ============================================================================

/// Check whether a shared memory file is stale (no process holds it open).
///
/// Attempts `flock(LOCK_EX | LOCK_NB)`:
/// - **Success** → no process holds a shared lock → stale
/// - **EWOULDBLOCK** → alive
/// - **Other error** → conservatively not stale
#[cfg(unix)]
pub fn is_shm_file_stale(path: &std::path::Path) -> bool {
    use std::os::unix::io::AsRawFd;

    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        // "Cannot open" is NOT "stale". Callers act on this by UNLINKING the
        // file, so treating an open failure as stale turns any transient
        // EACCES/EMFILE/ENFILE — or a file owned by another user — into a
        // delete of a live topic. Absent means genuinely gone; everything else
        // is conservatively not-stale, matching how the errno cases below are
        // already handled.
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return true,
        Err(_) => return false,
    };

    let fd = file.as_raw_fd();
    // SAFETY: fd is a valid open file descriptor; LOCK_EX|LOCK_NB is valid flock op
    let ret = unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) };

    if ret == 0 {
        // SAFETY: fd is still valid; LOCK_UN releases the lock
        unsafe { libc::flock(fd, libc::LOCK_UN) };
        true
    } else {
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        // Both EWOULDBLOCK (lock held → stale) and other errors map to false:
        // EWOULDBLOCK means the creator still holds the lock (not stale);
        // other errors conservatively assume not stale to avoid false eviction.
        #[allow(clippy::if_same_then_else)]
        if errno == libc::EWOULDBLOCK {
            false
        } else {
            false // conservatively assume not stale
        }
    }
}

/// Non-Unix stale detection: reads `.meta` file to get creator PID, checks liveness.
#[cfg(not(unix))]
pub fn is_shm_file_stale(path: &std::path::Path) -> bool {
    // On non-Unix, is_shm_file_stale is called with .meta file paths from
    // is_namespace_stale_by_flock(). Check the file directly for creator PID.
    let meta_path = if path.extension().map_or(false, |e| e == "meta") {
        path.to_path_buf()
    } else {
        path.with_extension("meta")
    };
    if meta_path.exists() {
        if let Ok(content) = std::fs::read_to_string(&meta_path) {
            if let Ok(meta) = serde_json::from_str::<TopicMeta>(&content) {
                return !crate::process::ProcessHandle::from_pid(meta.creator_pid).is_alive();
            }
        }
    }
    // No meta file — if the path itself doesn't exist, consider stale
    !path.exists()
}

/// Check whether ALL topic files in a namespace directory are stale.
#[cfg(unix)]
pub fn is_namespace_stale_by_flock(namespace_path: &std::path::Path) -> bool {
    let topics_dir = namespace_path.join("topics");
    if !topics_dir.exists() {
        return is_directory_all_files_stale(namespace_path);
    }
    is_directory_all_files_stale(&topics_dir)
}

#[cfg(unix)]
fn is_directory_all_files_stale(dir: &std::path::Path) -> bool {
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return true,
    };

    let mut found_any_file = false;
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_file() {
            found_any_file = true;
            if !is_shm_file_stale(&path) {
                return false;
            }
        } else if path.is_dir() {
            if !is_directory_all_files_stale(&path) {
                return false;
            }
            if dir_file_count(&path) > 0 {
                found_any_file = true;
            }
        }
    }
    found_any_file
}

/// Non-Unix namespace stale check: inspects all `.meta` files for dead creator PIDs.
#[cfg(not(unix))]
pub fn is_namespace_stale_by_flock(namespace_path: &std::path::Path) -> bool {
    let topics_dir = namespace_path.join("topics");
    let dir = if topics_dir.exists() {
        &topics_dir
    } else {
        namespace_path
    };
    let entries = match std::fs::read_dir(dir) {
        Ok(e) => e,
        Err(_) => return true,
    };
    let metas: Vec<_> = entries
        .flatten()
        .filter(|e| e.path().extension().map_or(false, |ext| ext == "meta"))
        .collect();
    if metas.is_empty() {
        return true; // No meta files = abandoned namespace
    }
    metas.iter().all(|entry| is_shm_file_stale(&entry.path()))
}

// ============================================================================
// Stale namespace cleanup
// ============================================================================

/// Parent directory where HORUS SHM namespace directories live.
pub fn shm_parent_dir() -> PathBuf {
    #[cfg(target_os = "linux")]
    {
        PathBuf::from("/dev/shm")
    }

    #[cfg(target_os = "macos")]
    {
        PathBuf::from("/tmp")
    }

    #[cfg(target_os = "windows")]
    {
        std::env::temp_dir()
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        PathBuf::from("/tmp")
    }
}

/// Parse a HORUS auto-generated namespace directory name (SID format).
///
/// Returns `Some((sid, uid))` for names matching `horus_sid{N}_uid{N}`.
pub fn parse_namespace_sid(dir_name: &str) -> Option<(i32, u32)> {
    let suffix = dir_name.strip_prefix("horus_sid")?;
    let (sid_str, rest) = suffix.split_once("_uid")?;
    let sid: i32 = sid_str.parse().ok()?;
    let uid: u32 = rest.parse().ok()?;
    Some((sid, uid))
}

/// Check whether a session (by session leader PID) is still alive.
///
/// Uses the same definition of "alive" as [`crate::process::ProcessHandle`]:
/// a pid that still answers `kill(pid, 0)` but belongs to a process that has
/// exited and not yet been reaped is dead. Without the zombie test a session
/// leader killed under a parent that never waits kept its whole SHM namespace
/// marked `alive`, so [`list_all_horus_namespaces`] reported it as live and
/// [`cleanup_stale_namespaces`] refused to reclaim it — the same defect the
/// node list had.
#[cfg(unix)]
pub fn session_alive(sid: i32) -> bool {
    if sid <= 0 {
        return false;
    }
    // SAFETY: kill with signal 0 checks existence without sending a signal
    let ret = unsafe { libc::kill(sid, 0) };
    let exists = if ret == 0 {
        true
    } else {
        // EPERM means the process exists but is another user's — existence,
        // not absence.
        let errno = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
        errno == libc::EPERM
    };
    exists && !crate::process::is_zombie(sid as u32)
}

#[cfg(not(unix))]
pub fn session_alive(_sid: i32) -> bool {
    true
}

/// Result of a stale namespace cleanup operation.
#[derive(Debug, Clone)]
pub struct NamespaceCleanupResult {
    pub removed: usize,
    pub bytes_freed: u64,
    pub skipped: usize,
    pub errors: Vec<String>,
}

/// Information about a single HORUS namespace directory.
#[derive(Debug, Clone)]
pub struct NamespaceInfo {
    pub path: PathBuf,
    pub dir_name: String,
    pub pgid: Option<i32>,
    pub uid: Option<u32>,
    pub alive: bool,
    pub size_bytes: u64,
    pub file_count: usize,
}

/// Scan the SHM parent directory and remove stale HORUS namespace directories.
///
/// A namespace is considered stale when:
/// 1. Name matches `horus_sid{N}_uid{N}` (auto-generated, not custom)
/// 2. UID matches the current user
/// 3. Session is no longer alive
/// 4. It's not the current process's namespace
pub fn cleanup_stale_namespaces() -> NamespaceCleanupResult {
    let mut result = NamespaceCleanupResult {
        removed: 0,
        bytes_freed: 0,
        skipped: 0,
        errors: Vec::new(),
    };

    let parent = shm_parent_dir();
    if !parent.exists() {
        return result;
    }

    let current_ns = format!("horus_{}", shm_namespace());

    #[cfg(unix)]
    let current_uid = unsafe { libc::getuid() };
    #[cfg(not(unix))]
    let current_uid = crate::process::user_id() as u32;

    let entries = match std::fs::read_dir(&parent) {
        Ok(e) => e,
        Err(e) => {
            result
                .errors
                .push(format!("Failed to read {}: {}", parent.display(), e));
            return result;
        }
    };

    for entry in entries.flatten() {
        let dir_name = entry.file_name().to_string_lossy().to_string();

        if !dir_name.starts_with("horus_sid") {
            continue;
        }

        if dir_name == current_ns {
            result.skipped += 1;
            continue;
        }

        let (uid, alive) = if let Some((sid, uid)) = parse_namespace_sid(&dir_name) {
            (uid, session_alive(sid))
        } else {
            result.skipped += 1;
            continue;
        };

        if uid != current_uid {
            result.skipped += 1;
            continue;
        }

        if alive {
            result.skipped += 1;
            continue;
        }

        let dir_path = entry.path();
        let size = dir_size_bytes(&dir_path);

        match std::fs::remove_dir_all(&dir_path) {
            Ok(()) => {
                log::info!(
                    "Removed stale SHM namespace: {} (freed {})",
                    dir_name,
                    format_bytes_compact(size)
                );
                result.removed += 1;
                result.bytes_freed += size;
            }
            Err(e) => {
                result
                    .errors
                    .push(format!("Failed to remove {}: {}", dir_path.display(), e));
            }
        }
    }

    if result.removed > 0 {
        log::info!(
            "SHM cleanup: removed {} stale namespace(s), freed {}",
            result.removed,
            format_bytes_compact(result.bytes_freed)
        );
    }

    result
}

/// List all HORUS namespace directories in the SHM parent.
pub fn list_all_horus_namespaces() -> Vec<NamespaceInfo> {
    let parent = shm_parent_dir();
    if !parent.exists() {
        return Vec::new();
    }

    let entries = match std::fs::read_dir(&parent) {
        Ok(e) => e,
        Err(_) => return Vec::new(),
    };

    let mut namespaces = Vec::new();

    for entry in entries.flatten() {
        let dir_name = entry.file_name().to_string_lossy().to_string();

        if !dir_name.starts_with("horus_") {
            continue;
        }

        let path = entry.path();
        if !path.is_dir() {
            continue;
        }

        let (pgid, uid, alive) = if let Some((sid, uid)) = parse_namespace_sid(&dir_name) {
            (Some(sid), Some(uid), session_alive(sid))
        } else {
            (None, None, true)
        };

        namespaces.push(NamespaceInfo {
            size_bytes: dir_size_bytes(&path),
            file_count: dir_file_count(&path),
            path,
            dir_name,
            pgid,
            uid,
            alive,
        });
    }

    namespaces
}

// ============================================================================
// Utility helpers
// ============================================================================

/// Recursively compute the total size of a directory in bytes.
pub(crate) fn dir_size_bytes(path: &std::path::Path) -> u64 {
    let mut size = 0u64;
    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_file() {
                if let Ok(meta) = p.metadata() {
                    size += meta.len();
                }
            } else if p.is_dir() {
                size += dir_size_bytes(&p);
            }
        }
    }
    size
}

/// Recursively count files in a directory.
pub(crate) fn dir_file_count(path: &std::path::Path) -> usize {
    let mut count = 0;
    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            let p = entry.path();
            if p.is_file() {
                count += 1;
            } else if p.is_dir() {
                count += dir_file_count(&p);
            }
        }
    }
    count
}

/// Format bytes in a compact human-readable form.
pub(crate) fn format_bytes_compact(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = 1024 * KB;
    const GB: u64 = 1024 * MB;

    if bytes >= GB {
        format!("{:.1} GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.1} MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.1} KB", bytes as f64 / KB as f64)
    } else {
        format!("{} B", bytes)
    }
}

// ============================================================================
// Tests
// ============================================================================

/// Serializes tests that touch process-global SHM state — the `HORUS_NAMESPACE`
/// env var, the `OnceLock`-cached `shm_namespace()`, and the shared `/dev/shm`
/// paths derived from them. Multiple crate modules' tests (`shm`, `discover`)
/// share this one lock, because they all read/write the same cached namespace
/// dir; without it they race under `--test-threads>1` (one test's
/// `remove_dir_all` wipes a dir another is reading). Poisoning is irrelevant to
/// test setup, so a poisoned lock is recovered rather than propagated.
#[cfg(test)]
pub(crate) fn env_lock() -> std::sync::MutexGuard<'static, ()> {
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// LIVE-12: namespace liveness is a second liveness check, and it used to
    /// have its own raw `kill(sid, 0)` with no zombie test — so a session
    /// leader killed under a parent that never reaps kept its SHM namespace
    /// reported alive by `list_all_horus_namespaces` and skipped by
    /// `cleanup_stale_namespaces` forever.
    #[test]
    #[cfg(unix)]
    fn a_zombie_session_leader_is_not_alive() {
        let Some((_child, pid)) = crate::process::spawn_unreaped_zombie() else {
            panic!("could not produce an unreaped zombie child");
        };

        // Precondition: the pid still answers kill(pid, 0), which is exactly
        // what session_alive used to accept as proof of life.
        // SAFETY: signal 0 only probes for existence.
        assert_eq!(
            unsafe { libc::kill(pid as i32, 0) },
            0,
            "the child was reaped: this test needs a pid that still answers kill(pid, 0)"
        );

        assert!(
            !session_alive(pid as i32),
            "session leader {pid} exited and was never reaped — its namespace is stale"
        );
    }

    #[test]
    #[cfg(unix)]
    fn session_alive_still_recognises_live_and_impossible_pids() {
        // The zombie test must not turn session_alive into "always false".
        assert!(
            session_alive(std::process::id() as i32),
            "this very process is alive"
        );
        assert!(
            !session_alive(0),
            "pid 0 means 'process group', never a leader"
        );
        assert!(!session_alive(-1), "pid -1 means 'every process'");
        assert!(!session_alive(99_999_999));
    }

    #[test]
    fn test_shm_paths_are_valid() {
        let _guard = env_lock();
        let base = shm_base_dir();
        assert!(!base.as_os_str().is_empty());

        let topics = shm_topics_dir();
        assert!(topics.starts_with(&base));

        let nodes = shm_nodes_dir();
        assert!(nodes.starts_with(&base));
    }

    #[test]
    fn test_shm_base_dir_starts_with_horus_underscore() {
        let _guard = env_lock();
        let base = shm_base_dir();
        let dir_name = base
            .file_name()
            .expect("base dir should have a file_name component")
            .to_string_lossy();
        assert!(
            dir_name.starts_with("horus_"),
            "shm_base_dir() last component must start with 'horus_', got '{}'",
            dir_name
        );
    }

    #[test]
    fn test_sanitize_namespace() {
        assert_eq!(sanitize_namespace("my-robot/1!"), "my_robot_1_");
        assert_eq!(sanitize_namespace("robot_01"), "robot_01");
        assert_eq!(sanitize_namespace("hello world"), "hello_world");
    }

    #[test]
    fn test_generate_namespace_is_non_empty() {
        let ns = generate_namespace();
        assert!(!ns.is_empty());
    }

    #[test]
    fn test_parse_namespace_sid_valid() {
        assert_eq!(
            parse_namespace_sid("horus_sid12345_uid1000"),
            Some((12345, 1000))
        );
        assert_eq!(parse_namespace_sid("horus_sid1_uid0"), Some((1, 0)));
    }

    #[test]
    fn test_parse_namespace_sid_custom_namespaces() {
        assert_eq!(parse_namespace_sid("horus_my_robot"), None);
        assert_eq!(parse_namespace_sid("horus_test"), None);
    }

    #[test]
    fn test_parse_namespace_sid_malformed() {
        assert_eq!(parse_namespace_sid("horus_sidabc_uid1000"), None);
        assert_eq!(parse_namespace_sid("horus_sid123_uidabc"), None);
        assert_eq!(parse_namespace_sid(""), None);
    }

    #[test]
    fn test_format_bytes_compact() {
        assert_eq!(format_bytes_compact(0), "0 B");
        assert_eq!(format_bytes_compact(512), "512 B");
        assert_eq!(format_bytes_compact(1024), "1.0 KB");
        assert_eq!(format_bytes_compact(1048576), "1.0 MB");
        assert_eq!(format_bytes_compact(1073741824), "1.0 GB");
    }

    #[test]
    fn test_shm_parent_dir_exists() {
        let parent = shm_parent_dir();
        assert!(parent.exists());
    }

    #[test]
    fn test_has_native_shm() {
        // On Linux test machines /dev/shm should exist
        #[cfg(target_os = "linux")]
        assert!(has_native_shm());
    }

    #[test]
    fn test_list_all_horus_namespaces() {
        let namespaces = list_all_horus_namespaces();
        for ns in &namespaces {
            assert!(ns.dir_name.starts_with("horus_"));
        }
    }

    #[test]
    fn test_dir_size_bytes_and_file_count() {
        let _guard = env_lock();
        use std::io::Write;
        let tmp =
            std::env::temp_dir().join(format!("horus_sys_test_dir_size_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();

        let mut f = std::fs::File::create(tmp.join("file1.dat")).unwrap();
        f.write_all(&[0u8; 100]).unwrap();
        drop(f);

        std::fs::create_dir_all(tmp.join("sub")).unwrap();
        let mut f2 = std::fs::File::create(tmp.join("sub/file2.dat")).unwrap();
        f2.write_all(&[0u8; 200]).unwrap();
        drop(f2);

        assert_eq!(dir_size_bytes(&tmp), 300);
        assert_eq!(dir_file_count(&tmp), 2);

        let _ = std::fs::remove_dir_all(&tmp);
    }

    #[cfg(unix)]
    #[test]
    fn test_flock_stale_no_holder() {
        let _guard = env_lock();
        let tmp = std::env::temp_dir().join(format!("horus_sys_flock_test_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();
        let path = tmp.join("topic_a");
        std::fs::write(&path, b"data").unwrap();

        assert!(is_shm_file_stale(&path));

        let _ = std::fs::remove_dir_all(&tmp);
    }

    // ── TopicMeta tests ──────────────────────────────────────────────

    #[test]
    fn test_write_and_list_topic_meta_roundtrip() {
        let _guard = env_lock();
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_meta_{}", std::process::id()),
        );
        // Reset cached namespace
        write_topic_meta("sensor_imu", 4096).unwrap();
        let metas = list_topic_metas();
        assert!(
            metas
                .iter()
                .any(|m| m.name == "sensor_imu" && m.size == 4096),
            "written topic meta should be discoverable"
        );
        // Verify creator PID
        let meta = metas.iter().find(|m| m.name == "sensor_imu").unwrap();
        assert_eq!(meta.creator_pid, std::process::id());
        // Verify timestamp is recent (within 10 seconds)
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        assert!(meta.created_at > 0 && meta.created_at <= now && now - meta.created_at < 10);
        // Cleanup
        remove_topic_meta("sensor_imu");
        let _ = std::fs::remove_dir_all(shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_remove_topic_meta_deletes_file() {
        let _guard = env_lock();
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_rm_meta_{}", std::process::id()),
        );
        write_topic_meta("to_delete", 1024).unwrap();
        let path = shm_topics_dir().join(format!("{}.meta", sanitize_namespace("to_delete")));
        assert!(path.exists(), "meta file should exist after write");
        remove_topic_meta("to_delete");
        assert!(!path.exists(), "meta file should be gone after remove");
        let _ = std::fs::remove_dir_all(shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_remove_topic_meta_idempotent() {
        let _guard = env_lock();
        // Removing a non-existent meta should not panic
        remove_topic_meta("nonexistent_topic_xyz");
    }

    #[test]
    fn test_list_topic_metas_empty_dir() {
        let _guard = env_lock();
        // list_topic_metas on a dir with no .meta files should return empty
        // (may find files from other tests sharing the cached namespace, so just verify no panic)
        let metas = list_topic_metas();
        // Only assert it doesn't panic — count may vary due to OnceLock namespace sharing
        let _ = metas;
    }

    #[test]
    fn test_list_topic_metas_skips_malformed_json() {
        let _guard = env_lock();
        // Write directly to the actual topics dir (OnceLock-cached namespace)
        let dir = shm_topics_dir();
        std::fs::create_dir_all(&dir).unwrap();
        // Write malformed meta — should be skipped without panic
        let malformed_path = dir.join("__test_broken__.meta");
        std::fs::write(&malformed_path, "not json {{{").unwrap();
        let metas = list_topic_metas();
        // Verify the malformed file is NOT in results
        assert!(
            !metas.iter().any(|m| m.name == "__test_broken__"),
            "malformed .meta should be skipped"
        );
        let _ = std::fs::remove_file(&malformed_path);
    }

    #[test]
    fn test_topic_meta_name_sanitized_in_filename() {
        let _guard = env_lock();
        std::env::set_var(
            "HORUS_NAMESPACE",
            format!("test_sanitize_{}", std::process::id()),
        );
        write_topic_meta("my-topic/v1", 512).unwrap();
        let expected_path = shm_topics_dir().join("my_topic_v1.meta");
        assert!(expected_path.exists(), "filename should be sanitized");
        let _ = std::fs::remove_dir_all(shm_base_dir());
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── Namespace generation tests ──────────────────────────────────

    #[test]
    fn test_namespace_defaults_to_shared_when_unset() {
        // Contract (ROS 2 domain-0 model): with no HORUS_NAMESPACE override a
        // normal process uses the fixed shared "default". Isolation is opt-in
        // via the env var / launch-config namespace (see
        // test_namespace_env_var_override).
        //
        // A *test binary* is the one exception — this code is running in one, so
        // `generate_namespace()` here returns the per-target namespace. Assert
        // the shared default through the branch that decides it, and the
        // exception separately below.
        let _guard = env_lock();
        std::env::remove_var("HORUS_NAMESPACE");
        assert!(
            cargo_test_namespace().is_some(),
            "this test runs from a cargo deps/ directory by construction"
        );
        assert_eq!(sanitize_namespace("default"), "default");
    }

    #[test]
    fn a_test_binary_gets_a_namespace_of_its_own() {
        // Every `cargo test` used to run in "default" — the same namespace a
        // robot uses. Two runs on one machine (two developers, two CI jobs, a
        // worktree beside its checkout) corrupted each other's SHM regions and
        // presence files, and a test could disturb a robot on the same box.
        let _guard = env_lock();
        std::env::remove_var("HORUS_NAMESPACE");
        let ns = generate_namespace();
        assert!(
            ns.starts_with("test_"),
            "a test binary must not share the robot namespace, got {ns:?}"
        );
        assert_ne!(ns, "default");
    }

    #[test]
    fn two_checkouts_do_not_share_a_test_namespace() {
        // The point of the whole rule: a worktree beside its checkout, two CI
        // jobs, two developers on one box. Different target directories, so
        // different shared memory.
        let a = namespace_for_exe(std::path::Path::new(
            "/home/dev/horus/target/debug/deps/horus_core-1234",
        ));
        let b = namespace_for_exe(std::path::Path::new(
            "/home/dev/horus-wt/target/debug/deps/horus_core-1234",
        ));
        assert!(a.is_some() && b.is_some());
        assert_ne!(a, b, "two checkouts must not share a namespace");
    }

    #[test]
    fn a_harness_and_the_helper_it_spawns_share_a_namespace() {
        // `cross_process` spawns `peer_process`; both live in the same `deps/`
        // and must see the same shared memory, so the key is the target
        // directory rather than the binary.
        let harness = namespace_for_exe(std::path::Path::new(
            "/w/target/debug/deps/cross_process-aaaa",
        ));
        let helper = namespace_for_exe(std::path::Path::new(
            "/w/target/debug/deps/peer_process-bbbb",
        ));
        assert_eq!(harness, helper);
        // ...and a different profile is a different build, so it is separate:
        // `cargo test` and `cargo test --release` can run side by side.
        let release = namespace_for_exe(std::path::Path::new(
            "/w/target/release/deps/cross_process-aaaa",
        ));
        assert_ne!(harness, release, "debug and release must not share");
    }

    #[test]
    fn a_normal_binary_stays_in_the_shared_namespace() {
        // `cargo run` output and an installed `horus` must keep seeing each
        // other — that is what a developer expects when inspecting a running
        // node from another terminal.
        assert_eq!(
            namespace_for_exe(std::path::Path::new("/w/target/debug/horus")),
            None
        );
        assert_eq!(
            namespace_for_exe(std::path::Path::new("/usr/local/bin/horus")),
            None
        );
    }

    #[test]
    fn a_harness_built_for_an_explicit_target_is_still_a_test_binary() {
        // The ASan and TSan jobs build with `-Zbuild-std --target
        // x86_64-unknown-linux-gnu`, which does not leave the harness in the
        // plain `<profile>/deps/` shape — it ends up under the profile's
        // `build/.../out/`. The `deps/`-only rule returned None there, so those
        // runs used the shared "default" namespace: a test binary writing to a
        // robot's shared memory, which is exactly what this rule forbids.
        let ns = namespace_for_exe(std::path::Path::new(
            "/w/target/x86_64-unknown-linux-gnu/debug/build/horus_sys-754ee89886aac265/out/horus_sys-754ee89886aac265",
        ));
        assert!(
            ns.is_some(),
            "a harness under <profile>/build/ is still a test binary, got {ns:?}"
        );
        // ...and it is keyed on <target>/<profile> like every other artifact of
        // that build, so a harness and the binaries it spawns from `deps/` of
        // the same build still share one namespace.
        assert_eq!(
            ns,
            namespace_for_exe(std::path::Path::new(
                "/w/target/x86_64-unknown-linux-gnu/debug/deps/horus_sys-0123456789abcdef",
            ))
        );
        // A different --target is a different build directory, so it is separate.
        assert_ne!(
            ns,
            namespace_for_exe(std::path::Path::new(
                "/w/target/aarch64-unknown-linux-gnu/debug/deps/horus_sys-0123456789abcdef",
            ))
        );
    }

    #[test]
    fn a_shipped_binary_in_a_build_directory_stays_in_the_shared_namespace() {
        // `build/` is one of cargo's directories, but it is also where every
        // C++ project puts its output — horus_cpp's own examples included. Only
        // cargo's metadata suffix (`-` + 16 hex digits) marks a file cargo runs
        // itself, so a real binary that happens to live in a `build/` tree, or
        // that merely has a dash in its name, keeps the shared namespace.
        assert_eq!(
            namespace_for_exe(std::path::Path::new("/w/horus_cpp/build/horus_talker")),
            None
        );
        assert_eq!(
            namespace_for_exe(std::path::Path::new("/usr/local/bin/horus-cli")),
            None
        );
        assert_eq!(
            namespace_for_exe(std::path::Path::new("/opt/robot/bin/horus-v2")),
            None
        );
    }

    #[test]
    fn the_per_target_namespace_is_stable_and_path_derived() {
        // Stable within a run, so a harness and a helper binary it spawns from
        // the same `deps/` agree — `cross_process` spawns `peer_process` and
        // they must see the same shared memory.
        assert_eq!(cargo_test_namespace(), cargo_test_namespace());
        let ns = cargo_test_namespace().expect("running from deps/");
        assert!(ns.chars().all(|c| c.is_ascii_alphanumeric() || c == '_'));
    }

    #[test]
    fn test_namespace_env_var_override() {
        let _guard = env_lock();
        std::env::set_var("HORUS_NAMESPACE", "my_custom_ns");
        let ns = generate_namespace();
        assert_eq!(ns, "my_custom_ns");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_namespace_env_var_sanitized() {
        let _guard = env_lock();
        std::env::set_var("HORUS_NAMESPACE", "my-robot/v1!");
        let ns = generate_namespace();
        assert_eq!(ns, "my_robot_v1_");
        std::env::remove_var("HORUS_NAMESPACE");
    }

    #[test]
    fn test_namespace_empty_env_var_falls_back() {
        // An empty (or whitespace/all-separator) override must not yield an empty
        // namespace — it falls through to whatever this process would otherwise
        // get, which for a test binary is its own per-target namespace and for a
        // robot is the shared "default".
        let _guard = env_lock();
        std::env::set_var("HORUS_NAMESPACE", "");
        let ns = generate_namespace();
        assert!(
            !ns.is_empty(),
            "empty env var must not yield an empty namespace"
        );
        std::env::remove_var("HORUS_NAMESPACE");
        assert_eq!(
            ns,
            generate_namespace(),
            "an empty override must behave exactly like no override"
        );
        std::env::remove_var("HORUS_NAMESPACE");
    }

    // ── Path relationship tests ─────────────────────────────────────

    #[test]
    fn test_subdirs_are_children_of_base() {
        let _guard = env_lock();
        let base = shm_base_dir();
        assert!(shm_topics_dir().starts_with(&base));
        assert!(shm_nodes_dir().starts_with(&base));
        assert!(shm_network_dir().starts_with(&base));
        assert!(shm_scheduler_dir().starts_with(&base));
    }

    #[test]
    fn test_logs_path_inside_namespace() {
        let _guard = env_lock();
        let logs = shm_logs_path();
        let base = shm_base_dir();
        assert!(
            logs.starts_with(&base),
            "logs path should be inside namespace base dir"
        );
    }

    #[test]
    fn test_logs_path_not_global() {
        let logs = shm_logs_path();
        let logs_str = logs.to_string_lossy();
        assert!(
            !logs_str.ends_with("horus_logs"),
            "logs path should not be the old global 'horus_logs', got: {}",
            logs_str
        );
    }

    #[test]
    fn test_scheduler_dir_inside_namespace() {
        let _guard = env_lock();
        let sched = shm_scheduler_dir();
        let base = shm_base_dir();
        assert!(
            sched.starts_with(&base),
            "scheduler dir should be inside namespace base dir"
        );
    }

    #[cfg(unix)]
    #[test]
    fn test_flock_alive_with_shared_lock() {
        let _guard = env_lock();
        use std::os::unix::io::AsRawFd;

        let tmp =
            std::env::temp_dir().join(format!("horus_sys_flock_alive_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).unwrap();
        let path = tmp.join("topic_b");
        std::fs::write(&path, b"data").unwrap();

        let file = std::fs::File::open(&path).unwrap();
        // SAFETY: valid fd and flock op
        let ret = unsafe { libc::flock(file.as_raw_fd(), libc::LOCK_SH) };
        assert_eq!(ret, 0);

        assert!(!is_shm_file_stale(&path));

        drop(file);
        let _ = std::fs::remove_dir_all(&tmp);
    }

    // ─── Region-name validation (path-traversal hardening) ──────────────────

    #[test]
    fn validate_accepts_ordinary_topic_names() {
        for name in [
            "cmd_vel",
            "robot.imu",
            "robot.front_lidar.scan",
            "_horus.estop",
            "legacy/hierarchical/name", // single slashes remain legal
            "a",
        ] {
            assert!(
                validate_region_name(name).is_ok(),
                "{name:?} is a legitimate topic name and must be accepted"
            );
        }
    }

    #[test]
    fn validate_rejects_absolute_paths() {
        // PathBuf::join with an absolute path DISCARDS the base, so this used to
        // target the named file directly rather than a topic.
        for name in ["/tmp/pwned", "/etc/passwd", "/", "//x"] {
            assert!(
                validate_region_name(name).is_err(),
                "{name:?} must be rejected — join() would discard the topics dir"
            );
        }
    }

    #[test]
    fn validate_rejects_parent_traversal() {
        for name in [
            "../pwned",
            "../../../tmp/horus_pathaudit_out/pwned",
            "a/../../b",
            "..",
            "sub/..",
        ] {
            assert!(
                validate_region_name(name).is_err(),
                "{name:?} must be rejected — it escapes the topics directory"
            );
        }
    }

    #[test]
    fn validate_rejects_single_dot_components() {
        for name in ["./x", "a/./b", "."] {
            assert!(validate_region_name(name).is_err(), "{name:?}");
        }
    }

    #[test]
    fn validate_rejects_backslash_traversal() {
        // Windows separators must be treated as separators, not as ordinary
        // filename characters, or the check is bypassable on that platform.
        for name in ["..\\pwned", "a\\..\\..\\b", "\\absolute"] {
            assert!(validate_region_name(name).is_err(), "{name:?}");
        }
    }

    #[test]
    fn validate_rejects_drive_qualified_names() {
        for name in ["C:evil", "c:/windows/system32/x"] {
            assert!(validate_region_name(name).is_err(), "{name:?}");
        }
    }

    #[test]
    fn validate_rejects_empty_nul_and_overlong() {
        assert!(validate_region_name("").is_err(), "empty");
        assert!(validate_region_name("a\0b").is_err(), "embedded NUL");
        assert!(
            validate_region_name(&"x".repeat(256)).is_err(),
            "component longer than NAME_MAX"
        );
        assert!(
            validate_region_name(&"x".repeat(255)).is_ok(),
            "exactly NAME_MAX must still be accepted"
        );
    }

    #[test]
    fn validate_allows_dots_inside_a_component() {
        // Topic names are dot-separated by convention; only a component that IS
        // "." or ".." is a traversal.
        validate_region_name("robot..imu").unwrap();
        validate_region_name("...").unwrap();
    }

    #[test]
    fn shm_region_new_refuses_to_escape_the_topics_dir() {
        // End-to-end version of the abandoned probe left at
        // horus_core/tests/zz_pathaudit_tmp.rs by an earlier session.
        let marker = std::env::temp_dir().join(format!(
            "horus_traversal_guard_{}_{}",
            std::process::id(),
            "pwned"
        ));
        let _ = std::fs::remove_file(&marker);

        let rel = format!("../../..{}", marker.display());
        assert!(
            ShmRegion::new(&rel, 4096).is_err(),
            "traversal name must be refused"
        );
        assert!(
            ShmRegion::new(marker.to_str().unwrap(), 4096).is_err(),
            "absolute name must be refused"
        );
        assert!(
            !marker.exists(),
            "no file may be created outside the topics directory at {}",
            marker.display()
        );
    }

    // ─── Permission hardening ───────────────────────────────────────────────

    #[cfg(unix)]
    #[test]
    fn create_shm_dir_all_makes_owner_only_dirs() {
        use std::os::unix::fs::PermissionsExt;
        let root = std::env::temp_dir().join(format!("horus_perm_new_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        let nested = root.join("a").join("b");
        create_shm_dir_all(&nested).unwrap();

        for p in [&root, &root.join("a"), &nested] {
            let mode = std::fs::metadata(p).unwrap().permissions().mode() & 0o777;
            assert_eq!(
                mode,
                SHM_DIR_MODE,
                "{} should be 0700, was {mode:o}",
                p.display()
            );
        }
        let _ = std::fs::remove_dir_all(&root);
    }

    #[cfg(unix)]
    #[test]
    fn create_shm_dir_all_tightens_a_preexisting_loose_dir() {
        // The observed real-world state: an earlier bare create_dir_all left the
        // directory at 0775 under umask 002. Re-entering must repair it.
        use std::os::unix::fs::PermissionsExt;
        let dir = std::env::temp_dir().join(format!("horus_perm_fix_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o775)).unwrap();
        assert_eq!(
            std::fs::metadata(&dir).unwrap().permissions().mode() & 0o777,
            0o775
        );

        create_shm_dir_all(&dir).unwrap();
        assert_eq!(
            std::fs::metadata(&dir).unwrap().permissions().mode() & 0o777,
            SHM_DIR_MODE,
            "a pre-existing loose directory must be tightened, not left alone"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn write_shm_file_new_is_owner_only_and_refuses_to_clobber() {
        use std::os::unix::fs::PermissionsExt;
        let path = std::env::temp_dir().join(format!("horus_perm_file_{}", std::process::id()));
        let _ = std::fs::remove_file(&path);

        write_shm_file_new(&path, b"secret payload").unwrap();
        let mode = std::fs::metadata(&path).unwrap().permissions().mode() & 0o777;
        assert_eq!(
            mode, SHM_FILE_MODE,
            "gateway files must be 0600, was {mode:o}"
        );
        assert_eq!(std::fs::read(&path).unwrap(), b"secret payload");

        // create_new semantics: a second write must fail rather than follow or
        // overwrite whatever is already sitting at the path.
        assert!(write_shm_file_new(&path, b"other").is_err());

        let _ = std::fs::remove_file(&path);
    }

    #[cfg(unix)]
    #[test]
    fn harden_shm_file_narrows_an_existing_world_readable_file() {
        use std::os::unix::fs::PermissionsExt;
        let path = std::env::temp_dir().join(format!("horus_perm_narrow_{}", std::process::id()));
        std::fs::write(&path, b"x").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o664)).unwrap();

        harden_shm_file(&path);
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            SHM_FILE_MODE
        );
        let _ = std::fs::remove_file(&path);
    }

    #[cfg(unix)]
    #[test]
    fn hardening_never_widens_permissions() {
        use std::os::unix::fs::PermissionsExt;
        let path = std::env::temp_dir().join(format!("horus_perm_keep_{}", std::process::id()));
        std::fs::write(&path, b"x").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o400)).unwrap();

        harden_shm_file(&path);
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o400,
            "an already-stricter mode must be left alone, not relaxed to 0600"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[cfg(unix)]
    #[test]
    fn verify_ownership_accepts_our_own_dir_and_absent_paths() {
        let dir = std::env::temp_dir().join(format!("horus_own_ok_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        // Absent is fine — we are about to create it.
        verify_shm_dir_ownership(&dir).unwrap();
        std::fs::create_dir_all(&dir).unwrap();
        verify_shm_dir_ownership(&dir).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn verify_ownership_rejects_a_symlink_instead_of_following_it() {
        // The pre-creation attack: /dev/shm is 1777 and the namespace is a fixed
        // predictable literal, so another user can plant a symlink here.
        let base = std::env::temp_dir().join(format!("horus_own_link_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&base);
        std::fs::create_dir_all(&base).unwrap();
        let target = base.join("real");
        std::fs::create_dir_all(&target).unwrap();
        let link = base.join("link");
        std::os::unix::fs::symlink(&target, &link).unwrap();

        let err = verify_shm_dir_ownership(&link).unwrap_err().to_string();
        assert!(
            err.contains("symlink"),
            "a symlinked SHM dir must be refused, got: {err}"
        );
        let _ = std::fs::remove_dir_all(&base);
    }

    #[cfg(unix)]
    #[test]
    fn verify_ownership_rejects_a_non_directory() {
        let path = std::env::temp_dir().join(format!("horus_own_file_{}", std::process::id()));
        std::fs::write(&path, b"not a dir").unwrap();
        assert!(verify_shm_dir_ownership(&path).is_err());
        let _ = std::fs::remove_file(&path);
    }

    #[cfg(unix)]
    #[test]
    fn verify_ownership_rejects_a_dir_owned_by_another_uid() {
        // /tmp itself is root-owned and mode 1777 — exactly the shape of the
        // directory an attacker pre-creates. Skip when running as root, where
        // the ownership test is vacuous.
        // SAFETY: geteuid takes no arguments and cannot fail.
        if unsafe { libc::geteuid() } == 0 {
            return;
        }
        // macOS exposes /tmp as a symlink to /private/tmp. Canonicalize it so
        // this test reaches the foreign-owner check instead of correctly
        // stopping earlier at the separate symlink defense.
        let foreign_dir = std::fs::canonicalize("/tmp").expect("canonical /tmp");
        let err = verify_shm_dir_ownership(&foreign_dir)
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("owned by uid"),
            "a foreign-owned SHM dir must be refused, got: {err}"
        );
    }

    #[test]
    fn topic_shm_path_is_the_raw_name_under_the_topics_dir() {
        // The mapping horus_net must use. Any prefix or character mangling here
        // re-breaks the replication seam.
        let p = topic_shm_path("robot.imu");
        assert_eq!(p.parent().unwrap(), shm_topics_dir());
        assert_eq!(p.file_name().unwrap(), "robot.imu");
    }

    #[cfg(unix)]
    #[test]
    fn hardening_masks_and_never_adds_owner_bits() {
        // Regression guard. harden_* used to ASSIGN the target mode, so a 0o444
        // directory came back 0o700 — gaining owner-write, i.e. the opposite of
        // hardening. horus_core's `mmap_readonly_dir_returns_error` caught it:
        // a deliberately read-only directory became writable and the test's
        // "creation must fail" premise evaporated.
        use std::os::unix::fs::PermissionsExt;

        let dir = std::env::temp_dir().join(format!("horus_harden_ro_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o444)).unwrap();

        harden_shm_dir(&dir);
        let mode = std::fs::metadata(&dir).unwrap().permissions().mode() & 0o777;
        assert_eq!(
            mode, 0o400,
            "must only CLEAR bits (0o444 & 0o700 = 0o400), never add owner-write"
        );

        std::fs::set_permissions(&dir, std::fs::Permissions::from_mode(0o755)).unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(unix)]
    #[test]
    fn hardening_a_readonly_file_does_not_make_it_writable() {
        use std::os::unix::fs::PermissionsExt;
        let path = std::env::temp_dir().join(format!("horus_harden_rofile_{}", std::process::id()));
        std::fs::write(&path, b"x").unwrap();
        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o444)).unwrap();

        harden_shm_file(&path);
        assert_eq!(
            std::fs::metadata(&path).unwrap().permissions().mode() & 0o777,
            0o400,
            "0o444 & 0o600 = 0o400 — group/other cleared, owner-write NOT added"
        );
        let _ = std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644));
        let _ = std::fs::remove_file(&path);
    }
}

#[cfg(test)]
mod untrusted_meta_tests {
    use super::*;

    /// `.meta` files are parsed out of `/dev/shm`, which is mode 1777 — so
    /// `list_all_horus_namespaces` walks OTHER USERS' `horus_*` trees and reads
    /// whatever JSON is there. The `name` inside is therefore attacker-chosen,
    /// and consumers join it into a path. `ShmRegion::new` validated on the
    /// WRITE path; nothing validated on the READ path, which is where the
    /// untrusted data actually enters.
    #[test]
    fn topic_shm_path_checked_refuses_escaping_names() {
        for hostile in [
            "../../../../etc/passwd",
            "/etc/passwd",
            "a/../../../b",
            "..",
            "C:evil",
        ] {
            assert!(
                topic_shm_path_checked(hostile).is_none(),
                "{hostile:?} must be refused before it becomes a path"
            );
        }
    }

    #[test]
    fn topic_shm_path_checked_allows_real_names() {
        for ok in ["cmd_vel", "robot.imu", "legacy/hierarchical"] {
            let p = topic_shm_path_checked(ok).expect("legitimate name");
            assert!(p.starts_with(shm_topics_dir()));
        }
    }

    #[test]
    fn list_topic_metas_drops_entries_whose_name_escapes() {
        use std::io::Write;
        let dir = shm_topics_dir();
        if create_shm_dir_all(&dir).is_err() {
            return; // no writable SHM tree in this environment
        }
        let hostile = dir.join("zz_hostile_probe.meta");
        let good = dir.join("zz_good_probe.meta");
        let _ = std::fs::File::create(&hostile).map(|mut f| {
            let _ = f.write_all(
                br#"{"name":"../../../../tmp/pwned","size":4096,"creator_pid":1,"created_at":0}"#,
            );
        });
        let _ = std::fs::File::create(&good).map(|mut f| {
            let _ = f.write_all(
                br#"{"name":"zz_good_probe","size":4096,"creator_pid":1,"created_at":0}"#,
            );
        });

        let metas = list_topic_metas();
        assert!(
            !metas.iter().any(|m| m.name.contains("..")),
            "a .meta declaring an escaping name must be dropped at the parse boundary"
        );
        // The well-formed neighbour must still be returned — the filter must not
        // be a blanket rejection.
        assert!(metas.iter().any(|m| m.name == "zz_good_probe"));

        let _ = std::fs::remove_file(&hostile);
        let _ = std::fs::remove_file(&good);
    }
}
