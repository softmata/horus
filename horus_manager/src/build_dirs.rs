//! Where cargo artifacts go, and where HORUS looks for them — one answer.
//!
//! THE BUG THIS EXISTS TO FIX. Build location and artifact lookup were decided
//! independently. Only three cargo spawns in `run_rust.rs` (:247, :753, :764)
//! set `CARGO_TARGET_DIR`, and all three sit inside the legacy
//! `if Path::new(CARGO_TOML).exists()` branch — the path for projects that have
//! their own root `Cargo.toml`. The template path (a project scaffolded by
//! `horus new`, which has no root manifest) spawned cargo with no
//! `CARGO_TARGET_DIR` at all and relied on cargo's default, while every artifact
//! lookup hardcoded the string `.horus/target/<profile>/<name>`.
//!
//! A child process inherits its parent's environment, so an exported
//! `CARGO_TARGET_DIR` redirected the build but not the lookup:
//!
//! ```text
//! $ CARGO_TARGET_DIR=/tmp/xtd horus run
//!     Finished `dev` profile ... in 45.03s
//! > Executing...
//! Error: No such file or directory (os error 2)
//! ```
//!
//! The build succeeded into `/tmp/xtd/debug/demo_env`; HORUS looked in
//! `.horus/target/debug/`. On the workspace path it is quieter and worse —
//! `run/mod.rs` guards the exec with `if binary.exists()` and no `else`, so
//! `horus run -p member` exits 0 having run nothing at all.
//!
//! Both halves now come from here, so they cannot disagree.
//!
//! WHY ABSOLUTE. Several cargo spawns run with `.current_dir(".horus")`
//! (run_rust.rs:337, :597, :858). `CARGO_TARGET_DIR` is resolved relative to the
//! cargo process's working directory, so passing the relative `.horus/target`
//! from those sites would land artifacts in `.horus/.horus/target`. That is why
//! those sites set nothing at all. An absolute path is correct from any cwd.

use std::path::{Path, PathBuf};

/// Target directory for a HORUS project, as an absolute path.
///
/// Honours `CARGO_TARGET_DIR` when the user has exported one — respecting the
/// intent rather than silently ignoring it — but resolves it to an absolute
/// path so build and lookup agree no matter which cwd cargo is spawned from.
///
/// This function is also the whole seam for pooling artifacts across projects
/// later: every HORUS project depends on the same `horus`/`horus_core`/
/// `horus_types`/`horus_macros` path deps, so a shared target directory builds
/// them once. Measured on three scratch projects, a second project added 2 MB
/// to a shared `cargo check` tree and 66 MB to a shared `cargo build` tree,
/// against ~900 MB-1.2 GB for a private one. Changing the fallback below is the
/// entire change — but it also requires per-project unique binary names, or two
/// projects both called `controller` overwrite each other's executable.
pub fn target_dir(project_dir: &Path) -> PathBuf {
    if let Some(dir) = std::env::var_os("CARGO_TARGET_DIR") {
        let p = PathBuf::from(dir);
        return if p.is_absolute() {
            p
        } else {
            project_dir.join(p)
        };
    }
    project_dir.join(".horus").join("target")
}

/// Absolute path of a built binary: `<target>/<profile>/<name>`.
pub fn binary_path(project_dir: &Path, profile: &str, name: &str) -> PathBuf {
    target_dir(project_dir).join(profile).join(name)
}

/// Point a `cargo` invocation at [`target_dir`], overriding any inherited value.
///
/// Call this on EVERY cargo spawn that builds a HORUS project. Setting it
/// explicitly is what makes the inherited-environment bug impossible: cargo and
/// the subsequent lookup now read the same source of truth.
pub fn apply(cmd: &mut std::process::Command, project_dir: &Path) {
    let dir = target_dir(project_dir);
    // Every cargo spawn goes through here, which makes this the one place that
    // sees a build about to start — so it is also where we can notice we are
    // queued behind someone else's build and say so. See `warn_if_locked` for
    // why the user would otherwise see an unexplained hang.
    if is_locked(&dir) {
        crate::cli_output::info(
            "Another build is using this project's target directory — waiting for it to finish",
        );
    }
    cmd.env("CARGO_TARGET_DIR", dir);
}

/// Whether cargo currently holds the build lock under `target_dir`.
///
/// cargo takes an advisory lock on `<target>/<profile>/.cargo-lock` for the
/// duration of a build and prints `Blocking waiting for file lock on build
/// directory` while it waits. HORUS never shows that line: every build spawn
/// sets `Stdio::piped()` on both streams and calls `cmd.output()`, which does
/// not return until the process exits, so the explanation arrives only after
/// the wait is over. To the user it is an indefinite hang behind a spinner.
///
/// Probing the lock before spawning lets us say what is happening up front.
/// The file's existence proves nothing — cargo leaves it behind — so take the
/// lock to find out, and immediately release it if we got it.
pub fn is_locked(target_dir: &Path) -> bool {
    for profile in ["debug", "release"] {
        let lock = target_dir.join(profile).join(".cargo-lock");
        if !lock.exists() {
            continue;
        }
        let Ok(file) = std::fs::OpenOptions::new().write(true).open(&lock) else {
            continue;
        };
        #[cfg(unix)]
        {
            use std::os::unix::io::AsRawFd;
            let fd = file.as_raw_fd();
            // LOCK_EX | LOCK_NB — a non-zero return means somebody else holds it.
            if unsafe { libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) } != 0 {
                return true;
            }
            unsafe { libc::flock(fd, libc::LOCK_UN) };
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The regression: build location and lookup must agree under an inherited
    /// `CARGO_TARGET_DIR`, which is exactly what they did not do.
    #[test]
    fn lookup_follows_an_inherited_target_dir() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let project = Path::new("/tmp/proj");
        let old = std::env::var_os("CARGO_TARGET_DIR");

        std::env::set_var("CARGO_TARGET_DIR", "/tmp/xtd");
        assert_eq!(target_dir(project), PathBuf::from("/tmp/xtd"));
        assert_eq!(
            binary_path(project, "debug", "demo"),
            PathBuf::from("/tmp/xtd/debug/demo"),
            "lookup must follow the same env var the build follows"
        );

        // A relative override is resolved against the project, not against
        // whatever cwd cargo happens to be spawned with.
        std::env::set_var("CARGO_TARGET_DIR", "out");
        assert_eq!(target_dir(project), PathBuf::from("/tmp/proj/out"));

        std::env::remove_var("CARGO_TARGET_DIR");
        assert_eq!(
            target_dir(project),
            PathBuf::from("/tmp/proj/.horus/target")
        );

        match old {
            Some(v) => std::env::set_var("CARGO_TARGET_DIR", v),
            None => std::env::remove_var("CARGO_TARGET_DIR"),
        }
    }
}
