//! `horus self update` — upgrade the horus CLI and installed plugins.
//!
//! The CLI half resolves the newest published release from the GitHub releases
//! API, downloads the platform asset published for that tag, verifies it
//! against the release's SHA256SUMS, replaces the running binary and refreshes
//! the cached source tree at the *same* tag.
//!
//! Both halves always move together. install.sh used to take the binary from
//! `releases/latest` and the source from `main` HEAD; the two trees then
//! disagreed about the shm wire format and the CLI could not read what its own
//! libraries wrote. And the source half is not optional: `horus run`/`horus
//! build` generate .horus/Cargo.toml with horus as *path* dependencies into
//! ~/.horus/cache/horus@<version> (cargo_gen.rs -> find_horus_source_dir), so
//! a binary-only update leaves a CLI that cannot build a single Rust project.
//!
//! What this replaced: the check polled the package registry at
//! horusrobotics.dev, which 404s, and answered every non-200 with `Ok(None)` —
//! "could not determine latest version" on the way to exit 0. The install then
//! ran `cargo install --path .` against the source snapshot from install day,
//! which is by construction the version already installed. Neither half could
//! ever update anything, and both failed quietly enough that nobody noticed.

use anyhow::{anyhow, bail, ensure, Context, Result};
use colored::*;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Duration;

/// The repository install.sh clones and downloads release assets from.
const REPO: &str = "softmata/horus";

/// Run `horus self update`.
///
/// - `check_only`: If true, report what an update would do and change nothing.
pub fn run_upgrade(check_only: bool) -> Result<()> {
    println!("{}", "horus self update".bold());
    println!();

    let cli_outcome = update_cli(check_only);

    // ── Phase 2: Check plugin updates ────────────────────────────────────
    // Plugins live in their own registry, so they are checked even when the
    // CLI half failed — but the CLI failure is still what the command exits
    // with, below.
    let plugins = list_installed_plugins();
    if !plugins.is_empty() {
        println!();
        println!("  {}", "Plugins:".bold());
        for plugin in &plugins {
            println!("    {} {}", "-".dimmed(), plugin);
        }
        if !check_only {
            upgrade_plugins();
        } else {
            check_plugin_updates();
        }
    }

    cli_outcome
}

/// The CLI half of `self update`: resolve, report, and (unless `check_only`)
/// install.
///
/// Every failure here is returned. It used to print a yellow note and hand back
/// `Ok(())`, so a check against a permanently 404ing endpoint looked exactly
/// like "you are up to date" and exited 0.
fn update_cli(check_only: bool) -> Result<()> {
    let current_version = env!("CARGO_PKG_VERSION");
    println!("  Current version: {}", current_version.cyan());
    println!(
        "  Checking {}...",
        format!("github.com/{REPO} releases").dimmed()
    );

    // Flattened into one message rather than wrapped with .context(): the CLI
    // maps anyhow errors with `err.to_string()` (horus_core/src/error.rs:962),
    // which prints only the outermost layer — a bare "could not check for
    // updates" with the reason dropped is how this failure stayed invisible.
    let latest =
        resolve_latest_release().map_err(|e| anyhow!("could not check for updates: {e}"))?;
    println!("  Latest version:  {}", latest.version.cyan());

    // Compared as semver, not as strings: a development build whose version is
    // ahead of the last release must not be told to "update" backwards to it.
    let current = semver::Version::parse(current_version)
        .with_context(|| format!("this binary reports a non-semver version {current_version:?}"))?;
    let newest = semver::Version::parse(&latest.version)
        .with_context(|| format!("release reported a non-semver version {:?}", latest.version))?;

    if newest < current {
        println!(
            "  {} This build ({}) is newer than the latest release ({}) — nothing to do",
            "*".green(),
            current_version.dimmed(),
            latest.version.dimmed()
        );
        return Ok(());
    }
    if newest == current {
        println!("  {} horus is up to date", "*".green());
        return Ok(());
    }

    println!(
        "  {} New version available: {} -> {}",
        "!".yellow(),
        current_version.dimmed(),
        latest.version.green()
    );
    if check_only {
        println!("    Run {} to install it.", "horus self update".cyan());
        return Ok(());
    }
    upgrade_horus(&latest)
}

/// A release resolved from the GitHub API.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LatestRelease {
    /// The tag exactly as GitHub published it, e.g. `v0.4.1`. This is what the
    /// asset URLs and `git clone --branch` use, so it is kept verbatim.
    pub tag: String,
    /// The tag with any leading `v` stripped, e.g. `0.4.1`. Valid semver.
    pub version: String,
}

/// Resolve the newest published release from the GitHub releases API.
pub fn resolve_latest_release() -> Result<LatestRelease> {
    let url = format!("https://api.github.com/repos/{REPO}/releases/latest");
    let (status, body) = http_get(&url, Duration::from_secs(15))?;
    parse_release_response(status, &String::from_utf8_lossy(&body))
}

/// The version string of the newest published release, e.g. `"0.4.1"`.
///
/// This used to be `Result<Option<String>>` and returned `Ok(None)` for every
/// failure — 403, rate limit, DNS, timeout, malformed body — which callers
/// could not tell apart from "already current". There is no such state now:
/// either a release was resolved or the reason it was not is in the error.
pub fn check_latest_version() -> Result<String> {
    resolve_latest_release().map(|r| r.version)
}

/// Turn a GitHub releases-API response into a resolved release.
///
/// Split out from the request so the failure modes that matter are unit
/// testable without a network: this is where "the check failed" stops being
/// confusable with "there is no update".
fn parse_release_response(status: u16, body: &str) -> Result<LatestRelease> {
    match status {
        200..=299 => {}
        // GitHub answers an unauthenticated caller that is over the hourly
        // limit with 403, and a request that sends no User-Agent with 403 as
        // well. Both used to arrive here as `Ok(None)`.
        403 | 429 => {
            let detail = github_message(body).unwrap_or_else(|| "no detail given".to_string());
            bail!(
                "GitHub refused the release check (HTTP {status}): {detail}. \
                 This is usually the unauthenticated API rate limit, which resets within the hour. \
                 Releases are listed at https://github.com/{REPO}/releases/latest"
            );
        }
        404 => bail!(
            "no published release at https://api.github.com/repos/{REPO}/releases/latest (HTTP 404)"
        ),
        _ => {
            let detail = github_message(body).unwrap_or_else(|| "no detail given".to_string());
            bail!("release check failed (HTTP {status}): {detail}");
        }
    }

    let json: serde_json::Value = serde_json::from_str(body)
        .context("the release API returned something that is not JSON")?;
    let tag = json
        .get("tag_name")
        .and_then(|v| v.as_str())
        .ok_or_else(|| anyhow!("the release API response has no tag_name"))?;

    // Releases are tagged `v0.4.0`; semver::Version::parse rejects the `v`.
    // Bare `0.4.0` tags are accepted too so a differently tagged release does
    // not brick the updater.
    let version = tag.strip_prefix('v').unwrap_or(tag);
    semver::Version::parse(version)
        .with_context(|| format!("release tag {tag:?} is not a version this CLI can compare"))?;

    Ok(LatestRelease {
        tag: tag.to_string(),
        version: version.to_string(),
    })
}

/// GitHub puts the human-readable reason for a refusal in `message`.
fn github_message(body: &str) -> Option<String> {
    serde_json::from_str::<serde_json::Value>(body)
        .ok()?
        .get("message")?
        .as_str()
        .map(str::to_string)
}

/// One HTTP GET, returning the status alongside the body so a non-2xx can be
/// reported with its reason instead of being collapsed into "no update".
/// A GitHub token from the environment, if one is set and non-empty.
///
/// `GITHUB_TOKEN` is what Actions injects; `GH_TOKEN` is what the `gh` CLI
/// uses, so a developer who has authenticated `gh` usually has it too. Blank
/// values are treated as absent -- an empty `Authorization: Bearer` header is
/// rejected by GitHub with a 401, which would turn "no token" into a hard
/// failure rather than the anonymous request it should be.
fn github_token() -> Option<String> {
    ["GITHUB_TOKEN", "GH_TOKEN"]
        .iter()
        .filter_map(|k| std::env::var(k).ok())
        .map(|v| v.trim().to_string())
        .find(|v| !v.is_empty())
}

fn http_get(url: &str, timeout: Duration) -> Result<(u16, Vec<u8>)> {
    let client = reqwest::blocking::Client::builder()
        .timeout(timeout)
        // GitHub replies 403 to an API request that sends no User-Agent, which
        // looks identical to a rate limit if you only read the status code.
        .user_agent(format!(
            "horus-cli/{} (+https://github.com/{REPO})",
            env!("CARGO_PKG_VERSION")
        ))
        .build()
        .context("building the HTTP client")?;

    let mut req = client
        .get(url)
        .header("Accept", "application/vnd.github+json");

    // Authenticate when a token is available.
    //
    // GitHub's unauthenticated API quota is per source IP, and on a shared CI
    // runner that address belongs to hundreds of unrelated jobs. The check then
    // fails with a 403 that has nothing to do with this repository or this
    // machine -- distribution.yml's macOS job failed exactly that way while
    // holding a usable GITHUB_TOKEN it never passed on. The same quota is what
    // a developer behind a corporate NAT hits.
    //
    // Authenticated requests get a far higher limit, which the 403 body already
    // tells the user; this is that advice taken. No token, no header, and the
    // anonymous path behaves as before -- a token is an optimisation here, never
    // a requirement, because reading public releases needs no credentials.
    if let Some(token) = github_token() {
        req = req.header("Authorization", format!("Bearer {token}"));
    }

    let resp = req
        .send()
        .map_err(|e| anyhow!("could not reach {url}: {}", error_chain(&e)))?;
    let status = resp.status().as_u16();
    let body = resp
        .bytes()
        .map_err(|e| anyhow!("reading the response from {url}: {}", error_chain(&e)))?;
    Ok((status, body.to_vec()))
}

/// Flatten an error and its causes into one line.
///
/// reqwest's Display stops at "error sending request for url (...)" and leaves
/// the reason — connection refused, DNS failure, a TLS error, a proxy that is
/// not there — in the source chain. The CLI maps anyhow errors with
/// `err.to_string()` (horus_core/src/error.rs:962), so anything left in a chain
/// never reaches the user, and "could not check for updates" without a reason
/// is barely better than the silence this replaced.
fn error_chain(e: &dyn std::error::Error) -> String {
    let mut parts = vec![e.to_string()];
    let mut source = e.source();
    while let Some(cause) = source {
        let text = cause.to_string();
        // reqwest repeats the outer message in its first cause on some paths.
        if !parts.contains(&text) {
            parts.push(text);
        }
        source = cause.source();
    }
    parts.join(": ")
}

// ── Installing a release ────────────────────────────────────────────────────

/// Install `release`: fetch its source, fetch and verify its binary, replace
/// the running binary, and record what was installed.
pub(crate) fn upgrade_horus(release: &LatestRelease) -> Result<()> {
    validate_release(release)?;

    let target = current_binary_path()?;
    // Checked before the download rather than after it: installs into a
    // root-owned directory are common, and finding out about them only after
    // a 50MB transfer and a clone wastes the user's time twice over.
    preflight_writable(&target)?;

    let cache = cache_root()?;
    std::fs::create_dir_all(&cache).with_context(|| format!("creating {}", cache.display()))?;

    // Scratch lives under the cache root, not in /tmp: the fetched source is
    // renamed into place inside this same directory at the end, and /tmp is
    // usually a different mount, where the rename fails with EXDEV.
    let work = tempfile::Builder::new()
        .prefix(".horus-update-")
        .tempdir_in(&cache)
        .with_context(|| format!("creating a scratch directory in {}", cache.display()))?;

    // The source is fetched first, and from the same tag as the binary. Mixing
    // refs is the bug this whole path exists to prevent, and fetching first
    // means a failure here leaves the installed CLI untouched.
    println!("  Fetching source for {}...", release.tag.cyan());
    let fetched = clone_at_tag(&release.tag, work.path())?;
    let commit = git_head_commit(&fetched);
    let topic_version = topic_version_of(&fetched);

    let downloaded = match platform_asset_name() {
        Some(asset) => {
            println!("  Looking for {}...", asset.cyan());
            download_release_asset(&release.tag, &asset)?
                .map(|(archive, sums)| (asset, archive, sums))
        }
        None => None,
    };

    // The cache is updated before the binary is swapped. If the swap then
    // fails, the user has new source and an old CLI — a mismatch version.rs
    // warns about loudly and `horus self update` can retry — rather than a new
    // CLI compiling user projects against the previous release's headers.
    let source_dir = install_source_cache(&fetched, &release.version)?;
    println!(
        "  {} Source cached at {}",
        "*".green(),
        source_dir.display().to_string().dimmed()
    );

    match downloaded {
        Some((asset, archive, sums)) => {
            println!("  Verifying checksum...");
            // A dedicated staging directory, so the search for the binary
            // inside the unpacked archive cannot pick up anything else this
            // function left in the scratch tree.
            let stage = work.path().join("asset");
            std::fs::create_dir_all(&stage)
                .with_context(|| format!("creating {}", stage.display()))?;
            install_verified_asset(&archive, &asset, &sums, &stage, &target)?;
            println!("  {} Installed verified binary", "*".green());
        }
        None => {
            // release.yml does not build every platform, so an asset that is
            // not published is not an error — compile the tree just fetched.
            // That is the *same tag*, which is what install.sh's slow path
            // does, not a rebuild of whatever source happens to be on disk.
            println!(
                "  {} No published binary for this platform — building {} from source (~3-5 min)",
                "!".yellow(),
                release.version.cyan()
            );
            let built = build_from_source(&source_dir, work.path())?;
            replace_binary(&built, &target)?;
        }
    }

    // A successful rename, like a cargo exit code of 0, is not proof that the
    // binary on disk is the new one. Ask it rather than asserting it.
    match installed_horus_version(&target) {
        Some(installed) if installed == release.version => {}
        Some(installed) => bail!(
            "{} reports version {installed} after the update, expected {} — \
             the install state files were left untouched",
            target.display(),
            release.version
        ),
        None => bail!(
            "{} could not be run to report its version after the update",
            target.display()
        ),
    }

    write_install_state(
        &horus_home()?,
        &InstallManifest {
            version: release.version.clone(),
            tag: release.tag.clone(),
            commit,
            topic_version,
            source_dir: source_dir.display().to_string(),
            binary: target.display().to_string(),
            install_method: "self-update".to_string(),
            installed_at: chrono::Utc::now().to_rfc3339_opts(chrono::SecondsFormat::Secs, true),
        },
    )?;

    println!("  {} Upgraded to {}", "*".green(), release.version.green());
    Ok(())
}

/// Both fields reach a process argument — the version through `cargo install`,
/// the tag through `git clone --branch` — and both are reported back to the
/// user as what was installed, so neither is taken on trust from a JSON body.
/// A leading `-` would be read as a flag, and a `/` or `..` in the version
/// would point the `remove_dir_all` in `install_source_cache` at a directory
/// outside the cache (install.sh guards its cache path the same way).
fn validate_release(release: &LatestRelease) -> Result<()> {
    semver::Version::parse(&release.version).with_context(|| {
        format!(
            "release reported a non-semver version {:?}",
            release.version
        )
    })?;
    let tag_ok = !release.tag.is_empty()
        && !release.tag.starts_with('-')
        && !release.tag.starts_with('.')
        && release
            .tag
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '_' | '-' | '+'));
    ensure!(tag_ok, "refusing to use {:?} as a git tag", release.tag);
    Ok(())
}

/// The release asset for this machine, named the way install.sh's
/// `detect_os`/`detect_arch` name it so the two cannot resolve different files
/// for the same host: `horus-{linux,macos,windows}-{amd64,arm64,armv7}.{tar.gz,zip}`.
///
/// `None` on a platform no release asset is named for. That is not an error —
/// it is the slow path, the same one install.sh takes when the download 404s.
fn platform_asset_name() -> Option<String> {
    let os = match std::env::consts::OS {
        "linux" => "linux",
        "macos" => "macos",
        "windows" => "windows",
        _ => return None,
    };
    let arch = match std::env::consts::ARCH {
        "x86_64" => "amd64",
        "aarch64" => "arm64",
        "arm" => "armv7",
        _ => return None,
    };
    let ext = if os == "windows" { "zip" } else { "tar.gz" };
    Some(format!("horus-{os}-{arch}.{ext}"))
}

/// The file name of the CLI binary on this platform.
fn binary_file_name() -> &'static str {
    if cfg!(windows) {
        "horus.exe"
    } else {
        "horus"
    }
}

/// Download the platform asset and the release's SHA256SUMS.
///
/// `Ok(None)` means only one thing: the release does not publish an asset for
/// this platform. A missing or unreadable SHA256SUMS is an error, never a
/// reason to install the asset unverified — install.sh:315-317 refuses for the
/// same reason.
fn download_release_asset(tag: &str, asset: &str) -> Result<Option<(Vec<u8>, String)>> {
    let base = format!("https://github.com/{REPO}/releases/download/{tag}");
    let (status, archive) = http_get(&format!("{base}/{asset}"), Duration::from_secs(600))?;
    if status == 404 {
        return Ok(None);
    }
    ensure!(
        (200..300).contains(&status),
        "downloading {base}/{asset} failed (HTTP {status})"
    );
    ensure!(!archive.is_empty(), "{base}/{asset} was empty");

    let (status, sums) = http_get(&format!("{base}/SHA256SUMS"), Duration::from_secs(60))?;
    ensure!(
        (200..300).contains(&status) && !sums.is_empty(),
        "could not fetch {base}/SHA256SUMS (HTTP {status}) — refusing to install an unverified binary. \
         Reinstall from source instead: HORUS_BUILD_FROM_SOURCE=1 curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash"
    );

    Ok(Some((archive, String::from_utf8_lossy(&sums).into_owned())))
}

/// Verify, unpack and install, in that order.
///
/// One function so a caller cannot get the order wrong: nothing is written over
/// `target`, and the archive is not even parsed, until its digest matches the
/// published one.
fn install_verified_asset(
    archive: &[u8],
    asset: &str,
    sums: &str,
    stage: &Path,
    target: &Path,
) -> Result<()> {
    verify_digest(archive, asset, sums)?;
    let binary = extract_asset(archive, asset, stage)?;
    replace_binary(&binary, target)
}

/// Check `archive` against the release's SHA256SUMS, fail-closed at every step
/// the way install.sh:284-317 does: no entry for this asset, or a digest that
/// does not match, aborts rather than continuing unverified. TLS alone does not
/// cover a compromised or substituted asset.
fn verify_digest(archive: &[u8], asset: &str, sums: &str) -> Result<()> {
    let expected = sums
        .lines()
        .find_map(|line| {
            let (digest, name) = line.split_once(char::is_whitespace)?;
            // `sha256sum` writes "<digest>  <name>" for text and "<digest> *<name>"
            // for binary mode; both leave the name after the whitespace run.
            let name = name.trim_start().trim_start_matches('*');
            (name == asset).then(|| digest.trim().to_ascii_lowercase())
        })
        .ok_or_else(|| {
            anyhow!("SHA256SUMS has no entry for {asset}; refusing to install an unverified binary")
        })?;

    let actual = {
        use sha2::{Digest, Sha256};
        let mut hasher = Sha256::new();
        hasher.update(archive);
        hex::encode(hasher.finalize())
    };

    ensure!(
        expected == actual,
        "checksum MISMATCH for {asset}\n    expected: {expected}\n    actual:   {actual}\n  \
         Refusing to install: this asset does not match the published release."
    );
    Ok(())
}

/// Unpack a verified asset into `stage` and return the horus binary inside it.
fn extract_asset(archive: &[u8], asset: &str, stage: &Path) -> Result<PathBuf> {
    if asset.ends_with(".zip") {
        // The Windows asset is the only zipped one and no zip decoder is linked
        // into the CLI. Windows 10 1803+ ships bsdtar as tar.exe, which reads
        // zip; Expand-Archive covers hosts older than that.
        let archive_path = stage.join(asset);
        std::fs::write(&archive_path, archive)
            .with_context(|| format!("writing {}", archive_path.display()))?;
        let unpacked = ran_ok(
            Command::new("tar")
                .arg("-xf")
                .arg(&archive_path)
                .arg("-C")
                .arg(stage),
        ) || ran_ok(Command::new("powershell").args([
            "-NoProfile",
            "-Command",
            &format!(
                "Expand-Archive -LiteralPath '{}' -DestinationPath '{}' -Force",
                ps_quote(&archive_path),
                ps_quote(stage)
            ),
        ]));
        ensure!(
            unpacked,
            "could not unpack {asset}: neither tar nor Expand-Archive could read it"
        );
    } else {
        let decoder = flate2::read::GzDecoder::new(archive);
        tar::Archive::new(decoder)
            .unpack(stage)
            .with_context(|| format!("unpacking {asset}"))?;
    }

    let wanted = binary_file_name();
    walkdir::WalkDir::new(stage)
        .into_iter()
        .flatten()
        .find(|e| e.file_type().is_file() && e.file_name() == wanted)
        .map(|e| e.into_path())
        .ok_or_else(|| anyhow!("{asset} does not contain a {wanted} binary"))
}

/// Run a command, discarding its output, and report whether it succeeded.
fn ran_ok(cmd: &mut Command) -> bool {
    cmd.output().map(|o| o.status.success()).unwrap_or(false)
}

/// Escape a path for a single-quoted PowerShell string.
fn ps_quote(path: &Path) -> String {
    path.display().to_string().replace('\'', "''")
}

/// Replace `target` with `new_bin`.
///
/// The replacement is staged in `target`'s own directory so the last step is a
/// rename within one filesystem: atomic, with no window in which the `horus` on
/// PATH is a half-written file. Staging in /tmp would cross a mount boundary on
/// most machines and fail with EXDEV.
fn replace_binary(new_bin: &Path, target: &Path) -> Result<()> {
    let dir = target
        .parent()
        .ok_or_else(|| anyhow!("{} has no parent directory", target.display()))?;
    let name = target
        .file_name()
        .and_then(|n| n.to_str())
        .ok_or_else(|| anyhow!("{} has no file name", target.display()))?;

    let staged = dir.join(format!(".{name}.new-{}", std::process::id()));
    let _ = std::fs::remove_file(&staged);
    std::fs::copy(new_bin, &staged).map_err(|e| write_error(e, dir))?;

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(&staged, std::fs::Permissions::from_mode(0o755))
            .with_context(|| format!("making {} executable", staged.display()))?;
    }

    if cfg!(windows) {
        // Windows refuses to overwrite or delete a running .exe, but it does
        // allow *renaming* one. Move the running binary aside, then rename the
        // staged file into its place; if that second step fails, put the old
        // one back so the user is not left with no horus at all. The .old file
        // stays locked until this process exits, so it is cleaned up by the
        // next update rather than here.
        let backup = dir.join(format!("{name}.old"));
        let _ = std::fs::remove_file(&backup);
        if let Err(e) = std::fs::rename(target, &backup) {
            let _ = std::fs::remove_file(&staged);
            return Err(write_error(e, dir));
        }
        if let Err(e) = std::fs::rename(&staged, target) {
            let _ = std::fs::rename(&backup, target);
            let _ = std::fs::remove_file(&staged);
            return Err(write_error(e, dir));
        }
    } else if let Err(e) = std::fs::rename(&staged, target) {
        // POSIX rename over a running binary is safe: processes already
        // executing it keep the old inode until they exit.
        let _ = std::fs::remove_file(&staged);
        return Err(write_error(e, dir));
    }

    Ok(())
}

/// Confirm the install directory can be written to at all.
///
/// Creating and removing a probe file is the only honest test: the directory
/// mode alone does not account for the sticky bit, ACLs, a read-only mount or
/// a container's user namespace.
fn preflight_writable(target: &Path) -> Result<()> {
    let dir = target
        .parent()
        .ok_or_else(|| anyhow!("{} has no parent directory", target.display()))?;
    let probe = dir.join(format!(".horus-update-probe-{}", std::process::id()));
    match std::fs::File::create(&probe) {
        Ok(_) => {
            let _ = std::fs::remove_file(&probe);
            Ok(())
        }
        Err(e) => Err(write_error(e, dir)),
    }
}

/// Turn a write failure on the install directory into advice.
///
/// `Permission denied (os error 13)` names neither the directory nor a remedy,
/// and a system-wide install (/usr/local/bin, or a HORUS_PREFIX tree) is the
/// common case where it happens.
fn write_error(e: std::io::Error, dir: &Path) -> anyhow::Error {
    if e.kind() == std::io::ErrorKind::PermissionDenied {
        return anyhow!(
            "no permission to write to {}. `horus self update` replaces the binary in place, so it \
             needs write access to that directory.\n  \
             Retry as: sudo -E \"$(command -v horus)\" self update\n  \
             (-E keeps HOME and HORUS_PREFIX, so the state and source cache stay where the \
             install put them rather than under root's home)\n  \
             Or reinstall into a directory you own: curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash",
            dir.display()
        );
    }
    anyhow!("writing to {}: {e}", dir.display())
}

// ── Source tree ─────────────────────────────────────────────────────────────

/// Clone the source at exactly `tag` into `into/source`.
fn clone_at_tag(tag: &str, into: &Path) -> Result<PathBuf> {
    let dest = into.join("source");
    let out = Command::new("git")
        .args(["clone", "--depth", "1", "--branch"])
        .arg(tag)
        .arg(format!("https://github.com/{REPO}.git"))
        .arg(&dest)
        .output()
        .context("running git — install git, or reinstall with the curl one-liner")?;
    ensure!(
        out.status.success(),
        "git clone of {tag} from https://github.com/{REPO}.git failed:\n{}",
        tail(&String::from_utf8_lossy(&out.stderr), 5)
    );

    // A clone can exit 0 with a tree that is unusable to `horus run`.
    // horus/Cargo.toml is the exact marker find_horus_source_dir() looks for
    // and horus_core/Cargo.toml carries the version; both are checked before
    // install_source_cache removes anything, so a bad fetch can never delete a
    // working cached tree (install.sh:236-241 guards the same way).
    for marker in ["horus/Cargo.toml", "horus_core/Cargo.toml"] {
        ensure!(
            dest.join(marker).exists(),
            "the tree fetched for {tag} is incomplete (no {marker})"
        );
    }
    Ok(dest)
}

/// Move a fetched tree into `<state root>/cache/horus@<version>`.
///
/// The `horus@<version>` name is a contract with two readers that rebuild it
/// from CARGO_PKG_VERSION rather than discovering it (run_rust.rs:1062,
/// registry/helpers.rs) — renaming the scheme breaks both.
fn install_source_cache(fetched: &Path, version: &str) -> Result<PathBuf> {
    let cache = cache_root()?;
    std::fs::create_dir_all(&cache).with_context(|| format!("creating {}", cache.display()))?;
    let dest = cache.join(format!("horus@{version}"));
    if dest.exists() {
        std::fs::remove_dir_all(&dest)
            .with_context(|| format!("replacing the cached source at {}", dest.display()))?;
    }
    std::fs::rename(fetched, &dest)
        .with_context(|| format!("moving the fetched source into {}", dest.display()))?;
    Ok(dest)
}

/// The commit the fetched tree is at, for the install manifest. `None` when git
/// cannot answer — the manifest records what is known, and readers tolerate a
/// missing field.
fn git_head_commit(dir: &Path) -> Option<String> {
    let out = Command::new("git")
        .arg("-C")
        .arg(dir)
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let sha = String::from_utf8_lossy(&out.stdout).trim().to_string();
    (sha.len() == 40 && sha.chars().all(|c| c.is_ascii_hexdigit())).then_some(sha)
}

/// The shm wire-format version the fetched tree speaks.
///
/// Read out of the source rather than linked against, because TOPIC_VERSION is
/// `pub(crate)` in horus_core. It is recorded because it, not the crate
/// version, is the number that actually breaks when a CLI and its libraries
/// come from different refs: two trees can both say 0.4.0 and still be unable
/// to read each other's topics.
fn topic_version_of(src: &Path) -> Option<u32> {
    let header = src.join("horus_core/src/communication/topic/header.rs");
    let text = std::fs::read_to_string(header).ok()?;
    let line = text
        .lines()
        .find(|l| l.contains("const TOPIC_VERSION") && l.contains('='))?;
    line.rsplit('=')
        .next()?
        .trim()
        .trim_end_matches(';')
        .trim()
        .parse()
        .ok()
}

/// Compile `src` and hand back the binary it produced.
///
/// Installed into a scratch root rather than straight into `~/.cargo/bin` so
/// the result goes through the same staged, atomic replace as a downloaded
/// binary — and so a build that fails halfway cannot leave a partly installed
/// CLI on PATH.
fn build_from_source(src: &Path, work: &Path) -> Result<PathBuf> {
    // The workspace root's Cargo.toml is a virtual manifest; `cargo install
    // --path` needs the package directory.
    let manifest_dir = src.join("horus_manager");
    let manifest = manifest_dir.join("Cargo.toml");
    let manifest_text = std::fs::read_to_string(&manifest)
        .with_context(|| format!("reading {}", manifest.display()))?;
    ensure!(
        manifest_text.contains("name = \"horus_manager\""),
        "{} is not the horus_manager package; refusing to install from it",
        manifest.display()
    );

    let root = work.join("build-root");
    // --locked so the build cannot silently pick up newer transitive deps.
    let status = Command::new("cargo")
        .current_dir(&manifest_dir)
        .args(["install", "--path", ".", "--locked", "--force", "--root"])
        .arg(&root)
        .status()
        .context("running cargo — install Rust from https://rustup.rs")?;
    ensure!(
        status.success(),
        "building horus from {} failed; retry manually with: cargo install --path {} --locked --force",
        src.display(),
        manifest_dir.display()
    );

    let built = root.join("bin").join(binary_file_name());
    ensure!(
        built.exists(),
        "cargo reported success but produced no {} in {}",
        binary_file_name(),
        root.display()
    );
    Ok(built)
}

// ── Installed state ─────────────────────────────────────────────────────────

/// The record written to `<state root>/install_manifest.toml`. install.sh writes
/// the same fields; version.rs and doctor.rs read them, and must tolerate the
/// file being absent — every install before this one predates it.
#[derive(Debug, serde::Serialize)]
struct InstallManifest {
    version: String,
    tag: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    commit: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    topic_version: Option<u32>,
    source_dir: String,
    binary: String,
    install_method: String,
    installed_at: String,
}

/// Write both state files into `home` — the root [`horus_home`] resolved.
fn write_install_state(home: &Path, manifest: &InstallManifest) -> Result<()> {
    std::fs::create_dir_all(home).with_context(|| format!("creating {}", home.display()))?;

    // A bare, newline-terminated version string, because version.rs:32-44 reads
    // this file with read_to_string().trim() and uninstall.sh:819 deletes it.
    // Nothing had written it since v0.2.0, which is why the version gate it
    // feeds was dead code.
    let version_file = home.join("installed_version");
    std::fs::write(&version_file, format!("{}\n", manifest.version))
        .with_context(|| format!("writing {}", version_file.display()))?;

    // Same shape install.sh's write_install_state() emits, header included, so
    // the two writers produce files a reader cannot tell apart except by the
    // install_method they record.
    let manifest_file = home.join("install_manifest.toml");
    let header = concat!(
        "# Written by `horus self update`. Describes the tree this install came from;\n",
        "# horus doctor and horus self update read it. Do not hand-edit.\n",
    );
    let rendered = format!(
        "{header}{}",
        toml::to_string_pretty(manifest).context("rendering install_manifest.toml")?
    );
    std::fs::write(&manifest_file, rendered)
        .with_context(|| format!("writing {}", manifest_file.display()))?;
    Ok(())
}

/// Where the install state files live: `$HORUS_PREFIX`, else `~/.horus`.
///
/// Resolved by version.rs so that the writer and the readers cannot disagree.
/// They did: this wrote to ~/.horus unconditionally while install.sh:245-247
/// put a HORUS_PREFIX install's state in the prefix, so `horus self update` on
/// a system-wide install updated the binary and then recorded that fact in a
/// directory nothing on that machine reads — including the next `self update`.
fn horus_home() -> Result<PathBuf> {
    crate::version::state_root()
}

/// `<state root>/cache` — where install.sh puts the source tree and `horus
/// clean` and uninstall.sh manage it.
fn cache_root() -> Result<PathBuf> {
    crate::version::cache_root()
}

/// The binary this process is running from: the one that has to be replaced.
///
/// `cargo install`'s destination is only a guess — an install into
/// /usr/local/bin or under HORUS_PREFIX is not in ~/.cargo/bin, and replacing
/// the wrong copy would leave the user on the old CLI while `self update`
/// reported success. Symlinks are resolved so the real file is replaced rather
/// than the link.
fn current_binary_path() -> Result<PathBuf> {
    let exe = std::env::current_exe()
        .ok()
        .and_then(|p| std::fs::canonicalize(&p).ok().or(Some(p)))
        .or_else(|| cargo_bin_dir().map(|d| d.join(binary_file_name())))
        .ok_or_else(|| anyhow!("could not determine the path of the running horus binary"))?;
    Ok(exe)
}

/// Ask the horus binary at `bin` which version it is. `None` when it cannot be
/// found or run.
fn installed_horus_version(bin: &Path) -> Option<String> {
    if !bin.exists() {
        return None;
    }
    let output = Command::new(bin).arg("--version").output().ok()?;
    if !output.status.success() {
        return None;
    }
    // clap prints "horus <version>".
    String::from_utf8_lossy(&output.stdout)
        .split_whitespace()
        .nth(1)
        .map(str::to_string)
}

/// The directory `cargo install` writes binaries to: `$CARGO_HOME/bin`,
/// falling back to `~/.cargo/bin`.
fn cargo_bin_dir() -> Option<PathBuf> {
    match std::env::var("CARGO_HOME") {
        Ok(home) if !home.is_empty() => Some(PathBuf::from(home).join("bin")),
        _ => dirs::home_dir().map(|h| h.join(".cargo").join("bin")),
    }
}

/// The last `n` non-empty lines of some command output, for error messages.
fn tail(text: &str, n: usize) -> String {
    let lines: Vec<&str> = text.lines().filter(|l| !l.trim().is_empty()).collect();
    lines[lines.len().saturating_sub(n)..].join("\n")
}

// ── Plugins ─────────────────────────────────────────────────────────────────

/// Check the latest version of a plugin package from the registry API.
///
/// The URL used to be `horusrobotics.dev/api/packages/<n>/latest` on the apex
/// domain, which 404s -- there is no API there and never was -- and every
/// non-200 mapped to `None`, so this function could not report an upgrade for
/// any plugin under any circumstances. It now asks the configured registry, and
/// returns `None` when there is not one.
fn check_plugin_version(name: &str) -> Option<String> {
    let base_url = crate::config::registry_url()?;
    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(5))
        .build()
        .ok()?;

    let encoded = name.replace('@', "%40").replace('/', "%2F");
    let url = format!(
        "{}/api/packages/{}/latest",
        base_url.trim_end_matches('/'),
        encoded
    );

    let resp = client.get(&url).send().ok()?;
    if !resp.status().is_success() {
        return None;
    }

    let body: serde_json::Value = resp.json().ok()?;
    body.get("version")
        .or_else(|| body.get("latest_version"))
        .and_then(|v| v.as_str())
        .map(String::from)
}

/// Load the plugin registry and collect installed plugin entries with their versions.
///
/// Returns a vec of (plugin_name, package_name, installed_version, is_global).
fn collect_plugin_entries() -> Vec<(String, String, String, bool)> {
    use crate::plugins::PluginRegistry;

    let mut entries = Vec::new();

    // Collect from global registry
    if let Ok(registry) = PluginRegistry::load_global() {
        for (cmd_name, entry) in &registry.plugins {
            entries.push((
                cmd_name.clone(),
                entry.package.clone(),
                entry.version.clone(),
                true,
            ));
        }
    }

    // Collect from local/project registry
    let cwd = std::env::current_dir().unwrap_or_else(|_| std::path::PathBuf::from("."));
    if let Some(registry) = PluginRegistry::load_project(&cwd) {
        for (cmd_name, entry) in &registry.plugins {
            // Skip if already found in global (global takes precedence for upgrade)
            if !entries.iter().any(|(name, _, _, _)| name == cmd_name) {
                entries.push((
                    cmd_name.clone(),
                    entry.package.clone(),
                    entry.version.clone(),
                    false,
                ));
            }
        }
    }

    entries
}

/// Check for plugin updates (display only, no install).
fn check_plugin_updates() {
    let entries = collect_plugin_entries();
    if entries.is_empty() {
        return;
    }

    println!();
    println!("  {}", "Checking plugin versions:".bold());
    for (cmd_name, package_name, installed_version, _is_global) in &entries {
        match check_plugin_version(package_name) {
            Some(latest) if latest != *installed_version => {
                println!(
                    "    {} {} ({}) {} -> {}",
                    "!".yellow(),
                    cmd_name,
                    package_name,
                    installed_version.dimmed(),
                    latest.green()
                );
            }
            Some(_) => {
                println!(
                    "    {} {} ({}) up to date",
                    "*".green(),
                    cmd_name,
                    installed_version.dimmed()
                );
            }
            None => {
                println!("    {} {} could not check version", "?".dimmed(), cmd_name);
            }
        }
    }
}

/// Upgrade all installed plugins to their latest versions.
fn upgrade_plugins() {
    let entries = collect_plugin_entries();
    if entries.is_empty() {
        return;
    }

    println!();
    println!("  {}", "Upgrading plugins:".bold());
    for (cmd_name, package_name, installed_version, is_global) in &entries {
        match check_plugin_version(package_name) {
            Some(latest) if latest != *installed_version => {
                println!(
                    "    {} Upgrading {} {} -> {}...",
                    "!".yellow(),
                    cmd_name,
                    installed_version.dimmed(),
                    latest.green()
                );

                match reinstall_plugin(package_name, &latest, *is_global) {
                    Ok(()) => {
                        println!(
                            "    {} {} upgraded to {}",
                            "*".green(),
                            cmd_name,
                            latest.green()
                        );
                    }
                    Err(e) => {
                        println!("    {} Failed to upgrade {}: {}", "!".yellow(), cmd_name, e);
                    }
                }
            }
            Some(_) => {
                println!(
                    "    {} {} already up to date ({})",
                    "*".green(),
                    cmd_name,
                    installed_version.dimmed()
                );
            }
            None => {
                println!(
                    "    {} {} could not determine latest version — skipping",
                    "?".dimmed(),
                    cmd_name
                );
            }
        }
    }
}

/// Reinstall a plugin at a specific version using the registry client.
fn reinstall_plugin(package_name: &str, version: &str, global: bool) -> Result<()> {
    use crate::plugins::PluginSource;
    use crate::{registry, workspace};

    let install_target = if global {
        workspace::InstallTarget::Global
    } else {
        workspace::InstallTarget::Local(std::env::current_dir()?)
    };

    let client = registry::RegistryClient::new()?;
    let installed_version =
        client.install_to_target(package_name, Some(version), install_target)?;

    // Re-register the plugin after reinstall
    if let Some(pkg_dir) =
        super::pkg::resolve_installed_package_dir(package_name, &installed_version, global)
    {
        let project_root = if global {
            None
        } else {
            Some(std::env::current_dir()?)
        };
        if let Err(e) = super::pkg::register_plugin_after_install(
            &pkg_dir,
            PluginSource::Registry,
            global,
            project_root.as_deref(),
        ) {
            log::warn!("Plugin re-registration failed: {}", e);
        }
    }

    Ok(())
}

/// List installed plugins from the global and local registries.
pub(crate) fn list_installed_plugins() -> Vec<String> {
    let mut plugins = Vec::new();

    // Check global plugins
    if let Some(home) = dirs::home_dir() {
        let global = home.join(".horus/plugins");
        if let Ok(entries) = std::fs::read_dir(global) {
            for entry in entries.flatten() {
                if let Some(name) = entry.file_name().to_str() {
                    plugins.push(format!("{} (global)", name));
                }
            }
        }
    }

    // Check local plugins
    let local = std::path::Path::new(".horus/plugins");
    if let Ok(entries) = std::fs::read_dir(local) {
        for entry in entries.flatten() {
            if let Some(name) = entry.file_name().to_str() {
                plugins.push(format!("{} (local)", name));
            }
        }
    }

    plugins
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Resolving the latest release ────────────────────────────────────
    //
    // These replace two tests that called check_latest_version() against the
    // registry and asserted the failure was fine — `Ok(None)` when the endpoint
    // 404s, which is the bug rather than the contract. The responses are fed in
    // directly so the failure modes are covered without a network.

    #[test]
    fn a_v_prefixed_tag_parses() {
        let release = parse_release_response(200, r#"{"tag_name": "v0.4.0"}"#).unwrap();
        assert_eq!(release.tag, "v0.4.0");
        assert_eq!(
            release.version, "0.4.0",
            "the v must be stripped for semver"
        );
    }

    #[test]
    fn a_bare_tag_parses() {
        // Not every tag in the wild carries the v; both must resolve to the
        // same comparable version.
        let release = parse_release_response(200, r#"{"tag_name": "0.4.0"}"#).unwrap();
        assert_eq!(release.tag, "0.4.0");
        assert_eq!(release.version, "0.4.0");
    }

    #[test]
    fn a_prerelease_tag_parses() {
        let release = parse_release_response(200, r#"{"tag_name": "v0.5.0-rc.1"}"#).unwrap();
        assert_eq!(release.version, "0.5.0-rc.1");
    }

    #[test]
    fn rate_limiting_is_an_error_not_up_to_date() {
        // The whole point. GitHub answers an over-limit unauthenticated caller
        // with 403; the old `_ => Ok(None)` reported that as "could not
        // determine latest version" and exited 0.
        let body = r#"{"message": "API rate limit exceeded for 203.0.113.7.",
                       "documentation_url": "https://docs.github.com/rest"}"#;
        let err = parse_release_response(403, body)
            .expect_err("a rate-limited check must not look like a successful one");
        let msg = err.to_string();
        assert!(
            msg.contains("403"),
            "the status belongs in the message: {msg}"
        );
        assert!(
            msg.contains("rate limit"),
            "the reason GitHub gave belongs in the message: {msg}"
        );
    }

    #[test]
    fn a_missing_user_agent_403_is_an_error() {
        // Same status, different cause — GitHub rejects an API request with no
        // User-Agent. Both are failures, neither is "no update".
        parse_release_response(
            403,
            r#"{"message": "Request forbidden by administrative rules."}"#,
        )
        .expect_err("403 must be reported");
    }

    #[test]
    fn server_errors_are_reported() {
        parse_release_response(503, "suspend-by-user").expect_err("5xx must be reported");
        parse_release_response(404, "{}").expect_err("404 must be reported");
    }

    #[test]
    fn a_response_without_a_tag_is_an_error() {
        parse_release_response(200, r#"{"name": "0.4.0"}"#)
            .expect_err("no tag_name means nothing was resolved");
        parse_release_response(200, "<html>not json</html>")
            .expect_err("a non-JSON body means nothing was resolved");
    }

    #[test]
    fn a_non_semver_tag_is_an_error() {
        // A `nightly` or `latest` tag cannot be compared against
        // CARGO_PKG_VERSION, so it must not reach the comparison.
        parse_release_response(200, r#"{"tag_name": "nightly"}"#)
            .expect_err("a tag that is not a version must be rejected");
    }

    #[test]
    fn error_chain_keeps_the_cause_a_reqwest_error_hides() {
        #[derive(Debug)]
        struct Inner;
        impl std::fmt::Display for Inner {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "connection refused")
            }
        }
        impl std::error::Error for Inner {}

        #[derive(Debug)]
        struct Outer(Inner);
        impl std::fmt::Display for Outer {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "error sending request")
            }
        }
        impl std::error::Error for Outer {
            fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
                Some(&self.0)
            }
        }

        assert_eq!(
            error_chain(&Outer(Inner)),
            "error sending request: connection refused"
        );
    }

    // ── validate_release ────────────────────────────────────────────────

    fn release(tag: &str, version: &str) -> LatestRelease {
        LatestRelease {
            tag: tag.to_string(),
            version: version.to_string(),
        }
    }

    #[test]
    fn upgrade_horus_rejects_non_semver_version() {
        // Regression: `version` comes from a JSON body and is passed to cargo
        // and printed back as what was installed, so it is validated before
        // anything is executed. The empty string used to reach the build step
        // and return Ok.
        upgrade_horus(&release("v", "")).expect_err("empty version must be rejected");
        upgrade_horus(&release("v--path", "--path"))
            .expect_err("a version that looks like a cargo flag must be rejected");
    }

    #[test]
    fn upgrade_horus_rejects_a_tag_that_is_not_a_tag() {
        // The tag reaches `git clone --branch <tag>`. A leading dash would be
        // read as a flag, and a slash would escape the cache directory that
        // install_source_cache removes.
        upgrade_horus(&release("--upload-pack=touch /tmp/pwned", "0.4.0"))
            .expect_err("a tag that looks like a git flag must be rejected");
        upgrade_horus(&release("../../etc", "0.4.0"))
            .expect_err("a tag with path separators must be rejected");
        upgrade_horus(&release("", "0.4.0")).expect_err("an empty tag must be rejected");
    }

    #[test]
    fn validate_release_accepts_a_real_release() {
        validate_release(&release("v0.4.0", "0.4.0")).unwrap();
        validate_release(&release("0.4.0", "0.4.0")).unwrap();
    }

    // ── Asset naming ────────────────────────────────────────────────────

    #[test]
    fn the_asset_name_matches_what_install_sh_downloads() {
        // install.sh builds "horus-${OS}-${ARCH}.${EXT}" from detect_os and
        // detect_arch. If these two ever disagree the installer and the updater
        // fetch different files for the same machine.
        if !matches!(std::env::consts::OS, "linux" | "macos" | "windows") {
            return;
        }
        let name = platform_asset_name().expect("a released platform must name an asset");
        let expected_ext = if cfg!(windows) { ".zip" } else { ".tar.gz" };
        assert!(
            name.ends_with(expected_ext),
            "{name} should end with {expected_ext}"
        );
        assert!(
            name.starts_with("horus-"),
            "{name} should start with horus-"
        );
        let stem = name.trim_end_matches(expected_ext);
        let (os, arch) = stem
            .trim_start_matches("horus-")
            .split_once('-')
            .expect("asset stem is horus-<os>-<arch>");
        assert!(
            ["linux", "macos", "windows"].contains(&os),
            "unexpected os segment {os} in {name}"
        );
        assert!(
            ["amd64", "arm64", "armv7"].contains(&arch),
            "unexpected arch segment {arch} in {name}"
        );
    }

    // ── Checksum verification ───────────────────────────────────────────

    fn sha256_hex(bytes: &[u8]) -> String {
        use sha2::{Digest, Sha256};
        let mut hasher = Sha256::new();
        hasher.update(bytes);
        hex::encode(hasher.finalize())
    }

    /// A one-entry tar.gz holding a stand-in `horus` binary, shaped like the
    /// release asset (the binary at the archive root, as release.yml packs it).
    fn fake_asset(contents: &[u8]) -> Vec<u8> {
        let mut header = tar::Header::new_gnu();
        header.set_size(contents.len() as u64);
        header.set_mode(0o755);
        header.set_cksum();

        let mut builder = tar::Builder::new(flate2::write::GzEncoder::new(
            Vec::new(),
            flate2::Compression::fast(),
        ));
        builder
            .append_data(&mut header, binary_file_name(), contents)
            .unwrap();
        builder.into_inner().unwrap().finish().unwrap()
    }

    #[test]
    fn checksum_mismatch_aborts_without_replacing_the_binary() {
        let tmp = tempfile::tempdir().unwrap();
        let target = tmp.path().join("horus");
        std::fs::write(&target, b"the currently installed binary").unwrap();

        let archive = fake_asset(b"the new binary");
        let asset = "horus-linux-amd64.tar.gz";
        // A digest of something else entirely: a substituted or truncated
        // asset.
        let sums = format!("{}  {asset}\n", sha256_hex(b"not this archive"));

        let stage = tmp.path().join("stage");
        std::fs::create_dir_all(&stage).unwrap();
        let err = install_verified_asset(&archive, asset, &sums, &stage, &target)
            .expect_err("a digest mismatch must abort the install");
        assert!(
            err.to_string().contains("MISMATCH"),
            "the error should say what failed: {err}"
        );
        assert_eq!(
            std::fs::read(&target).unwrap(),
            b"the currently installed binary",
            "the installed binary must be untouched after a failed verification"
        );
    }

    #[test]
    fn a_verified_asset_replaces_the_binary() {
        let tmp = tempfile::tempdir().unwrap();
        let target = tmp.path().join("horus");
        std::fs::write(&target, b"the currently installed binary").unwrap();

        let archive = fake_asset(b"the new binary");
        let asset = "horus-linux-amd64.tar.gz";
        let sums = format!(
            "0000000000000000000000000000000000000000000000000000000000000000  horus-macos-arm64.tar.gz\n\
             {}  {asset}\n",
            sha256_hex(&archive)
        );

        let stage = tmp.path().join("stage");
        std::fs::create_dir_all(&stage).unwrap();
        install_verified_asset(&archive, asset, &sums, &stage, &target).unwrap();
        assert_eq!(std::fs::read(&target).unwrap(), b"the new binary");
    }

    #[test]
    fn sha256sums_without_an_entry_for_this_asset_is_fatal() {
        // install.sh:294-297 refuses here rather than installing unverified;
        // so does this. An asset the release does not vouch for is not an
        // asset to run.
        let archive = fake_asset(b"whatever");
        let err = verify_digest(
            &archive,
            "horus-linux-amd64.tar.gz",
            "abc123  horus-windows-amd64.zip\n",
        )
        .expect_err("no entry for our asset must be fatal");
        assert!(err.to_string().contains("no entry"), "{err}");
    }

    #[test]
    fn verify_digest_reads_binary_mode_sha256sums() {
        // `sha256sum -b` writes "<digest> *<name>"; the star is not part of the
        // file name.
        let archive = fake_asset(b"whatever");
        let sums = format!("{} *horus-linux-amd64.tar.gz\n", sha256_hex(&archive));
        verify_digest(&archive, "horus-linux-amd64.tar.gz", &sums).unwrap();
    }

    #[test]
    #[cfg(unix)]
    fn an_unwritable_install_dir_names_the_directory_and_the_sudo_form() {
        use std::os::unix::fs::PermissionsExt;

        let tmp = tempfile::tempdir().unwrap();
        let bin_dir = tmp.path().join("bin");
        std::fs::create_dir_all(&bin_dir).unwrap();
        std::fs::set_permissions(&bin_dir, std::fs::Permissions::from_mode(0o555)).unwrap();

        // root ignores the mode bits, and so do a few filesystems, so there is
        // nothing to observe in those environments.
        if preflight_writable(&bin_dir.join("horus")).is_ok() {
            std::fs::set_permissions(&bin_dir, std::fs::Permissions::from_mode(0o755)).unwrap();
            return;
        }

        let err = preflight_writable(&bin_dir.join("horus"))
            .expect_err("an unwritable install directory must be refused up front");
        let msg = err.to_string();
        // "Permission denied (os error 13)" is what this used to be, and it
        // names neither the directory nor a way out.
        assert!(
            msg.contains(&bin_dir.display().to_string()),
            "the message must name the directory: {msg}"
        );
        assert!(
            msg.contains("sudo"),
            "the message must offer the sudo form: {msg}"
        );

        // Let the tempdir clean itself up.
        std::fs::set_permissions(&bin_dir, std::fs::Permissions::from_mode(0o755)).unwrap();
    }

    // ── Where the state goes ────────────────────────────────────────────

    #[test]
    fn state_and_cache_roots_follow_horus_prefix() {
        // Both roots are resolved by version.rs so the writer here and the
        // readers there cannot disagree. They did: this wrote ~/.horus while
        // install.sh:245-247 put a prefix install's state in the prefix, so
        // `self update` recorded the new version where nothing reads it.
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let previous = std::env::var_os("HORUS_PREFIX");
        std::env::set_var("HORUS_PREFIX", "/opt/horus");
        let home = horus_home();
        let cache = cache_root();
        match previous {
            Some(previous) => std::env::set_var("HORUS_PREFIX", previous),
            None => std::env::remove_var("HORUS_PREFIX"),
        }

        assert_eq!(home.unwrap(), PathBuf::from("/opt/horus"));
        assert_eq!(cache.unwrap(), PathBuf::from("/opt/horus/cache"));
    }

    #[test]
    fn state_and_cache_roots_default_to_dot_horus() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let previous = std::env::var_os("HORUS_PREFIX");
        std::env::remove_var("HORUS_PREFIX");
        let home = horus_home();
        let cache = cache_root();
        if let Some(previous) = previous {
            std::env::set_var("HORUS_PREFIX", previous);
        }

        let home = home.unwrap();
        assert_eq!(home, crate::paths::home_dir().unwrap().join(".horus"));
        assert_eq!(cache.unwrap(), home.join("cache"));
    }

    // ── Install state ───────────────────────────────────────────────────

    #[test]
    fn install_state_is_written_in_the_format_version_rs_reads() {
        let tmp = tempfile::tempdir().unwrap();
        let home = tmp.path().join(".horus");
        write_install_state(
            &home,
            &InstallManifest {
                version: "0.4.1".to_string(),
                tag: "v0.4.1".to_string(),
                commit: Some("a".repeat(40)),
                topic_version: Some(4),
                source_dir: "/home/u/.horus/cache/horus@0.4.1".to_string(),
                binary: "/home/u/.cargo/bin/horus".to_string(),
                install_method: "self-update".to_string(),
                installed_at: "2026-08-31T00:00:00Z".to_string(),
            },
        )
        .unwrap();

        // version.rs:32-44 reads this with read_to_string().trim(), so it is a
        // bare version and nothing else.
        let raw = std::fs::read_to_string(home.join("installed_version")).unwrap();
        assert_eq!(raw, "0.4.1\n");
        assert_eq!(raw.trim(), "0.4.1");

        let manifest: toml::Table =
            toml::from_str(&std::fs::read_to_string(home.join("install_manifest.toml")).unwrap())
                .expect("the manifest must be parseable TOML");
        assert_eq!(manifest["version"].as_str(), Some("0.4.1"));
        assert_eq!(manifest["tag"].as_str(), Some("v0.4.1"));
        assert_eq!(manifest["topic_version"].as_integer(), Some(4));
        assert_eq!(manifest["install_method"].as_str(), Some("self-update"));
        assert_eq!(
            manifest["source_dir"].as_str(),
            Some("/home/u/.horus/cache/horus@0.4.1")
        );
    }

    #[test]
    fn install_state_omits_fields_it_could_not_determine() {
        // git or the header file may be missing; the manifest records what is
        // known rather than writing a zero that doctor.rs would compare.
        let tmp = tempfile::tempdir().unwrap();
        let home = tmp.path().join(".horus");
        write_install_state(
            &home,
            &InstallManifest {
                version: "0.4.1".to_string(),
                tag: "v0.4.1".to_string(),
                commit: None,
                topic_version: None,
                source_dir: "/tmp/src".to_string(),
                binary: "/tmp/bin/horus".to_string(),
                install_method: "self-update".to_string(),
                installed_at: "2026-08-31T00:00:00Z".to_string(),
            },
        )
        .unwrap();
        let text = std::fs::read_to_string(home.join("install_manifest.toml")).unwrap();
        assert!(!text.contains("commit"), "{text}");
        assert!(!text.contains("topic_version"), "{text}");
        toml::from_str::<toml::Table>(&text).expect("still valid TOML without the optional fields");
    }

    #[test]
    fn a_windows_style_path_survives_the_manifest() {
        // Backslashes are TOML escape characters; an unescaped Windows path
        // would produce a manifest that doctor.rs cannot parse.
        let tmp = tempfile::tempdir().unwrap();
        let home = tmp.path().join(".horus");
        write_install_state(
            &home,
            &InstallManifest {
                version: "0.4.1".to_string(),
                tag: "v0.4.1".to_string(),
                commit: None,
                topic_version: None,
                source_dir: r"C:\Users\u\.horus\cache\horus@0.4.1".to_string(),
                binary: r"C:\Users\u\.cargo\bin\horus.exe".to_string(),
                install_method: "self-update".to_string(),
                installed_at: "2026-08-31T00:00:00Z".to_string(),
            },
        )
        .unwrap();
        let manifest: toml::Table =
            toml::from_str(&std::fs::read_to_string(home.join("install_manifest.toml")).unwrap())
                .expect("backslashes must be escaped, not emitted raw");
        assert_eq!(
            manifest["source_dir"].as_str(),
            Some(r"C:\Users\u\.horus\cache\horus@0.4.1")
        );
    }

    #[test]
    fn topic_version_is_read_out_of_the_fetched_tree() {
        // TOPIC_VERSION is pub(crate) in horus_core, so it is grepped rather
        // than linked. This is the shape the constant has today
        // (horus_core/src/communication/topic/header.rs:66).
        let tmp = tempfile::tempdir().unwrap();
        let header = tmp.path().join("horus_core/src/communication/topic");
        std::fs::create_dir_all(&header).unwrap();
        std::fs::write(
            header.join("header.rs"),
            "pub(crate) const TOPIC_MAGIC: u32 = 1;\npub(crate) const TOPIC_VERSION: u32 = 4;\n",
        )
        .unwrap();
        assert_eq!(topic_version_of(tmp.path()), Some(4));
    }

    #[test]
    fn topic_version_is_absent_rather_than_wrong() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(topic_version_of(tmp.path()), None);
    }

    // ── Building from source ────────────────────────────────────────────

    #[test]
    fn build_from_source_refuses_a_tree_that_is_not_horus() {
        // Kept from the cwd-build regression: the build must come from a tree
        // that really is horus_manager, checked before cargo is run. It used to
        // be possible to `cargo install --path .` the user's own project.
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src-tree");
        std::fs::create_dir_all(src.join("horus_manager")).unwrap();
        std::fs::write(
            src.join("horus_manager/Cargo.toml"),
            "[package]\nname = \"totally-not-horus\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let err = build_from_source(&src, tmp.path())
            .expect_err("a tree without the horus_manager package must be refused");
        assert!(
            err.to_string().contains("horus_manager"),
            "the error should name the package it wanted: {err}"
        );
    }

    #[test]
    fn build_from_source_reports_a_missing_manifest() {
        let tmp = tempfile::tempdir().unwrap();
        let err = build_from_source(&tmp.path().join("src-tree"), tmp.path())
            .expect_err("a tree with no manifest at all must be refused");
        assert!(
            err.to_string().contains("Cargo.toml"),
            "the error should name the manifest it could not read: {err}"
        );
    }

    // ── list_installed_plugins ──────────────────────────────────────────

    #[test]
    fn list_installed_plugins_returns_vec() {
        // Should return a Vec (possibly empty) without errors
        let plugins = list_installed_plugins();
        // Just verify it's a vec and doesn't panic
        let _ = plugins.len();
    }

    #[test]
    fn list_installed_plugins_no_local_plugins_dir() {
        // When there's no .horus/plugins in cwd, local plugins should be empty
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        // Should have no local plugins (global might exist from real home)
        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert!(local_plugins.is_empty());
    }

    #[test]
    fn list_installed_plugins_finds_local_plugins() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Create .horus/plugins with some entries
        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("my-plugin"), "").unwrap();
        std::fs::write(plugin_dir.join("another-plugin"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 2);
        assert!(
            local_plugins.iter().any(|p| p.contains("my-plugin")),
            "Should find my-plugin in {:?}",
            local_plugins,
        );
        assert!(
            local_plugins.iter().any(|p| p.contains("another-plugin")),
            "Should find another-plugin in {:?}",
            local_plugins,
        );
    }

    #[test]
    fn list_installed_plugins_formats_local_entries() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("test-plugin"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert!(
            local_plugins.iter().any(|p| *p == "test-plugin (local)"),
            "Should format as 'name (local)', got {:?}",
            local_plugins,
        );
    }

    #[test]
    fn list_installed_plugins_finds_directories_as_plugins() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Plugin entries can be directories too
        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(plugin_dir.join("dir-plugin")).unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 1);
        assert!(local_plugins[0].contains("dir-plugin"));
    }

    #[test]
    fn list_installed_plugins_empty_plugin_dir() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Create empty .horus/plugins
        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert!(local_plugins.is_empty());
    }

    #[test]
    fn list_installed_plugins_global_format() {
        // If any global plugins exist, they should contain "(global)"
        let plugins = list_installed_plugins();
        for plugin in &plugins {
            assert!(
                plugin.contains("(global)") || plugin.contains("(local)"),
                "Every plugin string should be tagged (global) or (local), got: {}",
                plugin,
            );
        }
    }

    // ── Edge cases ──────────────────────────────────────────────────────

    #[test]
    fn list_installed_plugins_handles_non_utf8_gracefully() {
        // The function uses to_str() which filters out non-UTF8 names.
        // On Linux we can create non-UTF8 filenames, but for portability
        // we just verify the function doesn't panic with normal filenames
        // containing special characters.
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("plugin-with-dashes"), "").unwrap();
        std::fs::write(plugin_dir.join("plugin_with_underscores"), "").unwrap();
        std::fs::write(plugin_dir.join("plugin.with.dots"), "").unwrap();
        std::fs::write(plugin_dir.join("UPPERCASE"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 4);
    }

    #[test]
    fn list_installed_plugins_many_entries() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        for i in 0..20 {
            std::fs::write(plugin_dir.join(format!("plugin-{}", i)), "").unwrap();
        }

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 20);
    }

    #[test]
    fn current_version_is_valid_semver() {
        let version = env!("CARGO_PKG_VERSION");
        assert!(
            semver::Version::parse(version).is_ok(),
            "CARGO_PKG_VERSION should be valid semver: {}",
            version,
        );
    }

    // The three `run_upgrade(false)` tests that used to live here are gone on
    // purpose. They asserted Ok on the strength of the update never happening;
    // now that it does happen, a unit test that called it would download a
    // release and overwrite the developer's installed horus. `run_upgrade(true)`
    // is still safe — it only reads — and every decision it makes is covered
    // above without a network.
}

#[cfg(test)]
mod github_token_tests {
    use super::github_token;

    /// The env vars this reads are process-global and libtest runs tests as
    /// threads of one process, so these run under one lock and restore what
    /// they found. Without that, a sibling test observing GITHUB_TOKEN sees
    /// whatever this set.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    struct EnvGuard {
        saved: Vec<(&'static str, Option<String>)>,
        _lock: std::sync::MutexGuard<'static, ()>,
    }

    impl EnvGuard {
        fn new() -> Self {
            let lock = ENV_LOCK.lock().unwrap_or_else(|e| e.into_inner());
            let saved = ["GITHUB_TOKEN", "GH_TOKEN"]
                .iter()
                .map(|k| (*k, std::env::var(k).ok()))
                .collect();
            for k in ["GITHUB_TOKEN", "GH_TOKEN"] {
                std::env::remove_var(k);
            }
            Self { saved, _lock: lock }
        }
    }

    impl Drop for EnvGuard {
        fn drop(&mut self) {
            for (k, v) in &self.saved {
                match v {
                    Some(val) => std::env::set_var(k, val),
                    None => std::env::remove_var(k),
                }
            }
        }
    }

    #[test]
    fn no_token_in_the_environment_means_an_anonymous_request() {
        let _g = EnvGuard::new();
        assert_eq!(
            github_token(),
            None,
            "reading public releases needs no credentials; absent must stay absent"
        );
    }

    #[test]
    fn github_token_is_used_when_set() {
        let _g = EnvGuard::new();
        std::env::set_var("GITHUB_TOKEN", "ghp_example");
        assert_eq!(github_token().as_deref(), Some("ghp_example"));
    }

    #[test]
    fn gh_token_is_the_fallback_the_gh_cli_sets() {
        let _g = EnvGuard::new();
        std::env::set_var("GH_TOKEN", "gho_example");
        assert_eq!(github_token().as_deref(), Some("gho_example"));
    }

    #[test]
    fn github_token_wins_over_gh_token() {
        let _g = EnvGuard::new();
        std::env::set_var("GITHUB_TOKEN", "first");
        std::env::set_var("GH_TOKEN", "second");
        assert_eq!(github_token().as_deref(), Some("first"));
    }

    /// An empty or whitespace-only value must read as absent.
    ///
    /// Actions sets `GITHUB_TOKEN` to the empty string in some configurations,
    /// and sending `Authorization: Bearer ` gets a 401 from GitHub — turning
    /// "no token" into a hard failure instead of the anonymous request that
    /// would have worked.
    #[test]
    fn a_blank_token_is_treated_as_absent() {
        let _g = EnvGuard::new();
        std::env::set_var("GITHUB_TOKEN", "");
        assert_eq!(github_token(), None);
        std::env::set_var("GITHUB_TOKEN", "   ");
        assert_eq!(github_token(), None);
    }

    #[test]
    fn a_blank_github_token_falls_through_to_gh_token() {
        let _g = EnvGuard::new();
        std::env::set_var("GITHUB_TOKEN", "");
        std::env::set_var("GH_TOKEN", "real");
        assert_eq!(github_token().as_deref(), Some("real"));
    }
}
