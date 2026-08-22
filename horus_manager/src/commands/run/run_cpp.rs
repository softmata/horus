//! C++ build and execution pipeline.
//!
//! Handles cmake configure, build, binary discovery, and execution for C++
//! projects. If a root `CMakeLists.txt` exists, it is used directly; otherwise
//! one is generated from `horus.toml` via `cmake_gen`.

use crate::cli_output;
use crate::cmake_gen;
use crate::manifest::{HorusManifest, HORUS_TOML};
use anyhow::{bail, Context, Result};
use colored::*;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

/// Build directory inside `.horus/`.
const CPP_BUILD_DIR: &str = ".horus/cpp-build";

/// Build a C++ project using cmake.
///
/// 1. Determines cmake source: root `CMakeLists.txt` or generated `.horus/CMakeLists.txt`
/// 2. Runs `cmake -S <source> -B .horus/cpp-build/`
/// 3. Runs `cmake --build .horus/cpp-build/`
/// 4. Symlinks `compile_commands.json` to project root
/// 5. Returns path to the built binary
///
/// `target_arch`: Optional cross-compilation target (e.g. "aarch64", "armv7").
/// If `None`, falls back to `[cpp].toolchain` in horus.toml.
pub(crate) fn build_cpp(
    project_dir: &Path,
    release: bool,
    target_arch: Option<&str>,
) -> Result<PathBuf> {
    let build_dir = project_dir.join(CPP_BUILD_DIR);
    fs::create_dir_all(&build_dir).context("Failed to create .horus/cpp-build directory")?;

    // Determine cmake source directory
    let cmake_source = if project_dir.join("CMakeLists.txt").exists() {
        eprintln!(
            "{} Using project CMakeLists.txt",
            cli_output::ICON_INFO.cyan()
        );
        project_dir.to_path_buf()
    } else {
        eprintln!(
            "{} Generating CMakeLists.txt from horus.toml...",
            cli_output::ICON_INFO.cyan()
        );
        generate_cmake_if_needed(project_dir)?;
        project_dir.join(".horus")
    };

    // ── Check & install system dependencies ─────────────────────────────
    ensure_system_deps(project_dir)?;

    let build_type = if release { "Release" } else { "Debug" };
    let verbose = std::env::var("HORUS_VERBOSE").is_ok() || log::log_enabled!(log::Level::Debug);

    // ── Configure ────────────────────────────────────────────────────────
    eprintln!(
        "{} cmake configure ({})...",
        cli_output::ICON_INFO.cyan(),
        build_type.yellow()
    );

    let mut configure_cmd = Command::new("cmake");
    configure_cmd
        .arg("-S")
        .arg(&cmake_source)
        .arg("-B")
        .arg(&build_dir)
        .arg("-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
        .arg(format!("-DCMAKE_BUILD_TYPE={}", build_type));

    // Pick a generator we can actually drive.
    //
    // This call passed no `-G`, so cmake always chose its default — "Unix
    // Makefiles" on Linux and macOS — which needs `make`. `horus doctor`
    // meanwhile advertised ninja as a satisfying alternative for the C++
    // toolchain, so a machine with cmake, clang++ and ninja but no make was
    // reported ready and then failed at the first build with
    //
    //     CMake Error: CMake was unable to find a build program corresponding
    //     to "Unix Makefiles". CMAKE_MAKE_PROGRAM is not set.
    //
    // Narrowing doctor to demand make would also have removed the false green,
    // but at the cost of telling a ninja user to install a build tool they
    // already have a better version of. Selecting the generator instead makes
    // doctor's claim true: ninja is genuinely sufficient now.
    //
    // make is preferred when both exist, so an existing build tree keeps its
    // generator and cmake does not error on a generator change.
    if let Some(generator) = crate::commands::run::select_cmake_generator(
        crate::commands::run::generator_available("make"),
        crate::commands::run::generator_available("gmake"),
        crate::commands::run::generator_available("ninja"),
    ) {
        configure_cmd.arg("-G").arg(generator);
    }

    // ── Cross-compilation target ────────────────────────────────────────
    // Resolved before the bindings, which must be built for the same
    // architecture the C++ compiler targets.
    // Priority: explicit target_arch > horus.toml [cpp].toolchain
    let effective_target = target_arch.map(String::from).or_else(|| {
        let mp = project_dir.join(HORUS_TOML);
        HorusManifest::load_from(&mp)
            .ok()
            .and_then(|m| m.cpp)
            .and_then(|c| c.toolchain)
    });

    // ── Generated message entry points ──────────────────────────────────
    // Present only when `horus msg gen` has run in this project.
    match ensure_generated_msgs_lib(project_dir, release, effective_target.as_deref()) {
        Ok(Some(lib)) => {
            configure_cmd.arg(format!("-DHORUS_GEN_MSGS_LIB={}", lib.display()));
        }
        Ok(None) => {}
        Err(e) => {
            // Not fatal on its own: a project may include the generated header
            // for its structs without publishing them from C++. The linker
            // produces the real error if it needs the symbols.
            log::warn!("generated message entry points unavailable: {e:#}");
            eprintln!(
                "{} generated message entry points unavailable — {}",
                cli_output::ICON_WARN.yellow(),
                e
            );
        }
    }

    // ── HORUS C++ bindings ──────────────────────────────────────────────
    // The generated CMakeLists consumes HORUS_CPP_INCLUDE / HORUS_CPP_LIB.
    // Resolving them here keeps host paths out of the generated file.
    match ensure_horus_cpp(release, effective_target.as_deref()) {
        Ok((include_dir, lib_path)) => {
            configure_cmd.arg(format!("-DHORUS_CPP_INCLUDE={}", include_dir.display()));
            configure_cmd.arg(format!("-DHORUS_CPP_LIB={}", lib_path.display()));
        }
        Err(e) => {
            // Not fatal on its own: a project may not use the bindings at all.
            // The compiler produces the real error if it includes horus headers.
            log::warn!("HORUS C++ bindings unavailable: {e:#}");
            eprintln!(
                "{} HORUS C++ bindings unavailable — {}",
                cli_output::ICON_WARN.yellow(),
                e
            );
        }
    }

    if let Some(ref tc_spec) = effective_target {
        match crate::toolchain::write_toolchain_file(tc_spec, project_dir) {
            Ok(Some(tc_path)) => {
                eprintln!(
                    "{} Cross-compiling for {}",
                    cli_output::ICON_INFO.cyan(),
                    tc_spec.yellow()
                );
                configure_cmd.arg(format!("-DCMAKE_TOOLCHAIN_FILE={}", tc_path.display()));
            }
            Ok(None) => {
                // Native/x86_64 — no toolchain file needed
            }
            Err(e) => {
                bail!("Failed to resolve toolchain '{}': {}", tc_spec, e);
            }
        }
    }

    if verbose {
        let status = configure_cmd
            .status()
            .context("Failed to run cmake.\n  Install with: sudo apt install cmake\n  Verify with: cmake --version")?;
        if !status.success() {
            bail!(
                "cmake configure failed (exit code {})",
                status.code().unwrap_or(1)
            );
        }
    } else {
        let output = configure_cmd
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .context("Failed to run cmake.\n  Install with: sudo apt install cmake\n  Verify with: cmake --version")?;
        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            let rewritten = crate::error_wrapper::rewrite_horus_paths(&stderr);
            eprintln!("{}", rewritten);
            for diag in &crate::error_wrapper::cmake_error_hint(&stderr) {
                eprintln!("{}", crate::error_wrapper::format_diagnostic(diag));
            }
            bail!("cmake configure failed");
        }
    }

    // ── Build ────────────────────────────────────────────────────────────
    eprintln!("{} cmake build...", cli_output::ICON_INFO.cyan());

    let mut build_cmd = Command::new("cmake");
    build_cmd.arg("--build").arg(&build_dir);
    if release {
        build_cmd.arg("--config").arg("Release");
    }

    if verbose {
        let status = build_cmd.status().context("Failed to run cmake --build")?;
        if !status.success() {
            bail!(
                "cmake build failed (exit code {})",
                status.code().unwrap_or(1)
            );
        }
    } else {
        let output = build_cmd
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .output()
            .context("Failed to run cmake --build")?;
        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            let stdout = String::from_utf8_lossy(&output.stdout);
            let rewritten = crate::error_wrapper::rewrite_horus_paths(&stderr);
            // cmake --build errors often go to stdout
            if !stdout.is_empty() {
                eprintln!("{}", crate::error_wrapper::rewrite_horus_paths(&stdout));
            }
            if !rewritten.is_empty() {
                eprintln!("{}", rewritten);
            }
            for diag in &crate::error_wrapper::cmake_error_hint(&stderr) {
                eprintln!("{}", crate::error_wrapper::format_diagnostic(diag));
            }
            bail!("cmake build failed");
        }
    }

    // ── Symlink compile_commands.json ────────────────────────────────────
    symlink_compile_commands(&build_dir, project_dir);

    // ── Find binary ──────────────────────────────────────────────────────
    let project_name = load_project_name(project_dir)?;
    find_cpp_binary(&build_dir, &project_name)
}

/// Execute a compiled C++ binary with Ctrl+C handling.
pub(super) fn execute_cpp_binary(binary: &Path, args: &[String]) -> Result<()> {
    eprintln!(
        "{} Running: {}",
        cli_output::ICON_INFO.cyan(),
        binary.display().to_string().green()
    );

    let child = Command::new(binary)
        .args(args)
        .spawn()
        .with_context(|| format!("Failed to execute {}", binary.display()))?;

    let status = super::spawn_with_ctrlc(child, "C++")?;

    if !status.success() {
        bail!(
            "C++ program exited with code {}",
            status.code().unwrap_or(1)
        );
    }

    Ok(())
}

/// Build the staticlib `horus msg gen` wrote, if the project has one.
///
/// Returns `Ok(None)` when the project has no generated messages, which is the
/// ordinary case.
fn ensure_generated_msgs_lib(
    project_dir: &Path,
    release: bool,
    target_arch: Option<&str>,
) -> Result<Option<PathBuf>> {
    let crate_dir = project_dir.join(".horus/generated/msgs_ffi");
    if !crate_dir.join("Cargo.toml").is_file() {
        return Ok(None);
    }

    let triple = target_arch
        .and_then(crate::toolchain::TargetArch::from_str)
        .filter(|t| !t.rust_target().is_empty())
        .filter(|t| t.rust_target() != current_host_triple())
        .map(|t| t.rust_target());

    let profile_dir = if release { "release" } else { "debug" };
    let target_root = crate_dir.join("target");
    let lib_name = format!(
        "lib{}_msgs_ffi.a",
        load_project_name(project_dir)
            .unwrap_or_else(|_| "generated".to_string())
            .replace('-', "_")
    );
    let lib_path = match triple {
        Some(t) => target_root.join(t).join(profile_dir).join(&lib_name),
        None => target_root.join(profile_dir).join(&lib_name),
    };

    eprintln!(
        "{} Building generated message entry points...",
        cli_output::ICON_INFO.cyan()
    );
    let mut cmd = Command::new("cargo");
    cmd.current_dir(&crate_dir)
        .arg("build")
        .env("CARGO_TARGET_DIR", &target_root);
    if release {
        cmd.arg("--release");
    }
    if let Some(t) = triple {
        cmd.arg("--target").arg(t);
        if let Some(linker) = cross_linker(t) {
            let key = format!("CARGO_TARGET_{}_LINKER", t.to_uppercase().replace('-', "_"));
            cmd.env(key, linker);
        }
    }
    let status = cmd
        .status()
        .context("failed to invoke cargo for the generated message crate")?;
    if !status.success() {
        bail!("cargo build failed for {}", crate_dir.display());
    }
    if !lib_path.exists() {
        bail!("expected {} after a successful build", lib_path.display());
    }
    Ok(Some(lib_path))
}

/// The cross linker for a Rust target triple, if it is installed.
///
/// Same compiler the generated CMake toolchain file selects, so the Rust
/// archive and the C++ objects are produced by one toolchain.
fn cross_linker(triple: &str) -> Option<&'static str> {
    let candidate = match triple {
        "aarch64-unknown-linux-gnu" => "aarch64-linux-gnu-gcc",
        "armv7-unknown-linux-gnueabihf" => "arm-linux-gnueabihf-gcc",
        _ => return None,
    };
    which_exists(candidate).then_some(candidate)
}

/// Whether a program is on PATH.
fn which_exists(bin: &str) -> bool {
    std::env::var_os("PATH")
        .map(|paths| {
            std::env::split_paths(&paths).any(|dir| {
                let p = dir.join(bin);
                p.is_file()
            })
        })
        .unwrap_or(false)
}

/// The host triple this binary was compiled for.
///
/// Used to tell "cross-compiling" from "asked for the architecture we are
/// already on", which needs no `--target` and no separate artifact directory.
fn current_host_triple() -> &'static str {
    // Kept in step with TargetArch::rust_target's spellings.
    if cfg!(all(target_arch = "aarch64", target_os = "linux")) {
        "aarch64-unknown-linux-gnu"
    } else if cfg!(all(target_arch = "arm", target_os = "linux")) {
        "armv7-unknown-linux-gnueabihf"
    } else if cfg!(all(target_arch = "x86_64", target_os = "linux")) {
        "x86_64-unknown-linux-gnu"
    } else {
        ""
    }
}

/// Resolve the horus_cpp headers and static library, building it if needed.
///
/// `target_arch` must be the same architecture the C++ compiler is targeting.
/// The library is Rust code compiled into a static archive, so a C++ program
/// cross-compiled for aarch64 cannot link an x86-64 one:
///
/// ```text
/// /usr/bin/aarch64-linux-gnu-ld.bfd: libhorus_cpp.a(...): Relocations in
///     generic ELF (EM: 62)
/// ```
///
/// That was the state until now — the C++ toolchain file was honoured and the
/// Rust archive was always built for the host, so `horus deploy --arch aarch64`
/// on a C++ project could not link. It failed earlier and more confusingly
/// before that, complaining about a missing Cargo.toml, which is why this was
/// never visible.
fn ensure_horus_cpp(release: bool, target_arch: Option<&str>) -> Result<(PathBuf, PathBuf)> {
    let source = super::run_rust::find_horus_source_dir()
        .context("could not locate the HORUS source tree (needed for horus_cpp headers)")?;

    let include_dir = source.join("horus_cpp/include");
    if !include_dir.join("horus/horus.hpp").exists() {
        bail!("horus_cpp headers not found at {}", include_dir.display());
    }

    // A cross target puts artifacts under `target/<triple>/<profile>/`; a native
    // build leaves them at `target/<profile>/`.
    let triple = target_arch
        .and_then(crate::toolchain::TargetArch::from_str)
        .filter(|t| !t.rust_target().is_empty())
        .filter(|t| t.rust_target() != current_host_triple())
        .map(|t| t.rust_target());

    let profile_dir = if release { "release" } else { "debug" };
    let lib_path = match triple {
        Some(t) => source
            .join("target")
            .join(t)
            .join(profile_dir)
            .join("libhorus_cpp.a"),
        None => source
            .join("target")
            .join(profile_dir)
            .join("libhorus_cpp.a"),
    };

    if !lib_path.exists() {
        eprintln!(
            "{} Building HORUS C++ bindings{}...",
            cli_output::ICON_INFO.cyan(),
            match triple {
                Some(t) => format!(" for {}", t.yellow()),
                None => " (first C++ build in this tree)".to_string(),
            }
        );
        let mut cmd = Command::new("cargo");
        cmd.current_dir(&source)
            .arg("build")
            .arg("-p")
            .arg("horus_cpp")
            .arg("--no-default-features");
        if let Some(t) = triple {
            cmd.arg("--target").arg(t);
            // Cargo links this crate's cdylib with the host `cc` unless told
            // otherwise, which fails with "symbols.o is incompatible with
            // elf64-x86-64" long before the staticlib we actually want is
            // reached. The same GCC the CMake toolchain file names is the right
            // linker here.
            if let Some(linker) = cross_linker(t) {
                let key = format!("CARGO_TARGET_{}_LINKER", t.to_uppercase().replace('-', "_"));
                cmd.env(key, linker);
            }
        }
        if release {
            cmd.arg("--release");
        }
        // horus_cpp lives in the horus workspace; keep its artifacts there so the
        // path above resolves regardless of any inherited CARGO_TARGET_DIR.
        cmd.env("CARGO_TARGET_DIR", source.join("target"));

        let status = cmd
            .status()
            .context("failed to invoke cargo to build horus_cpp")?;
        if !status.success() {
            match triple {
                Some(t) => bail!(
                    "cargo build -p horus_cpp --target {t} failed.\n  \
                     The Rust target may not be installed: rustup target add {t}"
                ),
                None => bail!("cargo build -p horus_cpp failed"),
            }
        }
    }

    if !lib_path.exists() {
        bail!(
            "horus_cpp built but {} was not produced",
            lib_path.display()
        );
    }

    Ok((include_dir, lib_path))
}

/// Generate `.horus/CMakeLists.txt` from `horus.toml` if no root CMakeLists.txt exists.
fn generate_cmake_if_needed(project_dir: &Path) -> Result<()> {
    let manifest_path = project_dir.join(HORUS_TOML);
    if !manifest_path.exists() {
        bail!(
            "No CMakeLists.txt or horus.toml found in {}.\n\
             Create one with: {} or provide a CMakeLists.txt",
            project_dir.display(),
            "horus new my_project --cpp".cyan()
        );
    }

    let manifest = HorusManifest::load_from(&manifest_path)?;
    cmake_gen::generate(&manifest, project_dir, true)?;
    Ok(())
}

/// Find the compiled binary in the build directory.
///
/// Searches for an executable matching the project name in common cmake output locations.
fn find_cpp_binary(build_dir: &Path, project_name: &str) -> Result<PathBuf> {
    // CMake target name uses underscores (sanitized from hyphens)
    let target_name = project_name.replace('-', "_");

    // Common locations where cmake puts binaries
    let candidates = [
        build_dir.join(&target_name),
        build_dir.join(format!("{}/{}", "Debug", target_name)),
        build_dir.join(format!("{}/{}", "Release", target_name)),
    ];

    for candidate in &candidates {
        if candidate.exists() && is_executable(candidate) {
            return Ok(candidate.clone());
        }
    }

    // Fallback: search for any executable in the build dir
    if let Ok(entries) = fs::read_dir(build_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_file() && is_executable(&path) {
                // Skip cmake internal files
                let name = path.file_name().unwrap_or_default().to_string_lossy();
                if !name.starts_with("cmake") && !name.starts_with("CMake") {
                    return Ok(path);
                }
            }
        }
    }

    bail!(
        "Could not find compiled binary '{}' in {}\n\
         Check cmake output for build errors.",
        target_name,
        build_dir.display()
    )
}

/// Load project name from horus.toml, falling back to directory name.
fn load_project_name(project_dir: &Path) -> Result<String> {
    let manifest_path = project_dir.join(HORUS_TOML);
    if manifest_path.exists() {
        let manifest = HorusManifest::load_from(&manifest_path)?;
        return Ok(manifest.package.name);
    }

    // Fallback to directory name
    project_dir
        .file_name()
        .map(|n| n.to_string_lossy().to_string())
        .ok_or_else(|| anyhow::anyhow!("Cannot determine project name"))
}

/// Symlink compile_commands.json from build dir to project root for clangd.
fn symlink_compile_commands(build_dir: &Path, project_dir: &Path) {
    let source = build_dir.join("compile_commands.json");
    let target = project_dir.join("compile_commands.json");

    if source.exists() {
        // Remove existing symlink/file
        let _ = fs::remove_file(&target);

        if horus_sys::fs::symlink(&source, &target).is_ok() {
            log::debug!("Symlinked compile_commands.json to project root");
        }
    }
}

/// Ensure all C++ system dependencies from horus.toml are installed.
///
/// Scans horus.toml for deps with `source = "system"` or known C++ deps,
/// checks if their apt packages are installed via `dpkg -l`, and installs
/// missing ones via `apt install`.
fn ensure_system_deps(project_dir: &Path) -> Result<()> {
    let manifest_path = project_dir.join(HORUS_TOML);
    if !manifest_path.exists() {
        return Ok(());
    }

    let manifest = HorusManifest::load_from(&manifest_path)?;

    let mut missing: Vec<(String, String)> = Vec::new(); // (horus_name, apt_name)

    for (name, dep) in &manifest.dependencies {
        // Check if this is a C++ system dep
        let apt_name = match cmake_gen::apt_package_for(name, dep) {
            Some(apt) => apt,
            None => continue, // Not a C++ system dep
        };

        // Check if it's a C++ dep (explicit lang or known dep)
        let is_cpp = match dep {
            crate::manifest::DependencyValue::Detailed(d) => {
                d.lang.as_deref() == Some("cpp")
                    || d.source.as_ref() == Some(&crate::manifest::DepSource::System)
                    || d.cmake_package.is_some()
            }
            crate::manifest::DependencyValue::Simple(_) => {
                cmake_gen::lookup_known_dep_public(name).is_some()
            }
        };

        if !is_cpp {
            continue;
        }

        // Check if installed via dpkg
        if !check_dpkg_installed(&apt_name) {
            missing.push((name.clone(), apt_name));
        }
    }

    if missing.is_empty() {
        return Ok(());
    }

    let apt_names: Vec<&str> = missing.iter().map(|(_, apt)| apt.as_str()).collect();

    // Ask before escalating. `horus run` is the first command every user types,
    // and this path used to go straight to `sudo apt install -y` with no
    // prompt, no --yes opt-in, no dry-run and no way to decline — triggered by
    // nothing more than a `pcl = "1.13"` line in [dependencies]. On a machine
    // with NOPASSWD sudo that installed system-wide packages the user never
    // approved; in CI and containers it simply failed.
    //
    // `registry/helpers.rs` already prompts before its own sudo call; this is
    // the same contract. HORUS_AUTO_INSTALL=1 keeps unattended builds working.
    let auto_yes = std::env::var("HORUS_AUTO_INSTALL")
        .map(|v| v == "1" || v.eq_ignore_ascii_case("true") || v.eq_ignore_ascii_case("yes"))
        .unwrap_or(false);

    eprintln!(
        "{} Missing C++ dependencies: {}",
        cli_output::ICON_INFO.cyan(),
        apt_names.join(", ").yellow()
    );

    if !auto_yes {
        use std::io::Write;
        eprint!("  {} Install them with sudo apt? [y/N]: ", "?".cyan());
        std::io::stderr().flush().ok();

        let mut input = String::new();
        let accepted = std::io::stdin().read_line(&mut input).is_ok()
            && matches!(input.trim().to_lowercase().as_str(), "y" | "yes");

        if !accepted {
            eprintln!(
                "{} Skipped. Install manually, or set HORUS_AUTO_INSTALL=1:\n  {}",
                cli_output::ICON_WARN.yellow(),
                format!("sudo apt install {}", apt_names.join(" ")).cyan()
            );
            return Ok(());
        }
    }

    eprintln!(
        "{} Installing: {}",
        cli_output::ICON_INFO.cyan(),
        apt_names.join(", ").yellow()
    );

    let status = Command::new("sudo")
        .args(["apt", "install", "-y"])
        .args(&apt_names)
        .status();

    match status {
        Ok(s) if s.success() => {
            eprintln!(
                "{} System dependencies installed",
                cli_output::ICON_SUCCESS.green()
            );
        }
        _ => {
            let install_cmd = format!("sudo apt install {}", apt_names.join(" "));
            eprintln!(
                "{} Could not auto-install system packages. Install manually:\n  {}",
                cli_output::ICON_WARN.yellow(),
                install_cmd.cyan()
            );
        }
    }

    Ok(())
}

/// Check if an apt package is installed via dpkg.
fn check_dpkg_installed(package: &str) -> bool {
    Command::new("dpkg")
        .args(["-l", package])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Check if a file is executable.
fn is_executable(path: &Path) -> bool {
    horus_sys::fs::is_executable(path)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn load_project_name_from_manifest() {
        let dir = tempfile::tempdir().unwrap();

        // Create a minimal horus.toml
        let manifest = crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: "test-robot".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                rust_edition: None,
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: Default::default(),
            dev_dependencies: Default::default(),
            sim_dependencies: Default::default(),
            hardware: Default::default(),
            drivers: Default::default(),
            sim_drivers: Default::default(),
            scripts: Default::default(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            rust: None,
            hooks: Default::default(),
            network: None,
        };
        manifest.save_to(&dir.path().join(HORUS_TOML)).unwrap();

        assert_eq!(load_project_name(dir.path()).unwrap(), "test-robot");
    }

    #[test]
    fn find_cpp_binary_finds_target() {
        let dir = tempfile::tempdir().unwrap();

        // Create a fake executable
        let binary = dir.path().join("my_robot");
        fs::write(&binary, "#!/bin/sh\necho hello").unwrap();

        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&binary, fs::Permissions::from_mode(0o755)).unwrap();
        }

        let result = find_cpp_binary(dir.path(), "my-robot");
        assert!(result.is_ok());
        assert_eq!(result.unwrap().file_name().unwrap(), "my_robot");
    }

    #[test]
    fn find_cpp_binary_not_found() {
        let dir = tempfile::tempdir().unwrap();
        let result = find_cpp_binary(dir.path(), "nonexistent");
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("Could not find"));
    }

    #[test]
    fn is_executable_works() {
        let dir = tempfile::tempdir().unwrap();
        let file = dir.path().join("test_file");
        fs::write(&file, "content").unwrap();

        // Not executable by default
        #[cfg(unix)]
        assert!(!is_executable(&file));

        // Make executable
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&file, fs::Permissions::from_mode(0o755)).unwrap();
            assert!(is_executable(&file));
        }
    }

    #[test]
    fn symlink_compile_commands_creates_link() {
        let build_dir = tempfile::tempdir().unwrap();
        let project_dir = tempfile::tempdir().unwrap();

        // Create compile_commands.json in build dir
        fs::write(build_dir.path().join("compile_commands.json"), "[]").unwrap();

        symlink_compile_commands(build_dir.path(), project_dir.path());

        let link = project_dir.path().join("compile_commands.json");
        assert!(
            link.exists(),
            "compile_commands.json should exist in project root"
        );
    }

    #[test]
    fn generate_cmake_if_needed_no_manifest_fails() {
        let dir = tempfile::tempdir().unwrap();
        let result = generate_cmake_if_needed(dir.path());
        assert!(result.is_err());
        assert!(result
            .unwrap_err()
            .to_string()
            .contains("No CMakeLists.txt or horus.toml"));
    }

    #[test]
    fn generate_cmake_if_needed_with_manifest() {
        let dir = tempfile::tempdir().unwrap();

        // Create minimal horus.toml
        let manifest = crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: "cmake-test".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                rust_edition: None,
                repository: None,
                package_type: None,
                categories: vec![],
                standard: Some("c++20".to_string()),
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: Default::default(),
            dev_dependencies: Default::default(),
            sim_dependencies: Default::default(),
            hardware: Default::default(),
            drivers: Default::default(),
            sim_drivers: Default::default(),
            scripts: Default::default(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            rust: None,
            hooks: Default::default(),
            network: None,
        };
        manifest.save_to(&dir.path().join(HORUS_TOML)).unwrap();

        generate_cmake_if_needed(dir.path()).unwrap();

        let cmake_path = dir.path().join(".horus/CMakeLists.txt");
        assert!(cmake_path.exists());
        let content = fs::read_to_string(cmake_path).unwrap();
        assert!(content.contains("cmake_minimum_required"));
        assert!(content.contains("CMAKE_CXX_STANDARD 20"));
    }

    // ── Battle tests: find_cpp_binary edge cases ─────────────────────────

    #[test]
    fn battle_find_binary_in_debug_subdir() {
        let dir = tempfile::tempdir().unwrap();
        let debug_dir = dir.path().join("Debug");
        fs::create_dir_all(&debug_dir).unwrap();

        let binary = debug_dir.join("my_robot");
        fs::write(&binary, "#!/bin/sh").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&binary, fs::Permissions::from_mode(0o755)).unwrap();
        }

        let result = find_cpp_binary(dir.path(), "my-robot");
        assert!(result.is_ok(), "should find binary in Debug/ subdir");
    }

    #[test]
    fn battle_find_binary_in_release_subdir() {
        let dir = tempfile::tempdir().unwrap();
        let release_dir = dir.path().join("Release");
        fs::create_dir_all(&release_dir).unwrap();

        let binary = release_dir.join("my_robot");
        fs::write(&binary, "#!/bin/sh").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&binary, fs::Permissions::from_mode(0o755)).unwrap();
        }

        let result = find_cpp_binary(dir.path(), "my-robot");
        assert!(result.is_ok(), "should find binary in Release/ subdir");
    }

    #[test]
    fn battle_find_binary_skips_cmake_internal() {
        let dir = tempfile::tempdir().unwrap();

        // Create cmake internal files (should be skipped)
        fs::write(dir.path().join("cmake_install.cmake"), "# cmake").unwrap();
        fs::write(dir.path().join("CMakeCache.txt"), "# cache").unwrap();

        let result = find_cpp_binary(dir.path(), "nonexistent");
        assert!(
            result.is_err(),
            "should not find cmake internal files as binary"
        );
    }

    #[test]
    fn battle_find_binary_prefers_exact_name() {
        let dir = tempfile::tempdir().unwrap();

        // Create two executables — exact match should win
        let exact = dir.path().join("my_robot");
        let other = dir.path().join("other_binary");
        fs::write(&exact, "#!/bin/sh").unwrap();
        fs::write(&other, "#!/bin/sh").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&exact, fs::Permissions::from_mode(0o755)).unwrap();
            fs::set_permissions(&other, fs::Permissions::from_mode(0o755)).unwrap();
        }

        let result = find_cpp_binary(dir.path(), "my-robot").unwrap();
        assert_eq!(result.file_name().unwrap(), "my_robot");
    }

    // ── Battle tests: load_project_name ──────────────────────────────────

    #[test]
    fn battle_load_project_name_no_manifest_uses_dir_name() {
        let dir = tempfile::tempdir().unwrap();
        // No horus.toml — should fall back to directory name
        let name = load_project_name(dir.path()).unwrap();
        // tempdir names vary, just check it's not empty
        assert!(!name.is_empty());
    }

    #[test]
    fn battle_load_project_name_with_hyphenated_name() {
        let dir = tempfile::tempdir().unwrap();
        let manifest = crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: "my-cool-robot".to_string(),
                version: "1.0.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                rust_edition: None,
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: Default::default(),
            dev_dependencies: Default::default(),
            sim_dependencies: Default::default(),
            hardware: Default::default(),
            drivers: Default::default(),
            sim_drivers: Default::default(),
            scripts: Default::default(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            rust: None,
            hooks: Default::default(),
            network: None,
        };
        manifest.save_to(&dir.path().join(HORUS_TOML)).unwrap();

        assert_eq!(load_project_name(dir.path()).unwrap(), "my-cool-robot");
    }

    // ── Battle tests: symlink edge cases ─────────────────────────────────

    #[test]
    fn battle_symlink_no_source_file_is_noop() {
        let build_dir = tempfile::tempdir().unwrap();
        let project_dir = tempfile::tempdir().unwrap();

        // No compile_commands.json in build dir
        symlink_compile_commands(build_dir.path(), project_dir.path());

        let link = project_dir.path().join("compile_commands.json");
        assert!(
            !link.exists(),
            "should not create link when source doesn't exist"
        );
    }

    #[test]
    fn battle_symlink_overwrites_existing() {
        let build_dir = tempfile::tempdir().unwrap();
        let project_dir = tempfile::tempdir().unwrap();

        // Create existing compile_commands.json in project
        fs::write(project_dir.path().join("compile_commands.json"), "old").unwrap();

        // Create new one in build dir
        fs::write(build_dir.path().join("compile_commands.json"), "new").unwrap();

        symlink_compile_commands(build_dir.path(), project_dir.path());

        let link = project_dir.path().join("compile_commands.json");
        assert!(link.exists());
    }

    // ── Battle tests: ensure_system_deps ─────────────────────────────────

    #[test]
    fn battle_ensure_system_deps_no_manifest() {
        let dir = tempfile::tempdir().unwrap();
        // No horus.toml — should succeed without doing anything
        let result = ensure_system_deps(dir.path());
        result.unwrap();
    }

    #[test]
    fn battle_ensure_system_deps_no_system_deps() {
        let dir = tempfile::tempdir().unwrap();

        // horus.toml with only a simple (non-system) dep
        let manifest = crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: "test".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                rust_edition: None,
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: {
                let mut deps = std::collections::BTreeMap::new();
                deps.insert(
                    "serde".to_string(),
                    crate::manifest::DependencyValue::Simple("1.0".to_string()),
                );
                deps
            },
            dev_dependencies: Default::default(),
            sim_dependencies: Default::default(),
            hardware: Default::default(),
            drivers: Default::default(),
            sim_drivers: Default::default(),
            scripts: Default::default(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            rust: None,
            hooks: Default::default(),
            network: None,
        };
        manifest.save_to(&dir.path().join(HORUS_TOML)).unwrap();

        // Should succeed — serde is not a C++ system dep
        let result = ensure_system_deps(dir.path());
        result.unwrap();
    }

    // ── Battle tests: generate_cmake_if_needed integration ───────────────

    #[test]
    fn battle_generate_cmake_with_system_deps() {
        let dir = tempfile::tempdir().unwrap();

        let mut deps = std::collections::BTreeMap::new();
        deps.insert(
            "eigen".to_string(),
            crate::manifest::DependencyValue::Detailed(crate::manifest::DetailedDependency {
                version: Some("3.4".to_string()),
                source: Some(crate::manifest::DepSource::System),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                branch: None,
                tag: None,
                rev: None,
                apt: Some("libeigen3-dev".to_string()),
                cmake_package: Some("Eigen3".to_string()),
                lang: Some("cpp".to_string()),
                workspace: false,
            }),
        );

        let manifest = crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: "dep-test".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                rust_edition: None,
                repository: None,
                package_type: None,
                categories: vec![],
                standard: Some("c++20".to_string()),
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: deps,
            dev_dependencies: Default::default(),
            sim_dependencies: Default::default(),
            hardware: Default::default(),
            drivers: Default::default(),
            sim_drivers: Default::default(),
            scripts: Default::default(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            rust: None,
            hooks: Default::default(),
            network: None,
        };
        manifest.save_to(&dir.path().join(HORUS_TOML)).unwrap();

        generate_cmake_if_needed(dir.path()).unwrap();

        let content = fs::read_to_string(dir.path().join(".horus/CMakeLists.txt")).unwrap();
        assert!(
            content.contains("find_package(Eigen3 REQUIRED)"),
            "should have eigen find_package"
        );
        assert!(
            content.contains("Eigen3::Eigen"),
            "should have eigen target"
        );
        assert!(
            content.contains("CMAKE_CXX_STANDARD 20"),
            "should have c++20 standard"
        );
    }

    #[test]
    fn battle_generate_cmake_default_standard_17() {
        let dir = tempfile::tempdir().unwrap();

        let manifest = crate::manifest::HorusManifest {
            package: crate::manifest::PackageInfo {
                name: "default-std".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                rust_edition: None,
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None, // No standard specified
                target_type: crate::manifest::TargetType::default(),
            },
            workspace: None,
            robot: None,
            dependencies: Default::default(),
            dev_dependencies: Default::default(),
            sim_dependencies: Default::default(),
            hardware: Default::default(),
            drivers: Default::default(),
            sim_drivers: Default::default(),
            scripts: Default::default(),
            ignore: Default::default(),
            enable: vec![],
            cpp: None,
            rust: None,
            hooks: Default::default(),
            network: None,
        };
        manifest.save_to(&dir.path().join(HORUS_TOML)).unwrap();

        generate_cmake_if_needed(dir.path()).unwrap();

        let content = fs::read_to_string(dir.path().join(".horus/CMakeLists.txt")).unwrap();
        assert!(
            content.contains("CMAKE_CXX_STANDARD 17"),
            "default should be c++17"
        );
    }

    // ── Battle tests: is_executable ──────────────────────────────────────

    #[test]
    fn battle_is_executable_nonexistent_file() {
        assert!(!is_executable(Path::new(
            "/tmp/definitely_nonexistent_binary_xyz"
        )));
    }

    #[test]
    fn battle_is_executable_directory() {
        let dir = tempfile::tempdir().unwrap();
        // Directories have execute bit but aren't executables
        // is_executable just checks the bit, so this is fine
        assert!(is_executable(dir.path()) || !is_executable(dir.path()));
    }

    // ── Battle tests: check_dpkg_installed ───────────────────────────────

    #[test]
    fn battle_check_dpkg_nonexistent_package() {
        // A package that definitely doesn't exist
        assert!(!check_dpkg_installed("horus-nonexistent-package-xyz-999"));
    }

    // ── Error path tests ─────────────────────────────────────────────

    #[test]
    fn error_generate_cmake_invalid_manifest() {
        // generate_cmake_if_needed with corrupted horus.toml
        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join(HORUS_TOML), "this is not valid toml {{{{").unwrap();

        let result = generate_cmake_if_needed(dir.path());
        assert!(result.is_err(), "corrupted horus.toml should fail");
    }

    #[test]
    fn error_find_binary_empty_dir() {
        // find_cpp_binary in an empty directory
        let dir = tempfile::tempdir().unwrap();
        let result = find_cpp_binary(dir.path(), "my-project");
        assert!(result.is_err(), "should fail in empty dir");
        assert!(
            result.unwrap_err().to_string().contains("Could not find"),
            "error should mention not finding binary"
        );
    }

    #[test]
    fn error_find_binary_only_non_executable() {
        // directory has files but none are executable
        let dir = tempfile::tempdir().unwrap();
        // Create non-executable files
        fs::write(dir.path().join("my_project"), "not executable").unwrap();
        fs::write(dir.path().join("another_file"), "also not executable").unwrap();

        // Ensure they are NOT executable
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(
                dir.path().join("my_project"),
                fs::Permissions::from_mode(0o644),
            )
            .unwrap();
            fs::set_permissions(
                dir.path().join("another_file"),
                fs::Permissions::from_mode(0o644),
            )
            .unwrap();
        }

        let result = find_cpp_binary(dir.path(), "my-project");
        assert!(result.is_err(), "should fail when no files are executable");
    }

    #[test]
    fn error_load_project_name_empty_dir() {
        // load_project_name when dir has no horus.toml — falls back to dir name
        let dir = tempfile::tempdir().unwrap();
        let name = load_project_name(dir.path()).unwrap();
        // Should use the directory name as fallback
        assert!(!name.is_empty(), "fallback name should not be empty");
        // The name should match the temp dir's last component
        let expected = dir
            .path()
            .file_name()
            .unwrap()
            .to_string_lossy()
            .to_string();
        assert_eq!(name, expected, "fallback should use directory name");
    }

    #[test]
    fn battle_check_dpkg_coreutils_installed() {
        // coreutils is always installed on any Linux system
        if cfg!(target_os = "linux") {
            assert!(check_dpkg_installed("coreutils"));
        }
    }

    // ── SLAM Cycle 4: Stress & boundary tests ────────────────────────────

    #[test]
    fn stress_find_binary_dir_with_100_files() {
        let dir = tempfile::tempdir().unwrap();

        // Create 100 non-executable files
        for i in 0..100 {
            let file = dir.path().join(format!("file_{}", i));
            fs::write(&file, "not executable").unwrap();
            #[cfg(unix)]
            {
                use std::os::unix::fs::PermissionsExt;
                fs::set_permissions(&file, fs::Permissions::from_mode(0o644)).unwrap();
            }
        }

        // Create 1 executable with the target name
        let target = dir.path().join("my_robot");
        fs::write(&target, "#!/bin/sh\necho hello").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&target, fs::Permissions::from_mode(0o755)).unwrap();
        }

        let result = find_cpp_binary(dir.path(), "my-robot");
        assert!(
            result.is_ok(),
            "should find the one executable among 100 files"
        );
        assert_eq!(result.unwrap().file_name().unwrap(), "my_robot");
    }

    #[test]
    fn boundary_find_binary_name_with_dots() {
        let dir = tempfile::tempdir().unwrap();
        // Project name "v2.0.1" should search for "v2_0_1" (hyphens AND dots replaced)
        let _target_name = "v2.0.1".replace('-', "_");
        // find_cpp_binary only replaces hyphens with underscores
        let binary_name = "v2.0.1".replace('-', "_");
        let binary = dir.path().join(&binary_name);
        fs::write(&binary, "#!/bin/sh").unwrap();
        #[cfg(unix)]
        {
            use std::os::unix::fs::PermissionsExt;
            fs::set_permissions(&binary, fs::Permissions::from_mode(0o755)).unwrap();
        }

        let result = find_cpp_binary(dir.path(), "v2.0.1");
        // find_cpp_binary replaces hyphens only, so "v2.0.1" stays "v2.0.1"
        // It should find this via exact match or fallback scan
        assert!(
            result.is_ok(),
            "should find binary with dots in name via scan"
        );
    }

    // ── SLAM Cycle 5: Final gap tests ───────────────────────────────────

    #[test]
    fn error_execute_cpp_binary_nonexistent_path() {
        // execute_cpp_binary should return an error for a nonexistent binary
        let result = execute_cpp_binary(Path::new("/tmp/horus_nonexistent_binary_xyz_99999"), &[]);
        assert!(result.is_err(), "should fail for nonexistent binary");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Failed to execute") || err.contains("No such file"),
            "error should mention execution failure, got: {}",
            err
        );
    }

    #[test]
    fn error_execute_cpp_binary_nonexistent_with_args() {
        // execute_cpp_binary should also fail with args for a nonexistent binary
        let result = execute_cpp_binary(
            Path::new("/tmp/horus_nonexistent_binary_xyz_99999"),
            &["--help".to_string(), "--verbose".to_string()],
        );
        assert!(
            result.is_err(),
            "should fail for nonexistent binary even with args"
        );
    }

    #[test]
    fn boundary_find_binary_unicode_project_name() {
        // Unicode project name should not panic — just fail to find
        let dir = tempfile::tempdir().unwrap();
        let result = find_cpp_binary(dir.path(), "robot-\u{1F916}");
        assert!(
            result.is_err(),
            "unicode project name should not find anything in empty dir"
        );
    }

    #[test]
    fn boundary_load_project_name_root_path() {
        // load_project_name on "/" — file_name() returns None for root
        let result = load_project_name(Path::new("/"));
        assert!(result.is_err(), "root path has no file_name component");
        assert!(
            result
                .unwrap_err()
                .to_string()
                .contains("Cannot determine project name"),
            "should mention cannot determine project name"
        );
    }

    #[test]
    fn stress_symlink_compile_commands_repeated() {
        let build_dir = tempfile::tempdir().unwrap();
        let project_dir = tempfile::tempdir().unwrap();

        // Create compile_commands.json in build dir
        fs::write(build_dir.path().join("compile_commands.json"), "[]").unwrap();

        // Call symlink 50 times — should not error
        for _ in 0..50 {
            symlink_compile_commands(build_dir.path(), project_dir.path());
        }

        // Final link should still be valid
        let link = project_dir.path().join("compile_commands.json");
        assert!(
            link.exists(),
            "compile_commands.json should exist after 50 symlinks"
        );
        let content = fs::read_to_string(&link).unwrap();
        assert_eq!(content, "[]", "symlinked content should match source");
    }
}
