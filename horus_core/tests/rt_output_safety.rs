//! No production path in horus_core may use `println!`/`eprintln!`.
//!
//! Those macros panic when the write fails. Piping a robot's output into
//! anything that exits early — `head`, a log reader that restarts, an operator
//! quitting `less` — closes stdout, and the panic kills the thread that was
//! printing. Measured before this guard existed, with a 1 kHz RT node and
//! `| head -c 100`:
//!
//!     thread 'horus-rt' panicked at library/std/src/io/stdio.rs: Broken pipe
//!     RESULT ticks=0
//!
//! The control loop never ran a single tick. The same failure on a background
//! thread is quieter and worse: the thread dies, the process keeps running, and
//! the subsystem is silently gone.
//!
//! `horus_core::terminal::{print_line, eprint_line}` exist precisely for this —
//! they use `write!` and swallow the error. This test keeps the crate on them.

use std::path::{Path, PathBuf};

fn crate_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn rust_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).expect("read src dir").flatten() {
        let path = entry.path();
        if path.is_dir() {
            rust_sources(&path, out);
        } else if path.extension().is_some_and(|e| e == "rs") {
            out.push(path);
        }
    }
}

/// Files pulled in only under `#[cfg(test)] mod foo;`. Their contents are test
/// code even though nothing inside them says so, and test code may print
/// however it likes: a panicking test is a reported failure, not a robot that
/// silently loses a thread.
fn test_only_modules(files: &[PathBuf]) -> Vec<PathBuf> {
    let mut out = Vec::new();
    for file in files {
        let Ok(src) = std::fs::read_to_string(file) else {
            continue;
        };
        let dir = file.parent().expect("source has a parent");
        let mut rest = src.as_str();
        while let Some(i) = rest.find("#[cfg(test)]") {
            rest = &rest[i + "#[cfg(test)]".len()..];
            let decl = rest.trim_start();
            let Some(after_mod) = decl.strip_prefix("mod ") else {
                continue;
            };
            let Some(name) = after_mod.split(';').next() else {
                continue;
            };
            // `mod foo;` (a file), not `mod foo { ... }` (inline)
            if !after_mod.starts_with(name) || after_mod[name.len()..].starts_with(" {") {
                continue;
            }
            let name = name.trim();
            if name.is_empty() || !name.chars().all(|c| c.is_alphanumeric() || c == '_') {
                continue;
            }
            out.push(dir.join(format!("{name}.rs")));
            out.push(dir.join(name).join("mod.rs"));
        }
    }
    out
}

/// Everything outside `#[cfg(test)]` blocks, with comments removed so a macro
/// named in prose does not fail the test — several of these files document the
/// hazard they are avoiding.
fn production_code(src: &str) -> String {
    let mut kept = String::with_capacity(src.len());
    let mut rest = src;
    while let Some(pos) = rest.find("#[cfg(test)]") {
        kept.push_str(&rest[..pos]);
        let after = &rest[pos..];
        let Some(brace) = after.find('{') else { break };
        let mut depth = 0usize;
        let mut end = None;
        for (i, c) in after[brace..].char_indices() {
            match c {
                '{' => depth += 1,
                '}' => {
                    depth -= 1;
                    if depth == 0 {
                        end = Some(brace + i + 1);
                        break;
                    }
                }
                _ => {}
            }
        }
        match end {
            Some(e) => rest = &after[e..],
            None => break,
        }
    }
    kept.push_str(rest);

    kept.lines()
        .map(|l| l.split("//").next().unwrap_or(""))
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn no_production_path_uses_a_panicking_print_macro() {
    let src_dir = crate_root().join("src");
    let mut files = Vec::new();
    rust_sources(&src_dir, &mut files);
    assert!(files.len() > 50, "expected to scan the whole crate");

    let skip = test_only_modules(&files);
    let mut offenders = Vec::new();

    for file in &files {
        if skip.contains(file) {
            continue;
        }
        let src = std::fs::read_to_string(file).expect("read source");
        let rel = file.strip_prefix(crate_root()).unwrap_or(file);
        for (n, line) in production_code(&src).lines().enumerate() {
            for m in ["println!", "eprintln!", "print!(", "eprint!("] {
                if line.contains(m) {
                    offenders.push(format!(
                        "  {}:{} uses {m}\n      {}",
                        rel.display(),
                        n + 1,
                        line.trim()
                    ));
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "panicking print macros in production code:\n{}\n\n\
         These kill the thread when the stream is closed — a robot piped into \
         `head`, or whose supervisor exited. Use `crate::terminal::print_line` \
         or `eprint_line`, which swallow the write error.",
        offenders.join("\n")
    );
}

#[test]
fn the_scan_actually_reaches_the_executors() {
    // A scan that silently stopped finding files would pass forever. Anchor it
    // on the paths where a print panic is fatal rather than merely bad.
    let root = crate_root();
    for rel in [
        "src/scheduling/rt.rs",
        "src/scheduling/rt_executor.rs",
        "src/scheduling/safety_monitor.rs",
        "src/core/rt_config.rs",
    ] {
        assert!(root.join(rel).exists(), "{rel} moved — update this anchor");
    }

    let mut files = Vec::new();
    rust_sources(&root.join("src"), &mut files);
    for entry in std::fs::read_dir(root.join("src/scheduling")).expect("scheduling dir") {
        let p = entry.expect("dir entry").path();
        if p.file_name()
            .and_then(|s| s.to_str())
            .is_some_and(|n| n.ends_with("_executor.rs"))
        {
            assert!(files.contains(&p), "{} was not scanned", p.display());
        }
    }
}
