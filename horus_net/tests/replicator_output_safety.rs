//! The replicator thread must never print with a macro that panics.
//!
//! Every diagnostic in `horus_net` is emitted from the `horus-net` thread, which
//! runs for the whole life of the process. `eprintln!` panics when the write
//! fails, and a robot's stderr fails routinely: the supervisor that launched it
//! exits, or an operator pipes `horus run` into `head`. A panic there unwinds
//! that thread alone — the release profile has no `panic = "abort"` — leaving
//! the process alive with networking silently dead, including networked e-stop,
//! while `ReplicatorHandle::is_running()` keeps returning true because the flag
//! is a separate atomic.
//!
//! Measured before the fix: a thread that calls `eprintln!` with the pipe reader
//! closed joins as `Err`; the same thread using `writeln!` returns normally.

use std::path::{Path, PathBuf};

fn crate_src() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("src")
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

/// Strip `#[cfg(test)] mod ... { ... }` blocks: test code may print however it
/// likes, since a panicking test is a reported failure rather than a robot that
/// silently loses its network.
fn without_test_modules(src: &str) -> String {
    let mut out = String::with_capacity(src.len());
    let mut rest = src;
    while let Some(pos) = rest.find("#[cfg(test)]") {
        out.push_str(&rest[..pos]);
        let after = &rest[pos..];
        let Some(brace) = after.find('{') else {
            break;
        };
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
    out.push_str(rest);
    out
}

#[test]
fn no_replicator_source_uses_a_panicking_print_macro() {
    let mut files = Vec::new();
    rust_sources(&crate_src(), &mut files);
    assert!(files.len() > 5, "expected to scan the whole crate");

    let mut offenders = Vec::new();
    for file in &files {
        let src = std::fs::read_to_string(file).expect("read source");
        let scanned = without_test_modules(&src);
        for (i, line) in scanned.lines().enumerate() {
            let code = line.split("//").next().unwrap_or("");
            if code.contains("eprintln!") || code.contains("println!") {
                offenders.push(format!(
                    "{}:{} {}",
                    file.file_name().unwrap().to_string_lossy(),
                    i + 1,
                    code.trim()
                ));
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "these run on the horus-net thread and panic on a broken stderr, killing \
         networking while the process reports it as running. Use \
         `horus_core::terminal::eprint_line` / `print_line`:\n  {}",
        offenders.join("\n  ")
    );
}
