//! Nothing on a real-time path may use `println!`/`eprintln!`.
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
//! The control loop never ran a single tick. `horus_core::terminal::print_line`
//! exists precisely for this — it uses `write!` and swallows the error — and
//! its own doc comment says so. This test keeps the RT paths on it.

use std::path::{Path, PathBuf};

/// Files reachable from an executor thread, where a panicking write is fatal.
const RT_REACHABLE: &[&str] = &[
    "src/scheduling/rt.rs",
    "src/scheduling/rt_executor.rs",
    "src/scheduling/compute_executor.rs",
    "src/scheduling/event_executor.rs",
    "src/scheduling/async_executor.rs",
    "src/scheduling/primitives.rs",
    "src/scheduling/safety_monitor.rs",
    "src/core/rt_config.rs",
];

fn crate_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Strip line comments and doc comments so a macro named in prose does not fail
/// the test — several of these files document the hazard they are avoiding.
fn code_only(src: &str) -> String {
    src.lines()
        .filter(|l| {
            let t = l.trim_start();
            !(t.starts_with("//") || t.starts_with("///") || t.starts_with("//!"))
        })
        .collect::<Vec<_>>()
        .join("\n")
}

#[test]
fn rt_paths_do_not_use_panicking_print_macros() {
    let root = crate_root();
    let mut offenders = Vec::new();

    for rel in RT_REACHABLE {
        let path = root.join(rel);
        let Ok(src) = std::fs::read_to_string(&path) else {
            panic!("{rel} is listed here but missing — update RT_REACHABLE");
        };
        // Only the live half of the file; test modules may print freely.
        let live = match src.find("#[cfg(test)]") {
            Some(i) => &src[..i],
            None => &src[..],
        };
        for (n, line) in code_only(live).lines().enumerate() {
            for m in ["println!", "eprintln!", "print!(", "eprint!("] {
                if line.contains(m) {
                    offenders.push(format!("  {rel}:{} uses {m}\n      {}", n + 1, line.trim()));
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "panicking print macros on real-time paths:\n{}\n\n\
         These abort the thread when stdout is closed — a robot piped into \
         `head`, or whose log reader restarts, loses the thread that printed. \
         Use `crate::terminal::print_line`, which swallows the write error.",
        offenders.join("\n")
    );
}

#[test]
fn rt_reachable_list_covers_the_executors() {
    // Guard against a new executor file being added without being listed here.
    let sched = crate_root().join("src/scheduling");
    let mut missing = Vec::new();
    for entry in std::fs::read_dir(&sched).expect("scheduling dir") {
        let p = entry.expect("dir entry").path();
        let name = p.file_name().and_then(|s| s.to_str()).unwrap_or("");
        if name.ends_with("_executor.rs") {
            let rel = format!("src/scheduling/{name}");
            if !RT_REACHABLE.contains(&rel.as_str()) {
                missing.push(rel);
            }
        }
    }
    assert!(
        missing.is_empty(),
        "executor files not covered by RT_REACHABLE: {missing:?}"
    );
    assert!(Path::new(&sched).exists());
}
