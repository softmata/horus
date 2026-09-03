//! Every `SLOT_WRITING` marker in `horus_core` must be followed by a Release
//! fence before the payload it guards is written.
//!
//! The marker is the write side of a Boehm seqlock: the consumer
//! (`recv_shm_pod_broadcast`) loads the stamp, copies the slot, executes an
//! Acquire fence and requires the stamp to be unchanged. That protocol rejects a
//! lapping producer only if the producer's payload store cannot be observed
//! ahead of the marker — which is what the Release fence buys, and what a
//! Release *store* on the marker does NOT. A release store orders the accesses
//! before itself and says nothing about the store that follows, so the consumer
//! can see new bytes under two matching stale stamps and accept a value that is
//! half of one message and half of another.
//!
//! That is not a theoretical distinction on the targets this ships to. aarch64
//! `-O` compiles the marker to `stlr` and the payload to a plain `str` with
//! nothing in between; with the fence it is `str` + `dmb ish` + `str`. On x86
//! the fence costs nothing (it is a no-op for StoreStore), so there is no
//! platform on which the naive form is worth keeping.
//!
//! `tests/loom_pod_broadcast.rs` proves the two broken forms tear
//! (`naive_protocol_tears_proving_the_model_can_detect_it` and
//! `colo_release_marker_tears_proving_the_fence_is_load_bearing`). Those models
//! are transcriptions, so nothing stops the real code from drifting back to a
//! form they already condemn — `Topic::send`'s colo fast path did exactly that,
//! stamping with `store(Release)` while the three other write phases in the
//! crate were fenced. This test is the thing that notices.

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

/// Write phases whose stamp is not read by anybody, checked one at a time.
///
/// `send_shm_sp_pod` serves SpmcShm only (`mod.rs`'s send-dispatch table), and
/// its consumer `recv_shm_spmc_pod` never loads the per-slot stamp at all — it
/// gates on head/tail and a CAS on `header.tail`. Its marker is therefore inert
/// and cannot tear, whatever ordering it uses; the "Boehm seqlock write side"
/// comment above it describes `send_shm_pod_broadcast`, not itself. If a reader
/// is ever wired onto that stamp, delete this entry and fence the site — do not
/// widen the exemption.
const EXEMPT_FNS: &[&str] = &["send_shm_sp_pod"];

/// Name of the `fn` a line sits inside, searching upward. Returns `None` rather
/// than guessing, which makes an unrecognised site fail the test instead of
/// silently matching an exemption.
fn enclosing_fn(lines: &[&str], line_idx: usize) -> Option<String> {
    for line in lines[..=line_idx].iter().rev() {
        let mut rest = line.trim_start();
        // Skip the modifiers a definition can carry before `fn`.
        loop {
            let next = rest
                .strip_prefix("pub(crate) ")
                .or_else(|| rest.strip_prefix("pub(super) "))
                .or_else(|| rest.strip_prefix("pub "))
                .or_else(|| rest.strip_prefix("const "))
                .or_else(|| rest.strip_prefix("async "))
                .or_else(|| rest.strip_prefix("unsafe "))
                .or_else(|| rest.strip_prefix("extern \"C\" "));
            match next {
                Some(r) => rest = r,
                None => break,
            }
        }
        if let Some(after) = rest.strip_prefix("fn ") {
            let name: String = after
                .chars()
                .take_while(|c| c.is_alphanumeric() || *c == '_')
                .collect();
            if !name.is_empty() {
                return Some(name);
            }
        }
    }
    None
}

/// A line that publishes the marker: it names `SLOT_WRITING` and stores it,
/// either through an atomic directly or through a `publish` helper. Reads of the
/// bit (`v1 & SLOT_WRITING`, `raw & !SLOT_WRITING`) and the `use`/`const` lines
/// match neither.
fn publishes_marker(line: &str) -> bool {
    let trimmed = line.trim_start();
    if trimmed.starts_with("//") || !trimmed.contains("SLOT_WRITING") {
        return false;
    }
    trimmed.contains(".store(") || trimmed.contains("publish(")
}

#[test]
fn every_writing_marker_is_followed_by_a_release_fence() {
    let mut files = Vec::new();
    rust_sources(&crate_root().join("src"), &mut files);
    files.sort();

    let mut checked = 0usize;
    let mut violations = Vec::new();

    for file in &files {
        let src = std::fs::read_to_string(file).expect("read source");
        let lines: Vec<&str> = src.lines().collect();
        let rel = file
            .strip_prefix(crate_root())
            .unwrap_or(file)
            .display()
            .to_string();

        for (i, line) in lines.iter().enumerate() {
            if !publishes_marker(line) {
                continue;
            }
            let site = format!("{rel}:{}", i + 1);
            match enclosing_fn(&lines, i) {
                Some(name) if EXEMPT_FNS.contains(&name.as_str()) => continue,
                Some(_) => {}
                None => {
                    violations.push(format!(
                        "{site}: marker published outside any fn — this test cannot \
                         tell whether it is a seqlock write phase"
                    ));
                    continue;
                }
            }

            // The statement may wrap; find the line that ends it, then the next
            // line that is actually code.
            let Some(end) = (i..lines.len()).find(|&j| lines[j].contains(';')) else {
                violations.push(format!("{site}: marker store has no statement end"));
                continue;
            };
            let next = lines[end + 1..].iter().find(|l| {
                let t = l.trim();
                !t.is_empty() && !t.starts_with("//")
            });

            checked += 1;
            match next {
                Some(l) if l.contains("fence(Ordering::Release)") => {}
                other => violations.push(format!(
                    "{site}: the WRITING marker is not followed by a Release fence \
                     (next code line: {}). A Release store on the marker does not \
                     order the payload write below it; see this file's header.",
                    other.map(|l| l.trim()).unwrap_or("<end of file>")
                )),
            }
        }
    }

    // A rename of the constant, or a refactor that moves every write phase out
    // of `src`, would otherwise leave this test passing while checking nothing.
    assert!(
        checked >= 3,
        "expected at least 3 SLOT_WRITING write phases in horus_core/src, found \
         {checked} — the marker was probably renamed and this guard is now blind"
    );

    assert!(
        violations.is_empty(),
        "unfenced seqlock write phase(s):\n  {}",
        violations.join("\n  ")
    );
}
