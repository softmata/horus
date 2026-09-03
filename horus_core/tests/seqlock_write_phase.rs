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
//!
//! What counts as "followed by a Release fence" is deliberately narrow on one
//! axis and wide on another: the call must be `sync::atomic::fence` and not
//! `compiler_fence` (which constrains the optimiser and emits no instruction —
//! it would leave the aarch64 reordering this file exists to prevent), and the
//! ordering may be `Release` or anything stronger (`AcqRel`, `SeqCst`).
//!
//! A guard that silently stops checking is worse than no guard, so every
//! mention of the constant in `src` must fall into a shape this file
//! recognises. An unrecognised one — a publication that grew a line wrap, say —
//! fails the test instead of dropping out of the scan.

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

/// The line with `//` comments dropped and string/char literals blanked, so the
/// scanning below sees punctuation that is code and nothing else.
///
/// Without this, a trailing comment or a literal that happens to contain `;`,
/// a bracket, or the word `fence` steers the delimiter counting and the fence
/// match. Both directions of that are bad: a spurious CI failure, or a real
/// unfenced site skipped.
fn code_only(line: &str) -> String {
    let chars: Vec<char> = line.chars().collect();
    let mut out = String::with_capacity(line.len());
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c == '/' && chars.get(i + 1) == Some(&'/') {
            break;
        }
        if c == '"' {
            i += 1;
            while i < chars.len() {
                if chars[i] == '\\' {
                    i += 2;
                    continue;
                }
                if chars[i] == '"' {
                    break;
                }
                i += 1;
            }
            i += 1;
            out.push('_');
            continue;
        }
        // `'` is a char literal only in `'x'` / `'\n'` shape; otherwise it opens
        // a lifetime and must be left alone.
        if c == '\'' {
            let escaped = chars.get(i + 1) == Some(&'\\');
            let simple = chars.get(i + 2) == Some(&'\'');
            if escaped || simple {
                i += 1;
                while i < chars.len() {
                    if chars[i] == '\\' {
                        i += 2;
                        continue;
                    }
                    if chars[i] == '\'' {
                        break;
                    }
                    i += 1;
                }
                i += 1;
                out.push('_');
                continue;
            }
        }
        out.push(c);
        i += 1;
    }
    out
}

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

/// Index of the line holding the `;` that ends the statement starting at
/// `start`.
///
/// The terminator is the first `;` at delimiter depth zero *relative to the
/// start of the statement*, not simply the first `;` at or after `start`. A
/// wrapped store whose arguments carry a block or a macro with its own `;`
/// — `store({ let v = f(); v }, Ordering::Release)` — would otherwise be cut
/// short, and the fence check would then run against a line from the middle of
/// the store and report a violation that is not there.
fn statement_end(lines: &[&str], start: usize) -> Option<usize> {
    let mut depth: i32 = 0;
    for (offset, line) in lines[start..].iter().enumerate() {
        for c in code_only(line).chars() {
            match c {
                '(' | '[' | '{' => depth += 1,
                ')' | ']' | '}' => depth -= 1,
                ';' if depth <= 0 => return Some(start + offset),
                _ => {}
            }
        }
    }
    None
}

/// Whether `code` calls `sync::atomic::fence` with Release or stronger.
///
/// Not a substring test for `fence(Ordering::Release)`: that also matches
/// `compiler_fence(Ordering::Release)`, which is an optimisation barrier that
/// emits no instruction and leaves aarch64 free to sink the payload store
/// past the marker — the guard would pass on exactly the bug it is here to
/// catch. `compiler_fence` is already used in this crate
/// (`src/memory/backend.rs`), so that is drift the codebase can actually
/// perform, not a hypothetical. Qualified paths (`std::sync::atomic::fence`,
/// `core::sync::atomic::fence`, `atomic::fence`) are all accepted, as are
/// orderings stronger than Release.
fn is_release_fence(code: &str) -> bool {
    const NEEDLE: &str = "fence(";
    const ORDERINGS: &[&str] = &["Release", "AcqRel", "SeqCst"];
    let mut from = 0usize;
    while let Some(rel) = code[from..].find(NEEDLE) {
        let at = from + rel;
        // `compiler_fence` and any other `*_fence`: the byte before the needle
        // continues an identifier. `::fence(` and ` fence(` do not.
        let joined_to_ident = code[..at]
            .chars()
            .next_back()
            .is_some_and(|c| c.is_alphanumeric() || c == '_');
        if !joined_to_ident {
            let args = &code[at + NEEDLE.len()..];
            let args = args.split(')').next().unwrap_or(args);
            if ORDERINGS.iter().any(|o| args.contains(o)) {
                return true;
            }
        }
        from = at + NEEDLE.len();
    }
    false
}

/// A statement that publishes the marker: it names `SLOT_WRITING` and stores
/// it, either through an atomic directly or through a `publish` helper.
fn publishes_marker(code: &str) -> bool {
    code.contains("SLOT_WRITING") && (code.contains(".store(") || code.contains("publish("))
}

/// A mention that only *reads* the bit — `v1 & SLOT_WRITING`,
/// `raw & !layout::SLOT_WRITING`. Nothing is published, so there is nothing to
/// order.
fn reads_marker(code: &str) -> bool {
    let Some(at) = code.find("SLOT_WRITING") else {
        return false;
    };
    let before = code[..at].trim_end_matches(|c: char| c.is_alphanumeric() || c == '_' || c == ':');
    let before = before.trim_end().trim_end_matches('!').trim_end();
    before.ends_with('&')
}

/// Blank lines, `//` comments and the body of a `/* */` block — none of them is
/// the statement that follows the marker.
fn is_code_line(line: &str) -> bool {
    let t = line.trim();
    !t.is_empty()
        && !t.starts_with("//")
        && !t.starts_with("/*")
        && !t.starts_with('*')
        && !t.starts_with("*/")
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
            let code = code_only(line);
            if !code.contains("SLOT_WRITING") {
                continue;
            }
            let site = format!("{rel}:{}", i + 1);
            let trimmed = code.trim();

            if !publishes_marker(&code) {
                // Every other shape the crate uses is inert: the constant's own
                // definition, a `use` of it, and reads of the bit. Anything
                // else is a shape this guard was not written for — most
                // importantly a publication that grew a line wrap, which would
                // otherwise just stop being scanned. Fail rather than skip.
                let inert = trimmed.starts_with("use ")
                    || trimmed.starts_with("const ")
                    || trimmed.starts_with("pub const ")
                    || trimmed.starts_with("pub(crate) const ")
                    || reads_marker(&code);
                if !inert {
                    violations.push(format!(
                        "{site}: `SLOT_WRITING` appears in a shape this guard does \
                         not recognise ({trimmed}). If a publication was wrapped \
                         across lines it is no longer being checked — teach \
                         `publishes_marker` the new shape rather than leaving the \
                         guard blind."
                    ));
                }
                continue;
            }

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
            // line that is actually code, and take that whole statement — the
            // fence may wrap too.
            let Some(end) = statement_end(&lines, i) else {
                violations.push(format!("{site}: marker store has no statement end"));
                continue;
            };
            let next = lines[end + 1..]
                .iter()
                .copied()
                .position(is_code_line)
                .map(|off| end + 1 + off);

            checked += 1;
            let Some(next) = next else {
                violations.push(format!(
                    "{site}: the WRITING marker is the last statement in the file \
                     — nothing fences the payload write that would follow it."
                ));
                continue;
            };
            let next_end = statement_end(&lines, next).unwrap_or(next);
            let next_stmt: String = lines[next..=next_end]
                .iter()
                .map(|l| code_only(l))
                .collect::<Vec<_>>()
                .join(" ");

            if !is_release_fence(&next_stmt) {
                violations.push(format!(
                    "{site}: the WRITING marker is not followed by a Release fence \
                     (next statement: {}). A Release store on the marker does not \
                     order the payload write below it, and `compiler_fence` does \
                     not either; see this file's header.",
                    next_stmt.trim()
                ));
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
