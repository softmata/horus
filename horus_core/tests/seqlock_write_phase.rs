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
///
/// Keyed on the file as well as the function name. An exemption is the one hole
/// in this guard, and an unqualified name would silently cover a second
/// `send_shm_sp_pod` defined in some other module. Moving the function is
/// expected to fail this test until the path below is updated: that is the
/// fail-closed direction to err in.
const EXEMPT_SITES: &[(&str, &str)] = &[
    ("src/communication/topic/dispatch.rs", "send_shm_sp_pod"),
    // A fixture, not a write phase. It stamps a mid-write marker and stops, to
    // prove a consumer does not read that marker as a lap; no payload store
    // follows, so there is nothing for a fence to order.
    (
        "src/communication/topic/tests.rs",
        "a_mid_write_marker_is_not_a_lap",
    ),
];

/// Lexer state, carried across line boundaries by `sanitize`.
#[derive(Clone, Copy)]
enum Scan {
    Code,
    /// Inside `/* */`, which Rust allows to nest.
    Block(u32),
    /// Inside `"..."`.
    Str,
    /// Inside a raw string, holding its hash count: `r##"` closes on `"##`.
    Raw(usize),
}

/// If a raw string opens at `i`, how many chars its opener spans and how many
/// hashes it carries.
///
/// `r"`, `r#"` and `br##"` open one; the `r` of `for`, and the raw identifier
/// `r#type`, do not.
fn raw_string_open(chars: &[char], i: usize) -> Option<(usize, usize)> {
    // Nothing may run into the `b`/`r`, or it is the tail of an identifier.
    if i.checked_sub(1)
        .and_then(|p| chars.get(p))
        .is_some_and(|c| c.is_alphanumeric() || *c == '_')
    {
        return None;
    }
    let mut j = i;
    if chars.get(j) == Some(&'b') {
        j += 1;
    }
    if chars.get(j) != Some(&'r') {
        return None;
    }
    j += 1;
    let hashes = chars[j.min(chars.len())..]
        .iter()
        .take_while(|c| **c == '#')
        .count();
    if chars.get(j + hashes) == Some(&'"') {
        Some((j + hashes + 1 - i, hashes))
    } else {
        None
    }
}

/// Whether the `hashes` chars at `from` are all `#`, closing a raw string.
fn closes_raw(chars: &[char], from: usize, hashes: usize) -> bool {
    (0..hashes).all(|k| chars.get(from + k) == Some(&'#'))
}

/// The file's lines with comments dropped and literals blanked, so the scanning
/// below sees punctuation that is code and nothing else. Numbering is
/// preserved: output line `i` is input line `i`.
///
/// Without this, a comment or a literal that happens to contain `;`, a bracket,
/// or the word `fence` steers the delimiter counting and the fence match. Both
/// directions of that are bad, and both were reachable when this ran per line
/// with no cross-line state:
///
/// * a `/* */` comment that merely *mentions* the constant was reported as an
///   unrecognised publication, failing CI on a comment;
/// * a payload written as `*ptr = msg;` was taken for the continuation line of
///   a block comment and skipped, so `marker; *ptr = msg; fence(Release)` — the
///   payload store left unordered with respect to the marker, exactly the bug
///   this file exists to reject — passed the guard.
///
/// The state is per file rather than per line because every construct that
/// matters here can span lines: block comments, ordinary strings continued with
/// a trailing `\`, and raw strings.
fn sanitize(src: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut state = Scan::Code;
    for line in src.lines() {
        let chars: Vec<char> = line.chars().collect();
        let mut buf = String::with_capacity(line.len());
        let mut i = 0;
        while i < chars.len() {
            match state {
                Scan::Block(depth) => {
                    if chars[i] == '/' && chars.get(i + 1) == Some(&'*') {
                        state = Scan::Block(depth + 1);
                        i += 2;
                    } else if chars[i] == '*' && chars.get(i + 1) == Some(&'/') {
                        state = if depth <= 1 {
                            Scan::Code
                        } else {
                            Scan::Block(depth - 1)
                        };
                        i += 2;
                    } else {
                        i += 1;
                    }
                }
                Scan::Str => match chars[i] {
                    // A trailing `\` continues the string onto the next line;
                    // `i` running past the end simply carries `Str` over.
                    '\\' => i += 2,
                    '"' => {
                        state = Scan::Code;
                        buf.push('_');
                        i += 1;
                    }
                    _ => i += 1,
                },
                Scan::Raw(hashes) => {
                    if chars[i] == '"' && closes_raw(&chars, i + 1, hashes) {
                        state = Scan::Code;
                        buf.push('_');
                        i += 1 + hashes;
                    } else {
                        i += 1;
                    }
                }
                Scan::Code => {
                    let c = chars[i];
                    if c == '/' && chars.get(i + 1) == Some(&'/') {
                        break;
                    }
                    if c == '/' && chars.get(i + 1) == Some(&'*') {
                        state = Scan::Block(1);
                        i += 2;
                        continue;
                    }
                    // Raw strings first: `\` is not an escape inside one, so
                    // scanning `r"C:\dir\"` as an ordinary string runs the
                    // closing quote past the end and blanks the rest of the
                    // file — a guard that checks nothing.
                    if let Some((skip, hashes)) = raw_string_open(&chars, i) {
                        state = Scan::Raw(hashes);
                        i += skip;
                        continue;
                    }
                    if c == '"' {
                        state = Scan::Str;
                        i += 1;
                        continue;
                    }
                    // `'` is a char literal only in `'x'` / `'\n'` shape;
                    // otherwise it opens a lifetime and must be left alone.
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
                            buf.push('_');
                            continue;
                        }
                    }
                    buf.push(c);
                    i += 1;
                }
            }
        }
        out.push(buf);
    }
    out
}

/// Name of the `fn` a line sits inside, searching upward. Returns `None` rather
/// than guessing, which makes an unrecognised site fail the test instead of
/// silently matching an exemption.
///
/// Runs on sanitized lines, so a `fn` written inside a comment cannot be
/// mistaken for the enclosing definition.
fn enclosing_fn(lines: &[String], line_idx: usize) -> Option<String> {
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
                // `sanitize` blanks the ABI string, so `extern "C" fn` reaches
                // this as `extern _ fn`.
                .or_else(|| rest.strip_prefix("extern _ "))
                .or_else(|| rest.strip_prefix("extern "));
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
/// The line that opens the wrapped publication whose argument list contains
/// `at`, if there is one.
///
/// `publishes_marker` matches a single line, so it stops recognising a
/// publication the moment the call wraps and `.store(` lands above its
/// `SLOT_WRITING` argument — which is exactly what rustfmt does once the
/// argument grows. The guard then reports the argument line as an unrecognised
/// shape instead of checking the publication.
///
/// Only consulted for a line that is not itself a publication, and only the
/// opening line of the call qualifies: it must carry the `.store(`/`publish(`,
/// and its statement must still be open at `at`. A merely *enclosing* block
/// does not match, or every publication inside one would be attributed to the
/// block's first line and the fence check would run against the wrong text.
fn wrapped_publication_start(lines: &[String], at: usize) -> Option<usize> {
    let lo = at.saturating_sub(WRAP_LOOKBACK);
    (lo..at).rev().find(|&j| {
        (lines[j].contains(".store(") || lines[j].contains("publish("))
            && statement_end(lines, j).is_some_and(|end| end >= at)
    })
}

/// How far above a `SLOT_WRITING` line to look for the `.store(` that owns it.
/// Generous: a wrapped publication spans a handful of lines, never dozens.
const WRAP_LOOKBACK: usize = 12;

fn statement_end(lines: &[String], start: usize) -> Option<usize> {
    let mut depth: i32 = 0;
    for (offset, line) in lines[start..].iter().enumerate() {
        for c in line.chars() {
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

/// Whether a sanitized line carries any code at all.
///
/// Comment bodies and blank lines sanitize to whitespace, so this does not have
/// to guess at comment shapes from punctuation — guessing is what let a
/// `*ptr = payload;` deref write be mistaken for the `*` continuation line of a
/// `/* */` block and skipped over on the way to finding the fence.
fn has_code(line: &str) -> bool {
    !line.trim().is_empty()
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
        let lines = sanitize(&src);
        let rel = file
            .strip_prefix(crate_root())
            .unwrap_or(file)
            .display()
            .to_string()
            .replace('\\', "/");

        for (i, code) in lines.iter().enumerate() {
            if !code.contains("SLOT_WRITING") {
                continue;
            }
            // A wrapped publication is still a publication: when this line is
            // only the argument list, evaluate from the line the call opened
            // on, so the fence check below sees the whole statement.
            let i = if publishes_marker(code) {
                i
            } else {
                wrapped_publication_start(&lines, i).unwrap_or(i)
            };
            let code = &lines[i];
            let site = format!("{rel}:{}", i + 1);
            let trimmed = code.trim();

            // Recognition runs against the whole statement, not the line: a
            // wrapped call carries `.store(` on its first line and
            // `SLOT_WRITING` on a later one, and neither line alone matches.
            let statement = match statement_end(&lines, i) {
                Some(end) => lines[i..=end].join(" "),
                None => code.to_string(),
            };

            if !publishes_marker(&statement) {
                // Every other shape the crate uses is inert: the constant's own
                // definition, a `use` of it, and reads of the bit. Anything
                // else is a shape this guard was not written for — most
                // importantly a publication that grew a line wrap, which would
                // otherwise just stop being scanned. Fail rather than skip.
                let inert = trimmed.starts_with("use ")
                    || trimmed.starts_with("const ")
                    || trimmed.starts_with("pub const ")
                    || trimmed.starts_with("pub(crate) const ")
                    || reads_marker(code);
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
                Some(name) if EXEMPT_SITES.iter().any(|(f, n)| *f == rel && *n == name) => continue,
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
                .position(|l| has_code(l))
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
            let next_stmt = lines[next..=next_end].join(" ");

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
