//! A pending safe-state request must be consumed before a node is skipped.
//!
//! Every executor polls `honor_safe_state_request` once per pass. The property
//! that matters is not that the call exists — it is WHERE it sits relative to
//! the guards that `continue` past a node. Put it after them and the poll is
//! skipped for exactly the nodes the ladder is trying to safe: an isolated node
//! is one that is not ticking, and a stopped node is one holding whatever
//! setpoint it last commanded.
//!
//! Two orderings regressed that way and are pinned here:
//!
//!   * the RT executor skipped `is_stopped` nodes before the poll, while its own
//!     comment said the poll "runs before the pause/stop gates: an Isolated node
//!     must reach its safe state even if it is also about to be stopped";
//!   * the compute coordinator put the poll below its `ready_indices.is_empty()`
//!     early-`continue`, so on a pass with nothing ready — the steady state for
//!     a set of isolated or rate-limited nodes — no node was safed at all.
//!
//! These loops run on their own threads with a live `SharedMonitors` and cannot
//! be entered from a test, so this reads the source. That is the same shape as
//! `horus_manager/tests/monitor_plugin_lookup_contract.rs`, and it carries the
//! same lesson: comments are stripped first, because both bodies *describe*
//! the ordering in prose and a guard that matched the prose would be satisfied
//! by the very comment the code contradicts.

use std::fs;
use std::path::PathBuf;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("horus_core has a parent")
        .to_path_buf()
}

/// A file's source with every `//` comment line removed.
fn code_only(rel: &str) -> String {
    let src = fs::read_to_string(repo_root().join(rel)).unwrap_or_else(|e| panic!("{rel}: {e}"));
    src.lines()
        .filter(|l| !l.trim_start().starts_with("//"))
        .collect::<Vec<_>>()
        .join("\n")
}

/// The slice of `code` between the first occurrence of `from` and of `to`.
fn between(code: &str, from: &str, to: &str, what: &str) -> String {
    let a = code.find(from).unwrap_or_else(|| {
        panic!("{what}: anchor {from:?} is gone; this guard would cover nothing")
    });
    let b = code[a..]
        .find(to)
        .unwrap_or_else(|| panic!("{what}: anchor {to:?} is gone; this guard would cover nothing"));
    code[a..a + b].to_string()
}

#[test]
fn the_rt_executor_safes_a_node_before_it_skips_a_stopped_one() {
    let code = code_only("horus_core/src/scheduling/rt_executor.rs");

    // The per-node preamble of the tick loop: from the loop header to the first
    // budget check, which is well past both the safing poll and the stop gates.
    let body = between(
        &code,
        "for (idx, node) in nodes.iter_mut().enumerate() {",
        "tick_node(",
        "rt executor tick loop",
    );

    let safing = body
        .find("honor_safe_state_request")
        .expect("the RT executor must poll honor_safe_state_request in its tick loop");
    let stop_gate = body
        .find("node.is_stopped")
        .expect("the RT executor must still gate on is_stopped somewhere in the loop");

    assert!(
        safing < stop_gate,
        "the RT executor skips stopped nodes at offset {stop_gate} before polling \
         honor_safe_state_request at {safing}. Stopping a node halts its ticks; it \
         does not put its actuator anywhere, so a node stopped by BudgetPolicy::Enforce \
         or `horus node kill` keeps holding its last commanded setpoint through a \
         safe-state request. The loop's own comment says this poll runs BEFORE the \
         pause/stop gates."
    );
}

#[test]
fn the_compute_executor_safes_its_nodes_even_when_none_is_ready() {
    let code = code_only("horus_core/src/scheduling/compute_executor.rs");

    let body = between(
        &code,
        "if ready_indices.is_empty() {",
        "let now = Instant::now();",
        "compute coordinator loop",
    );

    assert!(
        !body.contains("honor_safe_state_request"),
        "the compute coordinator polls honor_safe_state_request AFTER its \
         `ready_indices.is_empty()` early-continue, so on any pass with nothing \
         ready no node is safed — and a set of isolated or rate-limited nodes is \
         permanently in that state. Under `safety.on_link_lost = \"safe_state\"`, \
         which deliberately does not latch an e-stop and leaves the scheduler \
         running, the request would never be consumed at all."
    );

    // ...and it is polled before that point.
    let preamble = between(
        &code,
        "ready_indices.push(i);",
        "if ready_indices.is_empty() {",
        "compute coordinator preamble",
    );
    assert!(
        preamble.contains("honor_safe_state_request"),
        "the compute coordinator must poll honor_safe_state_request before the \
         `ready_indices.is_empty()` early-continue"
    );
}
