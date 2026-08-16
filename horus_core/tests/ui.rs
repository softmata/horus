//! Compile-time behaviour of the `message!` macro.
//!
//! `message!` is a tt-muncher whose entry rule, `($($input:tt)+)`, also matches
//! its own `@munch ...` output. Without a terminal failure arm any syntax slip
//! re-entered the entry rule forever and reported:
//!
//! ```text
//! error: recursion limit reached while expanding `$crate::message!`
//! = help: consider increasing the recursion limit
//! ```
//!
//! — no line inside the block, no mention of the real mistake, and a `help:`
//! that only makes the compiler spin longer before failing identically. Four of
//! the five most natural mistakes hit it: `pub` on a field, a missing comma, a
//! semicolon separator, and a tuple struct.
//!
//! The `fail/` cases lock in that each of those now produces one actionable
//! error. The `pass/` case exists so the failure arm cannot be tightened into
//! rejecting valid definitions — it covers serde-backed fields, `#[fixed]` POD
//! messages, multiple messages per invocation, and doc comments.
//!
//! Regenerate the expected stderr after an intentional message change with:
//!
//! ```text
//! TRYBUILD=overwrite cargo test -p horus_core --test ui
//! ```

#[test]
fn message_macro_rejects_malformed_definitions() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/fail/*.rs");
}

#[test]
fn message_macro_accepts_every_documented_form() {
    let t = trybuild::TestCases::new();
    t.pass("tests/ui/pass/*.rs");
}
