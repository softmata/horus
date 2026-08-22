//! The facade must not make users reach through it into the internal crates.
//!
//! `horus` re-exports `horus_types` as `horus::types`, which reads like the
//! place message and geometry types live. Half of them did not: `Tensor` and
//! the point-cloud extension traits are in `horus_core::types`, so the obvious
//! path failed —
//!
//! ```text
//! error[E0432]: unresolved import `horus::types::Tensor`
//!               no `Tensor` in the root
//! ```
//!
//! — and the documentation told readers to write `horus::horus_core::types::Tensor`
//! instead, at twelve separate sites. Naming an internal crate in a user's
//! import is the facade failing at the one job it has: `horus_core` is not
//! something a user should have to know exists, and the path is not guessable,
//! so it is found by reading the source or copying a snippet.
//!
//! `horus::types` now re-exports both sets. These tests are compile-time: if a
//! type stops being reachable from the documented path, this file fails to
//! build.
//!
//! Run: `cargo test -p horus --test import_paths`

// The path that used to fail. Each of these is a type the docs referenced via
// `horus::horus_core::`.
#[allow(unused_imports)]
mod reachable_from_the_obvious_path {
    pub use horus::types::Tensor;
}

/// The prelude is what the docs tell people to use for everything else; it must
/// keep working unchanged.
#[allow(unused_imports)]
mod prelude_still_works {
    pub use horus::prelude::*;
}

/// `horus_types`' own contents must not have been displaced by the merge.
#[allow(unused_imports)]
mod horus_types_still_reachable {
    pub use horus::types::Pose2D;
}

#[test]
fn the_documented_import_paths_compile() {
    // The assertions are the `use` statements above, checked at compile time.
    // This test exists so the file is run and reported.
}

/// A user should never have to name an internal crate. This is a lint on the
/// documentation rather than the code: if a doc page reintroduces
/// `horus::horus_core::`, the facade has a gap that should be closed by
/// re-exporting the type, not by documenting the long path.
#[test]
fn the_docs_do_not_route_users_through_horus_core() {
    let docs = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("horus-docs/content/docs"));

    let Some(root) = docs.filter(|p| p.is_dir()) else {
        eprintln!("SKIP: horus-docs is not checked out beside horus");
        return;
    };

    let mut offenders = Vec::new();
    let mut stack = vec![root.clone()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
                continue;
            }
            if !path.extension().is_some_and(|e| e == "mdx" || e == "md") {
                continue;
            }
            let Ok(text) = std::fs::read_to_string(&path) else {
                continue;
            };
            for (i, line) in text.lines().enumerate() {
                if line.contains("horus::horus_core::") {
                    let rel = path.strip_prefix(&root).unwrap_or(&path);
                    offenders.push(format!("{}:{}: {}", rel.display(), i + 1, line.trim()));
                }
            }
        }
    }

    assert!(
        offenders.is_empty(),
        "these doc lines route users through the internal crate:\n  {}\n\n\
         Re-export the type from `horus` instead — a user should not have to \
         know `horus_core` exists.",
        offenders.join("\n  ")
    );
}

// Every path the documentation now uses, checked at compile time. These are
// the ones that previously required naming `horus_core`.
#[allow(unused_imports)]
mod every_documented_path {
    pub use horus::communication::SendBlockingError;
    pub use horus::error::HorusResult as _R1;
    pub use horus::types::point::{PointXYZExt, PointXYZIExt, PointXYZRGBExt};
    pub use horus::types::Tensor;
    pub use horus::HorusResult as _R2;

    // Module paths the docs reference.
    #[allow(unused)]
    fn module_paths_resolve() {
        let _ = std::any::type_name::<horus::error::HorusError>();
    }
}

#[allow(unused_imports)]
mod macro_support_paths {
    // `topics!` expands to code that names these; the docs also reference them
    // directly, and both used to require `horus::horus_core::`.
    pub use horus::serde_yaml;
    pub use horus::topics;
}

/// The coordinate-frame tree must be reachable from `horus::prelude`.
///
/// `horus_py` and `horus_cpp` both depend on `horus-tf`, so Python and C++
/// users got the frame tree out of the box — C++ reaches `horus::TransformFrame`
/// straight from `<horus/horus.hpp>`, and tutorial 3 uses it there. The Rust
/// umbrella crate did not depend on it. `horus::prelude` therefore offered
/// `TransformStamped` — the message describing a transform — but not the tree
/// that produces one, so the language HORUS is written in was the only one of
/// the three that could not do this without hand-adding a pinned git
/// dependency.
///
/// Compile-time: if these stop being reachable from the prelude, this file
/// fails to build.
#[test]
fn transform_frame_is_reachable_from_the_prelude() {
    use horus::prelude::*;

    let tf = TransformFrame::new();
    let world = tf.register_frame("world", None).expect("register world");
    let base = tf
        .register_frame("base_link", Some("world"))
        .expect("register base_link under world");
    assert_ne!(world, base, "each frame gets its own id");

    // The value type travels with the tree, not just the stamped message.
    let _identity: Transform = Transform::default();
    let _config: TransformFrameConfig = TransformFrameConfig::default();

    // The message type was always reachable; keep it that way.
    let _stamped: Option<TransformStamped> = None;
}
