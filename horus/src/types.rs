//! Message and geometry types.
//!
//! This used to be a plain re-export of the `horus_types` crate, which left
//! half the types unreachable from the obvious path. `Tensor`, the point-cloud
//! extension traits and their neighbours live in `horus_core::types`, so
//!
//! ```rust,ignore
//! use horus::types::Tensor;              // error: no `Tensor` in the root
//! use horus::horus_core::types::Tensor;  // what the docs had to say instead
//! ```
//!
//! Reaching through `horus::horus_core::` defeats the point of a facade: it
//! names an internal crate the user was never meant to know about, and it is
//! not guessable — you find it by reading the source or by copying a doc
//! snippet. Both sets are re-exported here, so the obvious path is the one that
//! works.
//!
//! The same argument then stopped one level short. This module is called
//! "types", its own heading says "Message and geometry types", and it is the
//! path a reader guesses before they have heard of [`crate::msg`] — but it held only
//! the geometry half. `horus::types::Pose2D` resolved while
//! `horus::types::CmdVel`, `Imu`, `Odometry`, `LaserScan` and `NavGoal` all
//! failed with `E0425` and no hint about where to look instead. A path that
//! works for half the names you try is worse than one that does not exist at
//! all: it reads as confirmation that you are in the right module and the type
//! is simply missing. Everything in [`crate::msg`] is re-exported here too, so the
//! guess lands. [`crate::msg`] stays the spelling the documentation uses, because it
//! is the one C++ and Python already write.

pub use horus_core::types::*;
pub use horus_types::*;

// Glob-re-exporting [`crate::msg`] rather than naming the messages one by one is
// what keeps the two from drifting: whatever becomes reachable as
// `horus::msg::X` is reachable as `horus::types::X` by construction, so
// this module cannot silently fall back to holding half the set. Where the
// three globs overlap (`Pose2D`, `ImageEncoding`, `Quaternion`, …) they
// resolve to the same item, so no name becomes ambiguous.
pub use crate::msg::*;
