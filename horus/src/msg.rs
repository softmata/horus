//! Standard message types, under one path — the same `horus::msg::` spelling
//! C++ uses.
//!
//! ```rust
//! use horus::msg::{CmdVel, Imu, Pose2D};
//! ```
//!
//! The cross-language compatibility table used to give the same seven wire
//! types three unrelated Rust paths — `horus_robotics::CmdVel`,
//! `horus_robotics::messages::sensor::Imu`, `horus_types::Pose2D` — while C++
//! wrote `horus::msg::X` for all of them and Python wrote `horus.X`. Which
//! crate a message happens to live in is a packaging decision (`horus_robotics`
//! and `horus_tf` are separate git repositories), and it was leaking into every
//! user's import lines: moving between the three languages meant re-learning
//! the location of every type.
//!
//! [`crate::prelude`] already pulls them all in, but a glob is not a path. It cannot
//! be written in a type position, and it tells a reader nothing about where
//! `Imu` came from. [`crate::types`] was the near miss that made the guess worse: it
//! existed, it sounded canonical, and it held only the geometry half, so
//! `horus::types::CmdVel` failed with `E0425` instead of pointing anywhere
//! useful. That module now mirrors this one, so neither guess dead-ends.
//!
//! Every name C++ declares across `<horus/msg/*.hpp>` resolves here — the
//! `msg_covers_the_whole_cpp_namespace` test below is that header list, so the
//! two namespaces cannot drift apart silently. The pool-backed payloads and
//! the transform POD are here too even though C++ files them one namespace up,
//! in plain `horus::`: `Image`, `PointCloud`, `DepthImage` and `Transform` are
//! in this crate's own "Key Message Types" table, so a reader who just watched
//! `horus::msg::CmdVel` work will type `horus::msg::Image` next, and getting
//! `E0425` there would rebuild the very trap [`crate::types`] used to be.
//!
//! What is deliberately *not* here is the machinery underneath those payloads:
//! `Tensor`, `TensorDtype`, `Device` and the raw `*Descriptor` structs. They
//! are not things you put in a `Topic`, they are what the pool types are built
//! out of, and they live in [`crate::types`] — which re-exports this module in full,
//! so `horus::types::` resolves for everything below plus those. Saying so out
//! loud is the point: an exclusion a reader only discovers as an `E0425` is the
//! same trap this module was written to remove.
//!
//! Nothing moved: these are re-exports, so every path that worked before still
//! works. This only makes one of them canonical, and picks the one the other
//! bindings already use.
//!
//! This module is glob-safe next to the prelude — you can write both, and the
//! one name where the two source crates disagree resolves the same way under
//! either:
//!
//! ```rust
//! use horus::prelude::*;
//! use horus::msg::*;
//!
//! // The action-server lifecycle enum, the meaning `horus::prelude` has
//! // always given the bare name.
//! let lifecycle: GoalStatus = GoalStatus::Succeeded;
//! assert!(lifecycle.is_terminal());
//!
//! // The navigation status code stored in `GoalResult::status` keeps the
//! // module path the docs already teach for it.
//! assert_eq!(navigation::GoalStatus::Succeeded as u8, 2);
//! ```

/// Robotics wire types — sensor, control, navigation, detection, vision,
/// force, input, perception and simulation. Rust ships a few that have no
/// C++ counterpart yet — the simulation service messages, and the vision
/// types `<horus/msg/vision.hpp>` deliberately declines to publish.
pub use horus_robotics::messages::*;

/// Math, diagnostics, time and generic messages. This is `horus_types`'
/// curated prelude rather than its crate root, because the root also
/// re-exports the crate's own module names (`math`, `time`, …) and those
/// are an implementation detail of where the types are filed, which is
/// exactly what this module exists to stop leaking.
pub use horus_types::prelude::*;

/// The zero-copy payloads. C++ declares these in `horus::` rather than
/// `horus::msg::` (`<horus/pool.hpp>`), and `DepthImage` has no C++ type at
/// all, so parity offers no answer here — but they are messages in every
/// sense that matters to a caller: they are what a `Topic<Image>` carries.
/// Leaving them out made `horus::msg::Image` an `E0425` sitting two lines
/// below a working `horus::msg::LaserScan`.
pub use horus_core::memory::{DepthImage, Image, PointCloud};

/// The element types you need to fill the payloads above —
/// `Image::new(w, h, ImageEncoding::Rgb8)`, `PointCloud::from_xyz(&[PointXYZ])`
/// — for the same reason: a namespace that hands you `PointCloud` but not
/// `PointXYZ` has only moved the missing import one line down.
pub use horus_core::types::{ImageEncoding, PointXYZ, PointXYZI, PointXYZRGB};

/// The rigid transform. C++ writes `horus::Transform` (`<horus/transform.hpp>`),
/// one namespace up, and Rust used to require a hand-added `horus_tf` git
/// dependency to name it at all.
pub use horus_tf::Transform;

/// `GoalStatus` is the one name the two source crates disagree about.
/// `horus_core::actions::GoalStatus` is the action-server lifecycle enum
/// (`Pending`/`Active`/`Succeeded`/`Aborted`/`Canceled`/`Preempted`/`Rejected`);
/// `horus_robotics::messages::navigation::GoalStatus` is the `#[repr(u8)]`
/// code stored in `GoalResult::status`, with different variants. C++ never
/// had to choose — `<horus/msg/navigation.hpp>` publishes `GOAL_STATUS_*`
/// constants and puts the enum class in `horus::`, not `horus::msg::` — so
/// there is no parity answer to copy.
///
/// Left to the glob above, this one name made `use horus::prelude::*;`
/// together with `use horus::msg::*;` a hard `E0659` ambiguity error
/// (`ambiguous_glob_imports` is deny-by-default), which is the one thing a
/// convenience namespace must not do to the prelude it sits beside. Pinning
/// it to the prelude's meaning is also the rule the documentation already
/// teaches: the bare name is the action enum, and the navigation code is
/// reached through its module — `horus::msg::navigation::GoalStatus`, the
/// exact shape of the `horus::prelude::navigation::GoalStatus` the
/// navigation reference has always told readers to write.
pub use horus_core::actions::GoalStatus;
