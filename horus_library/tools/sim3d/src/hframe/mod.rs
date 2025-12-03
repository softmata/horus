//! HFrame integration for Sim3D
//!
//! This module provides Bevy-compatible wrappers around HFrame, the high-performance
//! lock-free transform system. All coordinate transforms use f64 precision internally
//! for robotics-grade accuracy.
//!
//! # Usage
//!
//! ```rust,ignore
//! use sim3d::hframe::{BevyHFrame, BevyTransform};
//!
//! // In Bevy system
//! fn my_system(mut hframe: ResMut<BevyHFrame>) {
//!     hframe.register_frame("camera", Some("base_link")).unwrap();
//!     hframe.update_transform("camera", &BevyTransform::from_translation([0.0, 0.0, 0.5]));
//!     let tf = hframe.tf("camera", "world").unwrap();
//! }
//! ```

pub mod bevy_wrapper;
pub mod urdf_parser;

// Re-export wrapper types for convenience
pub use bevy_wrapper::render_hframe_frames;

// Primary type alias
pub use bevy_wrapper::HFrameTree;

// Re-export URDF parser

// Re-export core HFrame types for direct access
pub mod core {}
