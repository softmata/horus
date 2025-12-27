//! Keyboard input drivers
//!
//! This module provides drivers for keyboard input.
//!
//! # Available Drivers
//!
//! - `SimulationKeyboardDriver` - Always available, generates synthetic input
//! - `CrosstermKeyboardDriver` - Terminal keyboard input via crossterm (requires `crossterm` feature)

mod simulation;

#[cfg(feature = "crossterm")]
mod crossterm_driver;

// Re-exports
pub use simulation::{SimulationKeyboardConfig, SimulationKeyboardDriver};

#[cfg(feature = "crossterm")]
pub use crossterm_driver::{CrosstermConfig, CrosstermKeyboardDriver};

use crate::KeyboardInput;
use horus_core::driver::DriverStatus;
use horus_core::error::HorusResult;

/// Standard keyboard keycodes
pub mod keycodes {
    // Letter keys (A–Z)
    pub const KEY_A: u32 = 65;
    pub const KEY_B: u32 = 66;
    pub const KEY_C: u32 = 67;
    pub const KEY_D: u32 = 68;
    pub const KEY_E: u32 = 69;
    pub const KEY_F: u32 = 70;
    pub const KEY_G: u32 = 71;
    pub const KEY_H: u32 = 72;
    pub const KEY_I: u32 = 73;
    pub const KEY_J: u32 = 74;
    pub const KEY_K: u32 = 75;
    pub const KEY_L: u32 = 76;
    pub const KEY_M: u32 = 77;
    pub const KEY_N: u32 = 78;
    pub const KEY_O: u32 = 79;
    pub const KEY_P: u32 = 80;
    pub const KEY_Q: u32 = 81;
    pub const KEY_R: u32 = 82;
    pub const KEY_S: u32 = 83;
    pub const KEY_T: u32 = 84;
    pub const KEY_U: u32 = 85;
    pub const KEY_V: u32 = 86;
    pub const KEY_W: u32 = 87;
    pub const KEY_X: u32 = 88;
    pub const KEY_Y: u32 = 89;
    pub const KEY_Z: u32 = 90;

    // Number keys (0–9)
    pub const KEY_0: u32 = 48;
    pub const KEY_1: u32 = 49;
    pub const KEY_2: u32 = 50;
    pub const KEY_3: u32 = 51;
    pub const KEY_4: u32 = 52;
    pub const KEY_5: u32 = 53;
    pub const KEY_6: u32 = 54;
    pub const KEY_7: u32 = 55;
    pub const KEY_8: u32 = 56;
    pub const KEY_9: u32 = 57;

    // Arrow keys
    pub const KEY_ARROW_UP: u32 = 38;
    pub const KEY_ARROW_DOWN: u32 = 40;
    pub const KEY_ARROW_LEFT: u32 = 37;
    pub const KEY_ARROW_RIGHT: u32 = 39;

    // Control / special keys
    pub const KEY_SPACE: u32 = 32;
    pub const KEY_ENTER: u32 = 13;
    pub const KEY_ESCAPE: u32 = 27;
    pub const KEY_TAB: u32 = 9;
    pub const KEY_BACKSPACE: u32 = 8;

    // Function keys
    pub const KEY_F1: u32 = 112;
    pub const KEY_F2: u32 = 113;
    pub const KEY_F3: u32 = 114;
    pub const KEY_F4: u32 = 115;
    pub const KEY_F5: u32 = 116;
    pub const KEY_F6: u32 = 117;
    pub const KEY_F7: u32 = 118;
    pub const KEY_F8: u32 = 119;
    pub const KEY_F9: u32 = 120;
    pub const KEY_F10: u32 = 121;
    pub const KEY_F11: u32 = 122;
    pub const KEY_F12: u32 = 123;
}

/// Enum of all available keyboard driver backends
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum KeyboardDriverBackend {
    /// Simulation driver (always available)
    #[default]
    Simulation,
    /// Crossterm-based terminal keyboard driver
    #[cfg(feature = "crossterm")]
    Crossterm,
}

/// Type-erased keyboard driver for runtime backend selection
pub enum KeyboardDriver {
    Simulation(SimulationKeyboardDriver),
    #[cfg(feature = "crossterm")]
    Crossterm(CrosstermKeyboardDriver),
}

impl KeyboardDriver {
    /// Create a new keyboard driver with the specified backend
    pub fn new(backend: KeyboardDriverBackend) -> HorusResult<Self> {
        match backend {
            KeyboardDriverBackend::Simulation => {
                Ok(Self::Simulation(SimulationKeyboardDriver::new()))
            }
            #[cfg(feature = "crossterm")]
            KeyboardDriverBackend::Crossterm => {
                Ok(Self::Crossterm(CrosstermKeyboardDriver::new()?))
            }
        }
    }

    /// Create a simulation driver (always available)
    pub fn simulation() -> Self {
        Self::Simulation(SimulationKeyboardDriver::new())
    }

    /// Poll for next key event (non-blocking)
    pub fn poll_event(&mut self) -> Option<KeyboardInput> {
        match self {
            Self::Simulation(d) => d.poll_event(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.poll_event(),
        }
    }

    // ========================================================================
    // Lifecycle methods
    // ========================================================================

    pub fn init(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.init(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.init(),
        }
    }

    pub fn shutdown(&mut self) -> HorusResult<()> {
        match self {
            Self::Simulation(d) => d.shutdown(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.shutdown(),
        }
    }

    pub fn is_available(&self) -> bool {
        match self {
            Self::Simulation(d) => d.is_available(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.is_available(),
        }
    }

    pub fn status(&self) -> DriverStatus {
        match self {
            Self::Simulation(d) => d.status(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.status(),
        }
    }

    // ========================================================================
    // Sensor methods
    // ========================================================================

    pub fn read(&mut self) -> HorusResult<KeyboardInput> {
        match self {
            Self::Simulation(d) => d.read(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.read(),
        }
    }

    pub fn has_data(&self) -> bool {
        match self {
            Self::Simulation(d) => d.has_data(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.has_data(),
        }
    }

    pub fn sample_rate(&self) -> Option<f32> {
        match self {
            Self::Simulation(d) => d.sample_rate(),
            #[cfg(feature = "crossterm")]
            Self::Crossterm(d) => d.sample_rate(),
        }
    }
}
