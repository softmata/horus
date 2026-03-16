//! Hardware device discovery — serial ports, cameras, input devices.
//!
//! Provides cross-platform enumeration of connected hardware:
//! - **Linux**: `/dev/ttyUSB*`, `/dev/video*`, `/dev/input/js*`
//! - **macOS**: `/dev/cu.*`, IOKit device enumeration
//! - **Windows**: SetupDi API, COM port enumeration, DirectShow
