//! Hardware drivers for HORUS
//!
//! All hardware drivers have been extracted to dedicated crates for modularity:
//!
//! | Crate | Drivers |
//! |-------|---------|
//! | `horus-sensors` | IMU, Camera, Lidar, GPS, Encoder, Ultrasonic, Battery, Force/Torque |
//! | `horus-actuators` | DC Motor, BLDC, Stepper, Servo |
//! | `horus-industrial` | CAN, I2C, SPI, Serial, Modbus, DigitalIO |
//! | `horus-dynamixel` | Dynamixel smart servos |
//! | `horus-roboclaw` | RoboClaw motor controller |
//!
//! # Usage
//!
//! ```rust,ignore
//! // Import drivers from dedicated crates
//! use horus_sensors::{ImuDriver, CameraDriver, LidarDriver};
//! use horus_actuators::{MotorDriver, ServoDriver};
//! use horus_industrial::{SerialDriver, CanDriver};
//! ```
//!
//! # Driver Traits
//!
//! Driver traits are defined in `horus_core::driver`:
//! - `Driver` - Base driver trait
//! - `Sensor<Output=T>` - Sensor driver trait
//! - `Actuator<Command=T>` - Actuator driver trait
