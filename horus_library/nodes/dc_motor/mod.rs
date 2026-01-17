use crate::PwmCommand;
use horus_core::error::HorusResult;

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use horus_core::{Node, NodeInfo, NodeInfoExt, Topic};
use std::time::{SystemTime, UNIX_EPOCH};

// GPIO/PWM hardware support (Raspberry Pi)
#[cfg(feature = "gpio-hardware")]
use rppal::gpio::{Gpio, OutputPin};
#[cfg(feature = "gpio-hardware")]
use rppal::pwm::{Channel, Polarity, Pwm};

/// Motor driver backend type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MotorDriver {
    Simulation,
    L298N,      // Dual H-bridge: 2 enable PWM, 4 direction pins
    TB6612,     // Dual H-bridge: 2 PWM, 4 direction, 1 standby
    DRV8833,    // Dual H-bridge: dual PWM per motor (no separate direction)
    GenericPwm, // Generic PWM + 2 direction pins per motor
}

/// DC Motor Controller Node - PWM-based DC motor control
///
/// Controls DC motors using PWM signals for speed and direction control.
/// Compatible with common motor drivers (L298N, TB6612, DRV8833, etc.).
/// Supports multiple motor channels with independent control.
///
/// # Supported Motor Drivers
/// - **L298N**: Dual H-bridge, up to 2A per channel, 5-35V
/// - **TB6612**: Dual H-bridge, up to 1.2A per channel, 4.5-13.5V
/// - **DRV8833**: Dual H-bridge, up to 1.5A per channel, 2.7-10.8V
/// - **Generic**: Any PWM-capable motor driver
///
/// # Hardware Interface
/// - L298N: IN1, IN2 (direction), ENA (PWM) for motor A
/// - TB6612: AIN1, AIN2 (direction), PWMA (PWM) for motor A
/// - DRV8833: AIN1, AIN2 (both PWM for speed+direction) for motor A
///
/// # Features
/// - Hardware PWM support via Raspberry Pi GPIO
/// - Configurable duty cycle limits and dead zones
/// - Direction inversion per channel
/// - Command timeout with auto-stop
/// - Feedback publishing
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::{DcMotorNode, MotorDriver};
///
/// let mut motor = DcMotorNode::new()?;
/// motor.set_driver(MotorDriver::TB6612);
/// motor.set_num_channels(2);
/// motor.set_gpio_pins(0, 12, 16, 20); // Motor A: PWM=12, DIR1=16, DIR2=20
/// motor.set_pwm_frequency(20000); // 20kHz
/// ```
pub struct DcMotorNode {
    subscriber: Topic<PwmCommand>,
    publisher: Topic<PwmCommand>, // Echo commands for monitoring

    // Configuration
    num_channels: u8,
    driver: MotorDriver,
    max_duty_cycle: f32,   // Limit maximum speed (0.0-1.0)
    min_duty_cycle: f32,   // Dead zone compensation
    pwm_frequency: u32,    // PWM frequency in Hz
    invert_channels: u8,   // Bitfield for channel inversion
    enable_feedback: bool, // Publish motor feedback

    // State tracking per channel (up to 8 channels)
    current_duty_cycles: [f32; 8],
    current_enabled: [bool; 8],
    last_command_time: [u64; 8],
    command_timeout_ms: u64, // Auto-stop after timeout

    // Hardware GPIO/PWM (per motor channel)
    #[cfg(feature = "gpio-hardware")]
    pwm_channels: [Option<Pwm>; 8], // PWM for speed control
    #[cfg(feature = "gpio-hardware")]
    dir1_pins: [Option<OutputPin>; 8], // Direction pin 1
    #[cfg(feature = "gpio-hardware")]
    dir2_pins: [Option<OutputPin>; 8], // Direction pin 2
    #[cfg(feature = "gpio-hardware")]
    standby_pin: Option<OutputPin>, // TB6612 standby pin (shared)
    hardware_enabled: bool,
    gpio_pin_numbers: [(u8, u8, u8); 8], // (pwm_pin, dir1_pin, dir2_pin) per motor
    standby_gpio_pin: u8,                // TB6612 standby pin number (0 = not used)
}

impl DcMotorNode {
    /// Create a new DC motor node with default topic "motor_cmd"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("motor_cmd")
    }

    /// Create a new DC motor node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        #[cfg(feature = "gpio-hardware")]
        const NONE_PWM: Option<Pwm> = None;
        #[cfg(feature = "gpio-hardware")]
        const NONE_PIN: Option<OutputPin> = None;
        Ok(Self {
            subscriber: Topic::new(topic)?,
            publisher: Topic::new(&format!("{}_feedback", topic))?,
            num_channels: 2, // Default to 2 motors (typical robot)
            driver: MotorDriver::Simulation,
            max_duty_cycle: 1.0,
            min_duty_cycle: 0.0,
            pwm_frequency: 20000, // 20kHz default (ultrasonic for quiet operation)
            invert_channels: 0,
            enable_feedback: true,
            current_duty_cycles: [0.0; 8],
            current_enabled: [false; 8],
            last_command_time: [0; 8],
            command_timeout_ms: 1000, // 1 second timeout
            #[cfg(feature = "gpio-hardware")]
            pwm_channels: [NONE_PWM; 8],
            #[cfg(feature = "gpio-hardware")]
            dir1_pins: [NONE_PIN; 8],
            #[cfg(feature = "gpio-hardware")]
            dir2_pins: [NONE_PIN; 8],
            #[cfg(feature = "gpio-hardware")]
            standby_pin: None,
            hardware_enabled: false,
            gpio_pin_numbers: [(0, 0, 0); 8],
            standby_gpio_pin: 0,
        })
    }

    /// Set motor driver type
    pub fn set_driver(&mut self, driver: MotorDriver) {
        self.driver = driver;
    }

    /// Configure GPIO pins for a motor channel
    /// - pwm_pin: PWM output for speed control
    /// - dir1_pin: Direction control pin 1
    /// - dir2_pin: Direction control pin 2
    pub fn set_gpio_pins(&mut self, channel: u8, pwm_pin: u8, dir1_pin: u8, dir2_pin: u8) {
        if channel < 8 {
            self.gpio_pin_numbers[channel as usize] = (pwm_pin, dir1_pin, dir2_pin);
        }
    }

    /// Set TB6612 standby pin (active high, shared across both motors)
    pub fn set_standby_pin(&mut self, gpio_pin: u8) {
        self.standby_gpio_pin = gpio_pin;
    }

    /// Set the number of motor channels (1-8)
    pub fn set_num_channels(&mut self, channels: u8) {
        self.num_channels = channels.clamp(1, 8);
    }

    /// Set duty cycle limits (for safety or motor protection)
    pub fn set_duty_cycle_limits(&mut self, min: f32, max: f32) {
        self.min_duty_cycle = min.clamp(0.0, 1.0);
        self.max_duty_cycle = max.clamp(0.0, 1.0);
    }

    /// Set PWM frequency in Hz (typical range: 1kHz-20kHz)
    pub fn set_pwm_frequency(&mut self, frequency: u32) {
        self.pwm_frequency = frequency;
    }

    /// Invert a specific channel (swap forward/reverse)
    pub fn set_channel_inverted(&mut self, channel: u8, inverted: bool) {
        if channel < 8 {
            if inverted {
                self.invert_channels |= 1 << channel;
            } else {
                self.invert_channels &= !(1 << channel);
            }
        }
    }

    /// Set command timeout in milliseconds (0 = disable)
    pub fn set_command_timeout(&mut self, timeout_ms: u64) {
        self.command_timeout_ms = timeout_ms;
    }

    /// Enable/disable motor feedback publishing
    pub fn set_feedback_enabled(&mut self, enabled: bool) {
        self.enable_feedback = enabled;
    }

    /// Get current duty cycle for a channel
    pub fn get_duty_cycle(&self, channel: u8) -> Option<f32> {
        if channel < 8 {
            Some(self.current_duty_cycles[channel as usize])
        } else {
            None
        }
    }

    /// Check if a channel is enabled
    pub fn is_channel_enabled(&self, channel: u8) -> bool {
        if channel < 8 {
            self.current_enabled[channel as usize]
        } else {
            false
        }
    }

    // ========== Hardware Backend Functions ==========

    /// Initialize GPIO/PWM hardware for a motor channel
    #[cfg(feature = "gpio-hardware")]
    fn init_hardware(
        &mut self,
        channel: u8,
        mut ctx: Option<&mut NodeInfo>,
    ) -> std::io::Result<()> {
        let idx = channel as usize;
        let (pwm_pin, dir1_pin, dir2_pin) = self.gpio_pin_numbers[idx];

        if pwm_pin == 0 || dir1_pin == 0 || dir2_pin == 0 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("GPIO pins not configured for motor {}", channel),
            ));
        }

        // Initialize PWM channel
        let pwm_channel = match pwm_pin {
            12 | 13 => Channel::Pwm0,
            18 | 19 => Channel::Pwm1,
            _ => {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("Invalid PWM GPIO pin {} (use 12, 13, 18, or 19)", pwm_pin),
                ));
            }
        };

        let pwm = Pwm::with_frequency(
            pwm_channel,
            self.pwm_frequency as f64,
            0.0, // Start at 0% duty cycle
            Polarity::Normal,
            true, // enabled
        )
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        self.pwm_channels[idx] = Some(pwm);

        // Initialize direction pins
        let gpio = Gpio::new().map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;

        let dir1 = gpio
            .get(dir1_pin)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?
            .into_output();
        self.dir1_pins[idx] = Some(dir1);

        let dir2 = gpio
            .get(dir2_pin)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?
            .into_output();
        self.dir2_pins[idx] = Some(dir2);

        // Initialize standby pin for TB6612 (if configured)
        if self.driver == MotorDriver::TB6612
            && self.standby_gpio_pin != 0
            && self.standby_pin.is_none()
        {
            let standby = gpio
                .get(self.standby_gpio_pin)
                .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?
                .into_output();
            self.standby_pin = Some(standby);
            // Set standby high to enable driver
            if let Some(ref mut pin) = self.standby_pin {
                pin.set_high();
            }
        }

        ctx.log_info(&format!(
            "Initialized {:?} driver for motor {}: PWM={}, DIR1={}, DIR2={}",
            self.driver, channel, pwm_pin, dir1_pin, dir2_pin
        ));

        Ok(())
    }

    /// Control motor via hardware GPIO/PWM
    #[cfg(feature = "gpio-hardware")]
    fn set_motor_hardware(
        &mut self,
        channel: u8,
        duty_cycle: f32,
        enable: bool,
    ) -> std::io::Result<()> {
        let idx = channel as usize;

        let pwm = self.pwm_channels[idx].as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "PWM not initialized")
        })?;

        let dir1 = self.dir1_pins[idx].as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "DIR1 pin not initialized")
        })?;

        let dir2 = self.dir2_pins[idx].as_mut().ok_or_else(|| {
            std::io::Error::new(std::io::ErrorKind::NotConnected, "DIR2 pin not initialized")
        })?;

        if !enable {
            // Disable motor - coast or brake depending on driver
            match self.driver {
                MotorDriver::L298N | MotorDriver::TB6612 | MotorDriver::GenericPwm => {
                    // Coast: both direction pins low, PWM off
                    dir1.set_low();
                    dir2.set_low();
                    pwm.set_duty_cycle(0.0)
                        .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
                }
                MotorDriver::DRV8833 => {
                    // Coast: both PWM pins low
                    dir1.set_low();
                    dir2.set_low();
                }
                MotorDriver::Simulation => {}
            }
            return Ok(());
        }

        // Determine direction and speed
        let speed = duty_cycle.abs().clamp(0.0, 1.0);
        let forward = duty_cycle >= 0.0;

        match self.driver {
            MotorDriver::L298N | MotorDriver::TB6612 | MotorDriver::GenericPwm => {
                // Standard H-bridge: direction pins control direction, PWM controls speed
                if forward {
                    dir1.set_high();
                    dir2.set_low();
                } else {
                    dir1.set_low();
                    dir2.set_high();
                }
                pwm.set_duty_cycle((speed * 100.0) as f64)
                    .map_err(|e| std::io::Error::new(std::io::ErrorKind::Other, e))?;
            }
            MotorDriver::DRV8833 => {
                // DRV8833: dual PWM for speed and direction
                // Forward: AIN1=PWM, AIN2=LOW
                // Reverse: AIN1=LOW, AIN2=PWM
                // Brake: AIN1=HIGH, AIN2=HIGH
                if speed < 0.01 {
                    // Brake
                    dir1.set_high();
                    dir2.set_high();
                } else if forward {
                    // Forward: PWM on dir1, dir2 low
                    dir2.set_low();
                    // For DRV8833, we can't use hardware PWM on both pins easily
                    // Simplified: use dir1 as PWM approximation
                    if speed > 0.5 {
                        dir1.set_high();
                    } else {
                        dir1.set_low();
                    }
                } else {
                    // Reverse: PWM on dir2, dir1 low
                    dir1.set_low();
                    if speed > 0.5 {
                        dir2.set_high();
                    } else {
                        dir2.set_low();
                    }
                }
            }
            MotorDriver::Simulation => {}
        }

        Ok(())
    }

    /// Process a motor command
    fn process_command(&mut self, mut cmd: PwmCommand, mut ctx: Option<&mut NodeInfo>) {
        let channel = cmd.channel_id;

        // Validate channel
        if channel >= self.num_channels {
            ctx.log_warning(&format!("Invalid channel ID: {}", channel));
            return;
        }

        // Apply channel inversion
        if (self.invert_channels & (1 << channel)) != 0 {
            cmd.duty_cycle = -cmd.duty_cycle;
        }

        // Apply duty cycle limits
        let duty = cmd.duty_cycle.abs();
        let limited_duty = if duty < self.min_duty_cycle {
            0.0 // Below minimum, treat as stopped
        } else {
            duty.clamp(self.min_duty_cycle, self.max_duty_cycle)
        };

        cmd.duty_cycle = if cmd.duty_cycle >= 0.0 {
            limited_duty
        } else {
            -limited_duty
        };

        // Update state
        let idx = channel as usize;
        self.current_duty_cycles[idx] = cmd.duty_cycle;
        self.current_enabled[idx] = cmd.enable;
        self.last_command_time[idx] = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        // Try hardware first, fall back to simulation
        #[cfg(feature = "gpio-hardware")]
        if self.driver != MotorDriver::Simulation
            && (self.hardware_enabled || self.pwm_channels[idx].is_some())
        {
            // Initialize hardware if needed
            if self.pwm_channels[idx].is_none() {
                let (pwm_pin, dir1_pin, dir2_pin) = self.gpio_pin_numbers[idx];
                if pwm_pin != 0 && dir1_pin != 0 && dir2_pin != 0 {
                    if let Err(e) = self.init_hardware(channel, ctx.as_deref_mut()) {
                        // Failed to initialize - log detailed error (only once per channel)
                        if self.last_command_time[idx]
                            == SystemTime::now()
                                .duration_since(UNIX_EPOCH)
                                .unwrap()
                                .as_millis() as u64
                        {
                            ctx.log_warning(&format!(
                                "DcMotorNode motor {}: Hardware unavailable - using SIMULATION mode",
                                channel
                            ));
                            ctx.log_warning(&format!(
                                "  Tried GPIO pins: PWM={}, DIR1={}, DIR2={}",
                                pwm_pin, dir1_pin, dir2_pin
                            ));
                            ctx.log_warning(&format!("  Error: {}", e));
                            ctx.log_warning("  Fix:");
                            ctx.log_warning("    1. Install: sudo apt install libraspberrypi-dev");
                            ctx.log_warning("    2. Use hardware PWM pins: GPIO 12, 13, 18, or 19");
                            ctx.log_warning("    3. Check motor driver wiring");
                            ctx.log_warning(
                                "    4. Rebuild with: cargo build --features=\"gpio-hardware\"",
                            );
                        }
                        self.hardware_enabled = false;
                    } else {
                        self.hardware_enabled = true;
                    }
                }
            }

            // Try hardware control
            if self.hardware_enabled && self.pwm_channels[idx].is_some() {
                match self.set_motor_hardware(channel, cmd.duty_cycle, cmd.enable) {
                    Ok(()) => {
                        ctx.log_debug(&format!(
                            "Motor {} (HW): duty={:.1}%, enable={}, driver={:?}",
                            channel,
                            cmd.duty_cycle * 100.0,
                            cmd.enable,
                            self.driver
                        ));
                        // Publish feedback if enabled
                        if self.enable_feedback {
                            let _ = self.publisher.send(cmd, &mut None);
                        }
                        return;
                    }
                    Err(_e) => {
                        // Hardware error, fall back to simulation
                        self.hardware_enabled = false;
                    }
                }
            }
        }

        // Simulation fallback
        ctx.log_debug(&format!(
            "Motor {} (SIM): duty={:.1}%, enable={}, freq={}Hz",
            channel,
            cmd.duty_cycle * 100.0,
            cmd.enable,
            cmd.frequency
        ));

        // Publish feedback if enabled
        if self.enable_feedback {
            let _ = self.publisher.send(cmd, &mut None);
        }
    }

    /// Check for command timeouts and stop motors if needed
    fn check_timeouts(&mut self, mut ctx: Option<&mut NodeInfo>) {
        if self.command_timeout_ms == 0 {
            return; // Timeout disabled
        }

        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        for channel in 0..self.num_channels {
            let idx = channel as usize;
            if self.current_enabled[idx] {
                let elapsed = current_time - self.last_command_time[idx];
                if elapsed > self.command_timeout_ms {
                    // Timeout - stop motor
                    self.current_duty_cycles[idx] = 0.0;
                    self.current_enabled[idx] = false;

                    ctx.log_warning(&format!(
                        "Motor {} stopped due to command timeout ({}ms)",
                        channel, elapsed
                    ));

                    // Publish stop command
                    if self.enable_feedback {
                        let stop_cmd = PwmCommand::coast(channel);
                        let _ = self.publisher.send(stop_cmd, &mut None);
                    }
                }
            }
        }
    }

    /// Emergency stop all motors
    pub fn emergency_stop(&mut self) {
        for channel in 0..self.num_channels {
            let idx = channel as usize;
            self.current_duty_cycles[idx] = 0.0;
            self.current_enabled[idx] = false;

            // Publish stop command
            if self.enable_feedback {
                let stop_cmd = PwmCommand::brake(channel);
                let _ = self.publisher.send(stop_cmd, &mut None);
            }
        }
    }
}

impl Node for DcMotorNode {
    fn name(&self) -> &'static str {
        "DcMotorNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Process all pending commands
        while let Some(cmd) = self.subscriber.recv(&mut None) {
            self.process_command(cmd, ctx.as_deref_mut());
        }

        // Check for command timeouts
        self.check_timeouts(ctx.as_deref_mut());
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("DcMotorNode shutting down - stopping all motors");

        // Emergency stop all motors (uses brake mode for safety)
        self.emergency_stop();

        ctx.log_info("All DC motors stopped safely");
        Ok(())
    }
}
