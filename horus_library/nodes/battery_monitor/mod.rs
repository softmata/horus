use crate::BatteryState;
use horus_core::error::HorusResult;

type Result<T> = HorusResult<T>;
use horus_core::{Hub, Node, NodeInfo, NodeInfoExt};
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

// I2C hardware support for fuel gauges/power monitors
#[cfg(feature = "i2c-hardware")]
use i2cdev::core::I2CDevice;
#[cfg(feature = "i2c-hardware")]
use i2cdev::linux::LinuxI2CDevice;

/// Battery Monitor Node
///
/// Monitors battery voltage, current, charge state, and cell health.
/// Supports various battery chemistries and monitoring interfaces.
///
/// # Supported Hardware
/// - I2C/SMBus battery fuel gauges (BQ27441, MAX17043, LC709203F)
/// - Analog voltage/current sensors (INA219, INA226, ACS712)
/// - Power modules with telemetry (PM07, PM06, Matek systems)
/// - Battery Management Systems (BMS) via UART/CAN
/// - Direct ADC sampling with voltage dividers and current shunts
///
/// # Battery Types
/// - LiPo/LiFePO4: 1S-6S+ configurations
/// - Li-ion: 18650, 21700, etc.
/// - NiMH: 6-12 cell packs
/// - Lead-acid: 6V, 12V, 24V systems
/// - Custom battery packs
///
/// # Features
/// - Voltage monitoring with cell-level detection
/// - Current sensing (charge/discharge)
/// - State of charge (SOC) estimation
/// - Remaining capacity and runtime calculation
/// - Temperature monitoring
/// - Low battery warnings and critical alerts
/// - Charge cycle counting
/// - Cell balancing status
///
/// # Hybrid Pattern
///
/// ```rust,ignore
/// let node = BatteryMonitorNode::builder()
///     .with_filter(|state| {
///         // Only publish when battery level changes significantly
///         Some(state)
///     })
///     .build()?;
/// ```
///
/// # Example
/// ```rust,ignore
/// use horus_library::nodes::BatteryMonitorNode;
///
/// let mut battery = BatteryMonitorNode::new()?;
/// battery.set_cell_count(3); // 3S LiPo
/// battery.set_capacity(5000.0); // 5000 mAh
/// battery.set_low_voltage_threshold(10.5); // 3.5V per cell
/// battery.set_critical_voltage_threshold(9.9); // 3.3V per cell
/// ```
pub struct BatteryMonitorNode<P = PassThrough<BatteryState>>
where
    P: Processor<BatteryState>,
{
    publisher: Hub<BatteryState>,

    // Processor for hybrid pattern
    processor: P,

    // Configuration
    cell_count: u8,
    nominal_voltage_per_cell: f32, // V (3.7V for LiPo, 3.2V for LiFePO4, 1.2V for NiMH)
    capacity_mah: f32,             // mAh
    chemistry: BatteryChemistry,
    monitor_interface: MonitorInterface,

    // Voltage thresholds
    full_voltage: f32,     // V (fully charged)
    nominal_voltage: f32,  // V (nominal/average)
    low_voltage: f32,      // V (low warning)
    critical_voltage: f32, // V (critical shutdown)

    // Current state
    voltage: f32,             // V
    current: f32,             // A (negative = discharging)
    charge_mah: f32,          // mAh remaining
    percentage: f32,          // %
    temperature: f32,         // °C
    cell_voltages: [f32; 16], // Individual cell voltages
    power_supply_status: u8,
    _cycle_count: u32, // Reserved for battery health tracking

    // Monitoring
    sampling_rate: f32, // Hz
    enable_cell_monitoring: bool,
    last_sample_time: u64,
    voltage_history: [f32; 10], // Moving average
    history_index: usize,

    // Alerts
    low_battery_warned: bool,
    critical_battery_warned: bool,
    over_current_warned: bool,
    over_temperature_warned: bool,
    max_current: f32,     // A
    max_temperature: f32, // °C

    // Hardware I2C device (INA219/INA226 current/voltage sensor)
    #[cfg(feature = "i2c-hardware")]
    i2c_device: Option<LinuxI2CDevice>,
    hardware_enabled: bool,
    i2c_address: u16,           // I2C address (0x40 default for INA219)
    i2c_bus: u8,                // I2C bus number (default 1)
    shunt_resistance_mohm: f32, // Shunt resistor value in milliohms (100mΩ default)

    // Timing state (moved from static mut for thread safety)
    last_log_time: u64,
}

/// Battery chemistry type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BatteryChemistry {
    LiPo,     // Lithium Polymer
    LiFePO4,  // Lithium Iron Phosphate
    LiIon,    // Lithium Ion
    NiMH,     // Nickel Metal Hydride
    LeadAcid, // Lead Acid
    Custom,
}

/// Monitoring interface type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MonitorInterface {
    I2C,       // I2C fuel gauge
    Analog,    // ADC sampling
    UART,      // Serial BMS
    CAN,       // CAN bus BMS
    PWM,       // PWM power module
    Simulated, // Software simulation
}

impl BatteryMonitorNode {
    /// Create a new battery monitor node
    pub fn new() -> Result<Self> {
        Self::new_with_topic("battery")
    }

    /// Create a new battery monitor with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        let mut node = Self {
            publisher: Hub::new(topic)?,
            cell_count: 3,                 // 3S default
            nominal_voltage_per_cell: 3.7, // LiPo default
            capacity_mah: 5000.0,
            chemistry: BatteryChemistry::LiPo,
            monitor_interface: MonitorInterface::Simulated,
            full_voltage: 12.6,    // 4.2V × 3
            nominal_voltage: 11.1, // 3.7V × 3
            low_voltage: 10.5,     // 3.5V × 3
            critical_voltage: 9.9, // 3.3V × 3
            voltage: 11.1,
            current: 0.0,
            charge_mah: 5000.0,
            percentage: 100.0,
            temperature: 25.0,
            cell_voltages: [0.0; 16],
            power_supply_status: BatteryState::STATUS_DISCHARGING,
            _cycle_count: 0,
            sampling_rate: 1.0, // 1 Hz default
            enable_cell_monitoring: false,
            last_sample_time: 0,
            voltage_history: [11.1; 10],
            history_index: 0,
            low_battery_warned: false,
            critical_battery_warned: false,
            over_current_warned: false,
            over_temperature_warned: false,
            max_current: 100.0,    // 100A default max
            max_temperature: 60.0, // 60°C default max
            #[cfg(feature = "i2c-hardware")]
            i2c_device: None,
            hardware_enabled: false,
            i2c_address: 0x40,            // Default INA219 address
            i2c_bus: 1,                   // Default I2C bus
            shunt_resistance_mohm: 100.0, // 100mΩ default shunt
            last_log_time: 0,
            processor: PassThrough::new(),
        };

        node.update_voltage_thresholds();
        Ok(node)
    }

    /// Create a builder for advanced configuration
    pub fn builder() -> BatteryMonitorNodeBuilder<PassThrough<BatteryState>> {
        BatteryMonitorNodeBuilder::new()
    }

    /// Set number of cells in series
    pub fn set_cell_count(&mut self, count: u8) {
        self.cell_count = count.clamp(1, 16);
        self.update_voltage_thresholds();
    }

    /// Set battery capacity in mAh
    pub fn set_capacity(&mut self, capacity_mah: f32) {
        self.capacity_mah = capacity_mah;
        self.charge_mah = capacity_mah; // Reset to full
    }

    /// Set battery chemistry
    pub fn set_chemistry(&mut self, chemistry: BatteryChemistry) {
        self.chemistry = chemistry;
        self.nominal_voltage_per_cell = match chemistry {
            BatteryChemistry::LiPo => 3.7,
            BatteryChemistry::LiFePO4 => 3.2,
            BatteryChemistry::LiIon => 3.6,
            BatteryChemistry::NiMH => 1.2,
            BatteryChemistry::LeadAcid => 2.0,
            BatteryChemistry::Custom => self.nominal_voltage_per_cell,
        };
        self.update_voltage_thresholds();
    }

    /// Set monitoring interface
    pub fn set_monitor_interface(&mut self, interface: MonitorInterface) {
        self.monitor_interface = interface;
    }

    /// Set custom voltage thresholds
    pub fn set_voltage_thresholds(&mut self, full: f32, nominal: f32, low: f32, critical: f32) {
        self.full_voltage = full;
        self.nominal_voltage = nominal;
        self.low_voltage = low;
        self.critical_voltage = critical;
    }

    /// Set low voltage threshold
    pub fn set_low_voltage_threshold(&mut self, voltage: f32) {
        self.low_voltage = voltage;
    }

    /// Set critical voltage threshold
    pub fn set_critical_voltage_threshold(&mut self, voltage: f32) {
        self.critical_voltage = voltage;
    }

    /// Set maximum current threshold
    pub fn set_max_current(&mut self, amps: f32) {
        self.max_current = amps;
    }

    /// Set maximum temperature threshold
    pub fn set_max_temperature(&mut self, celsius: f32) {
        self.max_temperature = celsius;
    }

    /// Set sampling rate in Hz
    pub fn set_sampling_rate(&mut self, rate: f32) {
        self.sampling_rate = rate.clamp(0.1, 100.0);
    }

    /// Enable individual cell voltage monitoring
    pub fn enable_cell_monitoring(&mut self, enable: bool) {
        self.enable_cell_monitoring = enable;
    }

    /// Update voltage thresholds based on chemistry
    fn update_voltage_thresholds(&mut self) {
        let cells = self.cell_count as f32;
        match self.chemistry {
            BatteryChemistry::LiPo => {
                self.full_voltage = 4.2 * cells;
                self.nominal_voltage = 3.7 * cells;
                self.low_voltage = 3.5 * cells;
                self.critical_voltage = 3.3 * cells;
            }
            BatteryChemistry::LiFePO4 => {
                self.full_voltage = 3.65 * cells;
                self.nominal_voltage = 3.2 * cells;
                self.low_voltage = 3.0 * cells;
                self.critical_voltage = 2.5 * cells;
            }
            BatteryChemistry::LiIon => {
                self.full_voltage = 4.2 * cells;
                self.nominal_voltage = 3.6 * cells;
                self.low_voltage = 3.4 * cells;
                self.critical_voltage = 3.0 * cells;
            }
            BatteryChemistry::NiMH => {
                self.full_voltage = 1.4 * cells;
                self.nominal_voltage = 1.2 * cells;
                self.low_voltage = 1.0 * cells;
                self.critical_voltage = 0.9 * cells;
            }
            BatteryChemistry::LeadAcid => {
                self.full_voltage = 2.15 * cells;
                self.nominal_voltage = 2.0 * cells;
                self.low_voltage = 1.85 * cells;
                self.critical_voltage = 1.75 * cells;
            }
            BatteryChemistry::Custom => {}
        }
    }

    /// Estimate state of charge from voltage (reserved for alternative SOC method)
    fn _estimate_soc_from_voltage(&self, voltage: f32) -> f32 {
        // Simple linear interpolation between full and empty
        let range = self.full_voltage - self.critical_voltage;
        if range <= 0.0 {
            return 50.0;
        }

        let normalized = (voltage - self.critical_voltage) / range;
        (normalized * 100.0).clamp(0.0, 100.0)
    }

    /// Update charge based on current integration (Coulomb counting)
    fn update_charge(&mut self, dt: f32) {
        if self.current.abs() < 0.01 {
            return; // Negligible current
        }

        // Convert current (A) and time (s) to charge (mAh)
        let delta_charge = (self.current * dt * 1000.0) / 3600.0; // A·s to mAh
        self.charge_mah += delta_charge;

        // Clamp to valid range
        self.charge_mah = self.charge_mah.clamp(0.0, self.capacity_mah);

        // Update percentage
        self.percentage = (self.charge_mah / self.capacity_mah * 100.0).clamp(0.0, 100.0);

        // Detect charging state
        if self.current > 0.1 {
            self.power_supply_status = BatteryState::STATUS_CHARGING;
        } else if self.current < -0.1 {
            self.power_supply_status = BatteryState::STATUS_DISCHARGING;
        } else {
            if self.percentage >= 99.0 {
                self.power_supply_status = BatteryState::STATUS_FULL;
            } else {
                self.power_supply_status = BatteryState::STATUS_UNKNOWN;
            }
        }
    }

    /// Apply voltage filtering
    fn filter_voltage(&mut self, raw_voltage: f32) -> f32 {
        self.voltage_history[self.history_index] = raw_voltage;
        self.history_index = (self.history_index + 1) % self.voltage_history.len();

        // Calculate moving average
        let sum: f32 = self.voltage_history.iter().sum();
        sum / self.voltage_history.len() as f32
    }

    /// Configure I2C hardware interface
    pub fn set_i2c_config(&mut self, bus: u8, address: u16, shunt_mohm: f32) {
        self.i2c_bus = bus;
        self.i2c_address = address;
        self.shunt_resistance_mohm = shunt_mohm;
    }

    // ========== Hardware Backend Functions ==========

    /// Initialize I2C hardware (INA219/INA226 power monitor)
    #[cfg(feature = "i2c-hardware")]
    fn init_i2c_hardware(&mut self, mut ctx: Option<&mut NodeInfo>) -> std::io::Result<()> {
        let device_path = format!("/dev/i2c-{}", self.i2c_bus);
        let mut device = LinuxI2CDevice::new(&device_path, self.i2c_address)?;

        // INA219 Configuration register (0x00)
        // Config: Bus voltage range = 16V (0x0), Gain = /1 40mV (0x0)
        //         Bus ADC = 12-bit (0x3), Shunt ADC = 12-bit (0x3)
        //         Mode = Shunt and Bus continuous (0x7)
        // Value: 0x019F (bus voltage range 16V, gain /1, 12-bit, continuous)
        let config: [u8; 3] = [0x00, 0x01, 0x9F]; // Register 0x00, config 0x019F
        device.write(&config)?;

        self.i2c_device = Some(device);
        self.hardware_enabled = true;

        ctx.log_info(&format!(
            "Initialized I2C power monitor: bus {}, addr 0x{:02X}, shunt {}mΩ",
            self.i2c_bus, self.i2c_address, self.shunt_resistance_mohm
        ));

        Ok(())
    }

    /// Read voltage and current from I2C hardware (INA219)
    #[cfg(feature = "i2c-hardware")]
    fn read_i2c_hardware(&mut self) -> std::io::Result<(f32, f32)> {
        let device = self.i2c_device.as_mut().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::NotConnected,
                "I2C device not initialized",
            )
        })?;

        // Read bus voltage (register 0x02)
        let reg_bus: [u8; 1] = [0x02];
        device.write(&reg_bus)?;
        let mut bus_data = [0u8; 2];
        device.read(&mut bus_data)?;
        let bus_raw = u16::from_be_bytes(bus_data) >> 3; // 13-bit value, shift right 3
        let voltage = (bus_raw as f32) * 0.004; // LSB = 4mV

        // Read shunt voltage (register 0x01)
        let reg_shunt: [u8; 1] = [0x01];
        device.write(&reg_shunt)?;
        let mut shunt_data = [0u8; 2];
        device.read(&mut shunt_data)?;
        let shunt_raw = i16::from_be_bytes(shunt_data); // Signed 16-bit
        let shunt_voltage_mv = (shunt_raw as f32) * 0.01; // LSB = 10μV

        // Calculate current: I = V_shunt / R_shunt
        let current = shunt_voltage_mv / self.shunt_resistance_mohm;

        Ok((voltage, current))
    }

    /// Simulate battery measurements
    fn simulate_measurement(&mut self, _dt: f32) {
        // Simulate voltage drop based on load
        let load_factor = (self.current.abs() / 50.0).min(1.0); // Assume 50A nominal
        let voltage_drop = load_factor * 0.5; // Up to 0.5V drop under load

        // Simulate discharge curve
        let soc_factor = self.percentage / 100.0;
        let base_voltage = self.critical_voltage
            + (self.full_voltage - self.critical_voltage) * soc_factor.powf(1.2);

        self.voltage = base_voltage - voltage_drop;

        // Simulate cell voltages (if enabled)
        if self.enable_cell_monitoring && self.cell_count > 0 {
            let avg_cell_voltage = self.voltage / self.cell_count as f32;
            for i in 0..self.cell_count as usize {
                // Add small variations between cells
                let variation = (i as f32 - (self.cell_count / 2) as f32) * 0.02;
                self.cell_voltages[i] = avg_cell_voltage + variation;
            }
        }

        // Simulate temperature (increases with current)
        let ambient = 25.0;
        let heating = (self.current.abs() / 10.0) * 10.0; // 10°C per 10A
        self.temperature = ambient + heating;
    }

    /// Check for alert conditions
    fn check_alerts(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Low voltage warning
        if self.voltage <= self.low_voltage && !self.low_battery_warned {
            self.low_battery_warned = true;
            ctx.log_warning(&format!(
                "Low battery: {:.2}V ({:.0}%)",
                self.voltage, self.percentage
            ));
        } else if self.voltage > self.low_voltage + 0.2 {
            self.low_battery_warned = false;
        }

        // Critical voltage alert
        if self.voltage <= self.critical_voltage && !self.critical_battery_warned {
            self.critical_battery_warned = true;
            ctx.log_error(&format!(
                "CRITICAL BATTERY: {:.2}V ({:.0}%) - SHUTDOWN IMMINENT",
                self.voltage, self.percentage
            ));
        }

        // Over-current warning
        if self.current.abs() > self.max_current && !self.over_current_warned {
            self.over_current_warned = true;
            ctx.log_warning(&format!(
                "Over-current: {:.1}A (limit: {:.1}A)",
                self.current.abs(),
                self.max_current
            ));
        } else if self.current.abs() < self.max_current * 0.9 {
            self.over_current_warned = false;
        }

        // Over-temperature warning
        if self.temperature > self.max_temperature && !self.over_temperature_warned {
            self.over_temperature_warned = true;
            ctx.log_warning(&format!(
                "Battery over-temperature: {:.1}°C (limit: {:.1}°C)",
                self.temperature, self.max_temperature
            ));
        } else if self.temperature < self.max_temperature - 5.0 {
            self.over_temperature_warned = false;
        }
    }

    /// Publish battery state
    fn publish_state(&mut self, mut ctx: Option<&mut NodeInfo>) {
        let state = BatteryState {
            voltage: self.voltage,
            current: self.current,
            charge: self.charge_mah / 1000.0, // Convert to Ah
            capacity: self.capacity_mah / 1000.0,
            percentage: self.percentage,
            power_supply_status: self.power_supply_status,
            temperature: self.temperature,
            cell_voltages: self.cell_voltages,
            cell_count: if self.enable_cell_monitoring {
                self.cell_count
            } else {
                0
            },
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        };

        // Process through pipeline and publish
        if let Some(processed) = self.processor.process(state) {
            if let Err(e) = self.publisher.send(processed, &mut None) {
                ctx.log_error(&format!("Failed to publish battery state: {:?}", e));
            }
        }
    }

    /// Get current battery state
    pub fn get_state(&self) -> BatteryState {
        BatteryState {
            voltage: self.voltage,
            current: self.current,
            charge: self.charge_mah / 1000.0,
            capacity: self.capacity_mah / 1000.0,
            percentage: self.percentage,
            power_supply_status: self.power_supply_status,
            temperature: self.temperature,
            cell_voltages: self.cell_voltages,
            cell_count: if self.enable_cell_monitoring {
                self.cell_count
            } else {
                0
            },
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        }
    }

    /// Check if battery is healthy
    pub fn is_healthy(&self) -> bool {
        self.voltage > self.critical_voltage
            && self.current.abs() < self.max_current
            && self.temperature < self.max_temperature
    }

    /// Get estimated remaining time in seconds
    pub fn time_remaining(&self) -> Option<f32> {
        if self.current < -0.1 {
            // Discharging
            Some((self.charge_mah / (self.current.abs() * 1000.0)) * 3600.0)
        } else {
            None
        }
    }
}

impl<P> Node for BatteryMonitorNode<P>
where
    P: Processor<BatteryState>,
{
    fn name(&self) -> &'static str {
        "BatteryMonitorNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_start();
        ctx.log_info("BatteryMonitorNode initialized");
        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        self.processor.on_shutdown();
        ctx.log_info("BatteryMonitorNode shutting down");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.processor.on_tick();
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64;

        // Calculate time since last sample
        let sample_interval_ns = (1_000_000_000.0 / self.sampling_rate) as u64;
        if current_time - self.last_sample_time < sample_interval_ns {
            return;
        }

        let dt = (current_time - self.last_sample_time) as f32 / 1_000_000_000.0;
        self.last_sample_time = current_time;

        // Try hardware first, fall back to simulation
        #[cfg(feature = "i2c-hardware")]
        if (self.hardware_enabled || self.i2c_device.is_some())
            && self.monitor_interface == MonitorInterface::I2C
        {
            // Initialize I2C if needed
            if self.i2c_device.is_none() && self.i2c_address != 0 {
                if let Err(e) = self.init_i2c_hardware(ctx.as_deref_mut()) {
                    // Provide detailed troubleshooting information (only log once)
                    if self.hardware_enabled || self.last_sample_time == 0 {
                        let device_path = format!("/dev/i2c-{}", self.i2c_bus);
                        ctx.log_warning(&format!(
                            "BatteryMonitorNode: Hardware unavailable - using SIMULATION mode"
                        ));
                        ctx.log_warning(&format!(
                            "  Tried: {} address 0x{:02X}",
                            device_path, self.i2c_address
                        ));
                        ctx.log_warning(&format!("  Error: {}", e));
                        ctx.log_warning("  Fix:");
                        ctx.log_warning("    1. Install: sudo apt install i2c-tools");
                        ctx.log_warning(
                            "    2. Enable I2C: sudo raspi-config -> Interface Options -> I2C",
                        );
                        ctx.log_warning("    3. Verify INA219/INA226 wiring and address");
                        ctx.log_warning("    4. Test with: i2cdetect -y 1");
                        ctx.log_warning(
                            "    5. Rebuild with: cargo build --features=\"i2c-hardware\"",
                        );
                    }
                    self.hardware_enabled = false;
                } else {
                    self.hardware_enabled = true;
                }
            }

            // Try hardware measurement
            if self.hardware_enabled && self.i2c_device.is_some() {
                match self.read_i2c_hardware() {
                    Ok((voltage, current)) => {
                        self.voltage = voltage;
                        self.current = current;
                    }
                    Err(_e) => {
                        // Hardware error, fall back to simulation
                        self.hardware_enabled = false;
                        self.simulate_measurement(dt);
                    }
                }
            } else {
                self.simulate_measurement(dt);
            }
        } else {
            // No I2C hardware or different interface - use simulation
            self.simulate_measurement(dt);
        }

        #[cfg(not(feature = "i2c-hardware"))]
        {
            // No hardware support compiled in - use simulation
            self.simulate_measurement(dt);
        }

        // Filter voltage
        self.voltage = self.filter_voltage(self.voltage);

        // Update charge estimate
        self.update_charge(dt);

        // Check for alerts
        self.check_alerts(ctx.as_deref_mut());

        // Publish state
        self.publish_state(ctx.as_deref_mut());

        // Periodic status logging
        let log_interval = 10_000_000_000; // 10 seconds
        if current_time - self.last_log_time > log_interval {
            let status = match self.power_supply_status {
                BatteryState::STATUS_CHARGING => "CHARGING",
                BatteryState::STATUS_DISCHARGING => "DISCHARGING",
                BatteryState::STATUS_FULL => "FULL",
                _ => "UNKNOWN",
            };

            let time_str = if let Some(time) = self.time_remaining() {
                format!("{:.0}min remaining", time / 60.0)
            } else {
                String::from("N/A")
            };

            ctx.log_info(&format!(
                "Battery: {:.2}V ({:.0}%) {:.1}A {:.1}°C {} | {}",
                self.voltage, self.percentage, self.current, self.temperature, status, time_str
            ));

            self.last_log_time = current_time;
        }
    }
}

/// Preset configurations for common battery types
impl BatteryMonitorNode {
    /// Configure for 3S LiPo battery (11.1V nominal)
    pub fn configure_3s_lipo(&mut self, capacity_mah: f32) {
        self.set_chemistry(BatteryChemistry::LiPo);
        self.set_cell_count(3);
        self.set_capacity(capacity_mah);
    }

    /// Configure for 4S LiPo battery (14.8V nominal)
    pub fn configure_4s_lipo(&mut self, capacity_mah: f32) {
        self.set_chemistry(BatteryChemistry::LiPo);
        self.set_cell_count(4);
        self.set_capacity(capacity_mah);
    }

    /// Configure for 6S LiPo battery (22.2V nominal)
    pub fn configure_6s_lipo(&mut self, capacity_mah: f32) {
        self.set_chemistry(BatteryChemistry::LiPo);
        self.set_cell_count(6);
        self.set_capacity(capacity_mah);
    }

    /// Configure for 12V lead-acid battery
    pub fn configure_12v_lead_acid(&mut self, capacity_mah: f32) {
        self.set_chemistry(BatteryChemistry::LeadAcid);
        self.set_cell_count(6);
        self.set_capacity(capacity_mah);
    }

    /// Configure for LiFePO4 battery
    pub fn configure_lifepo4(&mut self, cells: u8, capacity_mah: f32) {
        self.set_chemistry(BatteryChemistry::LiFePO4);
        self.set_cell_count(cells);
        self.set_capacity(capacity_mah);
    }
}

/// Builder for BatteryMonitorNode with fluent API for processor configuration
pub struct BatteryMonitorNodeBuilder<P>
where
    P: Processor<BatteryState>,
{
    topic: String,
    processor: P,
}

impl BatteryMonitorNodeBuilder<PassThrough<BatteryState>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "battery".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl<P> BatteryMonitorNodeBuilder<P>
where
    P: Processor<BatteryState>,
{
    /// Set the topic for publishing battery state
    pub fn topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> BatteryMonitorNodeBuilder<P2>
    where
        P2: Processor<BatteryState>,
    {
        BatteryMonitorNodeBuilder {
            topic: self.topic,
            processor,
        }
    }

    /// Add a closure processor for transformations
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> BatteryMonitorNodeBuilder<ClosureProcessor<BatteryState, BatteryState, F>>
    where
        F: FnMut(BatteryState) -> BatteryState + Send + 'static,
    {
        BatteryMonitorNodeBuilder {
            topic: self.topic,
            processor: ClosureProcessor::new(f),
        }
    }

    /// Add a filter processor
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> BatteryMonitorNodeBuilder<FilterProcessor<BatteryState, BatteryState, F>>
    where
        F: FnMut(BatteryState) -> Option<BatteryState> + Send + 'static,
    {
        BatteryMonitorNodeBuilder {
            topic: self.topic,
            processor: FilterProcessor::new(f),
        }
    }

    /// Chain another processor in a pipeline
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> BatteryMonitorNodeBuilder<Pipeline<BatteryState, BatteryState, BatteryState, P, P2>>
    where
        P2: Processor<BatteryState, Output = BatteryState>,
        P: Processor<BatteryState, Output = BatteryState>,
    {
        BatteryMonitorNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the BatteryMonitorNode
    #[cfg(feature = "i2c-hardware")]
    pub fn build(self) -> Result<BatteryMonitorNode<P>> {
        let mut node = BatteryMonitorNode {
            publisher: Hub::new(&self.topic)?,
            processor: self.processor,
            cell_count: 3,
            nominal_voltage_per_cell: 3.7,
            capacity_mah: 5000.0,
            chemistry: BatteryChemistry::LiPo,
            monitor_interface: MonitorInterface::Simulated,
            full_voltage: 12.6,
            nominal_voltage: 11.1,
            low_voltage: 10.5,
            critical_voltage: 9.9,
            voltage: 11.1,
            current: 0.0,
            charge_mah: 5000.0,
            percentage: 100.0,
            temperature: 25.0,
            cell_voltages: [0.0; 16],
            power_supply_status: BatteryState::STATUS_DISCHARGING,
            _cycle_count: 0,
            sampling_rate: 1.0,
            enable_cell_monitoring: false,
            last_sample_time: 0,
            voltage_history: [11.1; 10],
            history_index: 0,
            low_battery_warned: false,
            critical_battery_warned: false,
            over_current_warned: false,
            over_temperature_warned: false,
            max_current: 100.0,
            max_temperature: 60.0,
            i2c_device: None,
            hardware_enabled: false,
            i2c_address: 0x40,
            i2c_bus: 1,
            shunt_resistance_mohm: 100.0,
            last_log_time: 0,
        };
        node.update_voltage_thresholds();
        Ok(node)
    }

    /// Build the BatteryMonitorNode (non-i2c version)
    #[cfg(not(feature = "i2c-hardware"))]
    pub fn build(self) -> Result<BatteryMonitorNode<P>> {
        let mut node = BatteryMonitorNode {
            publisher: Hub::new(&self.topic)?,
            processor: self.processor,
            cell_count: 3,
            nominal_voltage_per_cell: 3.7,
            capacity_mah: 5000.0,
            chemistry: BatteryChemistry::LiPo,
            monitor_interface: MonitorInterface::Simulated,
            full_voltage: 12.6,
            nominal_voltage: 11.1,
            low_voltage: 10.5,
            critical_voltage: 9.9,
            voltage: 11.1,
            current: 0.0,
            charge_mah: 5000.0,
            percentage: 100.0,
            temperature: 25.0,
            cell_voltages: [0.0; 16],
            power_supply_status: BatteryState::STATUS_DISCHARGING,
            _cycle_count: 0,
            sampling_rate: 1.0,
            enable_cell_monitoring: false,
            last_sample_time: 0,
            voltage_history: [11.1; 10],
            history_index: 0,
            low_battery_warned: false,
            critical_battery_warned: false,
            over_current_warned: false,
            over_temperature_warned: false,
            max_current: 100.0,
            max_temperature: 60.0,
            hardware_enabled: false,
            i2c_address: 0x40,
            i2c_bus: 1,
            shunt_resistance_mohm: 100.0,
            last_log_time: 0,
        };
        node.update_voltage_thresholds();
        Ok(node)
    }
}
