/// Hardware driver integration example.
///
/// Demonstrates how to wire real hardware into horus:
/// - Load drivers from horus.toml [drivers] configuration
/// - Use Terra HAL for standard peripherals (motors, IMU, LiDAR)
/// - Create custom driver nodes for specialized hardware
/// - Bridge hardware data into the horus topic system
///
/// This example uses SIMULATED drivers (no real hardware required)
/// but shows the exact same patterns used for real deployments.
///
/// ROS2 equivalent: ros2_control, micro-ros-agent, hardware_interface

use horus::prelude::*;
use horus::DurationExt;

message! {
    /// Motor command (position or velocity)
    MotorCmd {
        joint_id: u32,
        /// 0 = position (radians), 1 = velocity (rad/s)
        mode: u32,
        value: f64,
    }

    /// Motor feedback
    MotorFeedback {
        joint_id: u32,
        position: f64,
        velocity: f64,
        current: f64,
        temperature: f64,
    }

    /// Gripper state
    GripperState {
        /// 0.0 = fully closed, 1.0 = fully open
        position: f64,
        /// Gripping force in Newtons
        force: f64,
        /// Object detected between fingers
        object_detected: u8,
    }
}

// ============================================================================
// Simulated Motor Driver — replaces a real Dynamixel driver
// ============================================================================

struct SimMotorDriver {
    cmd_sub: Topic<MotorCmd>,
    feedback_pub: Topic<MotorFeedback>,
    /// Simulated joint positions
    positions: [f64; 6],
    velocities: [f64; 6],
}

impl SimMotorDriver {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_sub: Topic::new("motor.cmd")?,
            feedback_pub: Topic::new("motor.feedback")?,
            positions: [0.0; 6],
            velocities: [0.0; 6],
        })
    }
}

impl Node for SimMotorDriver {
    fn name(&self) -> &str { "motor_driver" }

    fn init(&mut self) -> Result<()> {
        // In a real driver, this would:
        //   let hw = horus::drivers::load()?;
        //   let dxl = hw.dynamixel("motor")?;
        //   dxl.connect()?;
        //   dxl.set_torque_enable(true)?;
        hlog!(info, "Motor driver: initialized (simulated)");
        hlog!(info, "  Real driver would load from horus.toml [drivers.motor]");
        hlog!(info, "  terra = \"dynamixel\", port = \"/dev/ttyUSB0\"");
        Ok(())
    }

    fn tick(&mut self) {
        // Process commands
        while let Some(cmd) = self.cmd_sub.recv() {
            let id = cmd.joint_id as usize;
            if id < 6 {
                match cmd.mode {
                    0 => {
                        // Position mode: move toward target
                        let error = cmd.value - self.positions[id];
                        self.velocities[id] = error * 5.0; // P-control
                    }
                    1 => {
                        // Velocity mode
                        self.velocities[id] = cmd.value;
                    }
                    _ => {}
                }
            }
        }

        // Simulate physics (simple integration)
        let dt = 1.0 / 200.0;
        for i in 0..6 {
            self.positions[i] += self.velocities[i] * dt;
            // Damping
            self.velocities[i] *= 0.99;
        }

        // Publish feedback for all joints
        for i in 0..6 {
            self.feedback_pub.send(MotorFeedback {
                joint_id: i as u32,
                position: self.positions[i],
                velocity: self.velocities[i],
                current: self.velocities[i].abs() * 0.1, // simulated current
                temperature: 25.0 + self.velocities[i].abs() * 2.0,
            });
        }
    }

    fn shutdown(&mut self) -> Result<()> {
        // Real driver: disable torque, disconnect
        hlog!(info, "Motor driver: shutdown (torque disabled)");
        Ok(())
    }
}

// ============================================================================
// Simulated IMU Driver — replaces a real BNO055 driver
// ============================================================================

struct SimImuDriver {
    imu_pub: Topic<horus_library::messages::Imu>,
    tick_count: u64,
}

impl SimImuDriver {
    fn new() -> Result<Self> {
        Ok(Self {
            imu_pub: Topic::new("imu.data")?,
            tick_count: 0,
        })
    }
}

impl Node for SimImuDriver {
    fn name(&self) -> &str { "imu_driver" }

    fn init(&mut self) -> Result<()> {
        // Real driver: hw.bno055("imu")?.calibrate()?
        hlog!(info, "IMU driver: initialized (simulated BNO055)");
        Ok(())
    }

    fn tick(&mut self) {
        self.tick_count += 1;
        let t = self.tick_count as f64 * 0.005; // 200Hz

        // Simulate IMU readings with slight noise
        let mut imu = horus_library::messages::Imu::default();
        imu.linear_acceleration[0] = 0.01 * (t * 0.3).sin(); // slight sway
        imu.linear_acceleration[1] = 0.005 * (t * 0.7).cos();
        imu.linear_acceleration[2] = 9.81; // gravity
        imu.angular_velocity[0] = 0.001 * (t * 1.1).sin();
        imu.angular_velocity[1] = 0.001 * (t * 0.9).cos();
        imu.angular_velocity[2] = 0.0;

        self.imu_pub.send(imu);
    }
}

// ============================================================================
// Simulated Gripper Driver — custom hardware, not a Terra driver
// ============================================================================

struct SimGripperDriver {
    cmd_sub: Topic<f64>, // target position (0.0 = closed, 1.0 = open)
    state_pub: Topic<GripperState>,
    position: f64,
    force: f64,
}

impl SimGripperDriver {
    fn new() -> Result<Self> {
        Ok(Self {
            cmd_sub: Topic::new("gripper.cmd")?,
            state_pub: Topic::new("gripper.state")?,
            position: 1.0, // start open
            force: 0.0,
        })
    }
}

impl Node for SimGripperDriver {
    fn name(&self) -> &str { "gripper_driver" }

    fn init(&mut self) -> Result<()> {
        // Custom driver: direct GPIO/serial communication
        hlog!(info, "Gripper driver: initialized (simulated)");
        hlog!(info, "  Real driver: custom serial protocol over /dev/ttyACM0");
        Ok(())
    }

    fn tick(&mut self) {
        // Process gripper commands
        if let Some(target) = self.cmd_sub.recv() {
            let target = target.clamp(0.0, 1.0);
            // Simulate gripper movement (slow close, fast open)
            let speed = if target < self.position { 0.02 } else { 0.05 };
            let error = target - self.position;
            if error.abs() > 0.01 {
                self.position += error.signum() * speed;
            }
            // Simulate gripping force (only when closing on an object)
            self.force = if self.position < 0.3 { (0.3 - self.position) * 50.0 } else { 0.0 };
        }

        self.state_pub.send(GripperState {
            position: self.position,
            force: self.force,
            object_detected: if self.force > 2.0 { 1 } else { 0 },
        });
    }
}

// ============================================================================
// Application Controller — uses driver data for control
// ============================================================================

struct ArmController {
    motor_cmd_pub: Topic<MotorCmd>,
    motor_fb_sub: Topic<MotorFeedback>,
    gripper_cmd_pub: Topic<f64>,
    tick_count: u64,
}

impl ArmController {
    fn new() -> Result<Self> {
        Ok(Self {
            motor_cmd_pub: Topic::new("motor.cmd")?,
            motor_fb_sub: Topic::new("motor.feedback")?,
            gripper_cmd_pub: Topic::new("gripper.cmd")?,
            tick_count: 0,
        })
    }
}

impl Node for ArmController {
    fn name(&self) -> &str { "arm_controller" }

    fn tick(&mut self) {
        self.tick_count += 1;

        // Simple demo: wave joint 1 back and forth
        let t = self.tick_count as f64 * 0.02;
        let target = 0.5 * (t * 0.5).sin();

        self.motor_cmd_pub.send(MotorCmd {
            joint_id: 1,
            mode: 0, // position mode
            value: target,
        });

        // Open/close gripper periodically
        if self.tick_count % 200 < 100 {
            self.gripper_cmd_pub.send(0.0); // close
        } else {
            self.gripper_cmd_pub.send(1.0); // open
        }

        // Log motor feedback periodically
        if self.tick_count % 50 == 0 {
            while let Some(fb) = self.motor_fb_sub.recv() {
                if fb.joint_id == 1 {
                    hlog!(info, "Joint 1: pos={:.3} vel={:.3} temp={:.1}°C",
                        fb.position, fb.velocity, fb.temperature);
                }
            }
        }
    }
}

// ============================================================================
// Main
// ============================================================================

fn main() -> Result<()> {
    hlog!(info, "Driver Integration Example");
    hlog!(info, "  Demonstrates: hardware drivers, Terra HAL, custom drivers");
    hlog!(info, "  Drivers configured in horus.toml [drivers] section");
    hlog!(info, "  All drivers SIMULATED — same patterns work with real hardware");
    hlog!(info, "");

    // In a real deployment, you would load drivers from config:
    //
    //   let hw = horus::drivers::load()?;
    //   let motor = hw.dynamixel("motor")?;
    //   let imu = hw.bno055("imu")?;
    //   let lidar = hw.rplidar("lidar")?;
    //
    // The drivers are configured in horus.toml:
    //   [drivers]
    //   motor = { terra = "dynamixel", port = "/dev/ttyUSB0" }
    //   imu = { terra = "bno055", bus = "/dev/i2c-1" }

    let mut scheduler = Scheduler::new().tick_rate(200_u64.hz());

    // Driver nodes — high priority, fast rate
    scheduler.add(SimMotorDriver::new()?).order(0).rate(200_u64.hz()).build()?;
    scheduler.add(SimImuDriver::new()?).order(1).rate(200_u64.hz()).build()?;
    scheduler.add(SimGripperDriver::new()?).order(2).rate(50_u64.hz()).build()?;

    // Application node — uses driver data
    scheduler.add(ArmController::new()?).order(50).rate(50_u64.hz()).build()?;

    scheduler.run()
}
