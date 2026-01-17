// F1Tenth Adaptive Racing Controller
//
// This controller implements:
// 1. Gap-Following algorithm for exploration (first lap)
// 2. Racing line learning from LIDAR data
// 3. Pure Pursuit path following for optimized racing
// 4. Curriculum learning across multiple tracks
//
// Usage:
//   1. Start sim2d with a track: horus sim2d --world maps/track_01_oval.yaml --robot f1tenth_robot.yaml
//   2. Run this controller: cd tests/sim2d/f1tenth_race && horus run

use horus::prelude::*;
use std::collections::VecDeque;
use std::f64::consts::PI;

// Racing state machine
#[derive(Debug, Clone, Copy, PartialEq)]
enum RacingPhase {
    Exploration,    // First lap - explore and learn the track
    Learning,       // Build optimal racing line from data
    Racing,         // Execute optimized racing line
}

// Checkpoint for lap timing
#[derive(Debug, Clone)]
struct Checkpoint {
    pos: (f64, f64),
    radius: f64,
    passed: bool,
}

// Racing line point with metadata
#[derive(Debug, Clone)]
struct RacingLinePoint {
    pos: (f64, f64),
    optimal_speed: f64,
    curvature: f64,
}

// Learned track data
#[derive(Debug, Default)]
struct TrackMemory {
    // Points collected during exploration
    exploration_points: Vec<(f64, f64, f64)>, // (x, y, heading)
    // Optimized racing line
    racing_line: Vec<RacingLinePoint>,
    // Lap times for improvement tracking
    lap_times: Vec<f64>,
    // Best sectors
    best_sectors: Vec<f64>,
}

pub struct F1TenthRacer {
    // HORUS communication
    cmd_vel: Topic<CmdVel>,
    odom_sub: Topic<Odometry>,
    lidar_sub: Topic<LaserScan>,

    // State
    phase: RacingPhase,
    current_pose: (f64, f64, f64), // x, y, theta
    current_speed: f64,

    // Track memory
    memory: TrackMemory,

    // Lap tracking
    lap_count: u32,
    lap_start_time: f64,
    current_lap_time: f64,
    checkpoints: Vec<Checkpoint>,

    // Gap-following parameters (exploration)
    gap_threshold: f64,
    safety_distance: f64,

    // Pure pursuit parameters (racing)
    lookahead_distance: f64,
    min_lookahead: f64,
    max_lookahead: f64,
    wheelbase: f64,

    // Speed control
    max_speed: f64,
    min_speed: f64,

    // Racing line following
    racing_line_idx: usize,

    // Statistics
    tick_count: u64,
    total_distance: f64,
    collisions: u32,

    // Recent positions for path smoothing
    position_history: VecDeque<(f64, f64)>,
}

impl F1TenthRacer {
    pub fn new() -> HorusResult<Self> {
        println!("F1Tenth Racer: Initializing...");
        println!("Phase 1: Exploration - Learning the track");

        Ok(Self {
            cmd_vel: Topic::new("f1tenth.cmd_vel")?,
            odom_sub: Topic::new("f1tenth.odom")?,
            lidar_sub: Topic::new("f1tenth.scan")?,

            phase: RacingPhase::Exploration,
            current_pose: (0.0, 0.0, 0.0),
            current_speed: 0.0,

            memory: TrackMemory::default(),

            lap_count: 0,
            lap_start_time: 0.0,
            current_lap_time: 0.0,
            checkpoints: Vec::new(),

            // Gap following tuning
            gap_threshold: 2.0,    // meters
            safety_distance: 0.5,  // meters

            // Pure pursuit tuning
            lookahead_distance: 1.0,
            min_lookahead: 0.5,
            max_lookahead: 2.5,
            wheelbase: 0.32,

            // Speed limits
            max_speed: 6.0,  // Start conservative
            min_speed: 1.0,

            racing_line_idx: 0,

            tick_count: 0,
            total_distance: 0.0,
            collisions: 0,

            position_history: VecDeque::with_capacity(100),
        })
    }

    // ==================== Gap Following (Exploration) ====================

    fn find_best_gap(&self, ranges: &[f32]) -> (f64, f64) {
        // Find the widest gap in LIDAR data
        let num_rays = ranges.len();
        if num_rays == 0 {
            return (0.0, self.min_speed);
        }

        let angle_min = -2.356;  // -135 degrees
        let angle_increment = 4.712 / num_rays as f64;  // 270 degrees total

        // Find consecutive points far enough away
        let mut best_gap_start = 0;
        let mut best_gap_end = 0;
        let mut best_gap_width = 0;

        let mut current_gap_start = 0;
        let mut in_gap = false;

        for i in 0..num_rays {
            let range = ranges[i] as f64;

            if range > self.gap_threshold {
                if !in_gap {
                    current_gap_start = i;
                    in_gap = true;
                }
            } else {
                if in_gap {
                    let gap_width = i - current_gap_start;
                    if gap_width > best_gap_width {
                        best_gap_width = gap_width;
                        best_gap_start = current_gap_start;
                        best_gap_end = i;
                    }
                    in_gap = false;
                }
            }
        }

        // Check final gap
        if in_gap {
            let gap_width = num_rays - current_gap_start;
            if gap_width > best_gap_width {
                best_gap_start = current_gap_start;
                best_gap_end = num_rays;
            }
        }

        // Calculate steering angle towards gap center
        let gap_center = (best_gap_start + best_gap_end) / 2;
        let target_angle = angle_min + gap_center as f64 * angle_increment;

        // Determine safe speed based on closest obstacle
        let min_range = ranges.iter()
            .filter(|&&r| r > 0.1)  // Ignore invalid readings
            .cloned()
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(10.0) as f64;

        let safe_speed = if min_range < self.safety_distance {
            self.min_speed
        } else if min_range < 2.0 {
            self.min_speed + (min_range - self.safety_distance) * 2.0
        } else {
            self.max_speed * 0.6  // Exploration speed cap
        };

        (target_angle.clamp(-0.4, 0.4), safe_speed.clamp(self.min_speed, self.max_speed))
    }

    // ==================== Racing Line Learning ====================

    fn record_exploration_point(&mut self) {
        let (x, y, theta) = self.current_pose;

        // Only record if moved enough from last point
        if let Some(&(last_x, last_y, _)) = self.memory.exploration_points.last() {
            let dist = ((x - last_x).powi(2) + (y - last_y).powi(2)).sqrt();
            if dist < 0.2 {
                return;  // Too close to last point
            }
        }

        self.memory.exploration_points.push((x, y, theta));
    }

    fn build_racing_line(&mut self) {
        println!("\nBuilding optimized racing line from {} exploration points...",
                 self.memory.exploration_points.len());

        if self.memory.exploration_points.len() < 10 {
            println!("Not enough exploration data!");
            return;
        }

        // Smooth the path using moving average
        let window_size = 5;
        let mut smoothed_points = Vec::new();

        for i in 0..self.memory.exploration_points.len() {
            let start = i.saturating_sub(window_size / 2);
            let end = (i + window_size / 2 + 1).min(self.memory.exploration_points.len());

            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let count = (end - start) as f64;

            for j in start..end {
                sum_x += self.memory.exploration_points[j].0;
                sum_y += self.memory.exploration_points[j].1;
            }

            smoothed_points.push((sum_x / count, sum_y / count));
        }

        // Calculate curvature and optimal speed for each point
        self.memory.racing_line.clear();

        for i in 1..smoothed_points.len() - 1 {
            let prev = smoothed_points[i - 1];
            let curr = smoothed_points[i];
            let next = smoothed_points[i + 1];

            // Calculate curvature using three points
            let curvature = self.calculate_curvature(prev, curr, next);

            // Speed is inversely proportional to curvature
            let optimal_speed = self.calculate_optimal_speed(curvature);

            self.memory.racing_line.push(RacingLinePoint {
                pos: curr,
                optimal_speed,
                curvature,
            });
        }

        println!("Racing line built with {} points", self.memory.racing_line.len());
    }

    fn calculate_curvature(&self, p1: (f64, f64), p2: (f64, f64), p3: (f64, f64)) -> f64 {
        // Menger curvature formula
        let (x1, y1) = p1;
        let (x2, y2) = p2;
        let (x3, y3) = p3;

        let a = ((x2 - x1).powi(2) + (y2 - y1).powi(2)).sqrt();
        let b = ((x3 - x2).powi(2) + (y3 - y2).powi(2)).sqrt();
        let c = ((x3 - x1).powi(2) + (y3 - y1).powi(2)).sqrt();

        if a < 0.001 || b < 0.001 || c < 0.001 {
            return 0.0;
        }

        let area = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)).abs() / 2.0;
        let curvature = 4.0 * area / (a * b * c);

        curvature.min(2.0)  // Cap curvature
    }

    fn calculate_optimal_speed(&self, curvature: f64) -> f64 {
        // Higher curvature = lower speed
        // Based on physics: v = sqrt(a_lateral / curvature)
        let max_lateral_accel = 4.0;  // m/s^2

        if curvature < 0.01 {
            self.max_speed  // Straight section
        } else {
            let physics_limit = (max_lateral_accel / curvature).sqrt();
            physics_limit.clamp(self.min_speed, self.max_speed)
        }
    }

    // ==================== Pure Pursuit (Racing) ====================

    fn pure_pursuit_control(&mut self) -> (f64, f64) {
        if self.memory.racing_line.is_empty() {
            return (0.0, self.min_speed);
        }

        let (x, y, theta) = self.current_pose;

        // Adaptive lookahead based on speed
        let lookahead = self.min_lookahead +
            (self.current_speed / self.max_speed) * (self.max_lookahead - self.min_lookahead);

        // Find lookahead point on racing line
        let mut target_point = None;
        let mut best_idx = self.racing_line_idx;

        // Search ahead on racing line
        for i in 0..self.memory.racing_line.len() {
            let idx = (self.racing_line_idx + i) % self.memory.racing_line.len();
            let point = &self.memory.racing_line[idx];
            let dist = ((point.pos.0 - x).powi(2) + (point.pos.1 - y).powi(2)).sqrt();

            if dist >= lookahead {
                target_point = Some(point.clone());
                best_idx = idx;
                break;
            }
        }

        // Update racing line index for next iteration
        self.racing_line_idx = best_idx;

        let target = match target_point {
            Some(p) => p,
            None => return (0.0, self.min_speed),
        };

        // Transform target to vehicle frame
        let dx = target.pos.0 - x;
        let dy = target.pos.1 - y;

        // Rotate to vehicle frame
        let local_x = dx * theta.cos() + dy * theta.sin();
        let local_y = -dx * theta.sin() + dy * theta.cos();

        // Pure pursuit steering calculation
        let l_sq = local_x.powi(2) + local_y.powi(2);
        if l_sq < 0.001 {
            return (0.0, target.optimal_speed);
        }

        let curvature = 2.0 * local_y / l_sq;
        let steering_angle = (self.wheelbase * curvature).atan();

        (steering_angle.clamp(-0.4, 0.4), target.optimal_speed)
    }

    // ==================== Lap Management ====================

    fn update_lap(&mut self, time: f64) {
        // Simple lap detection based on returning to start area
        let (x, y, _) = self.current_pose;

        // Check if crossed start/finish (near origin)
        let dist_to_start = (x.powi(2) + y.powi(2)).sqrt();

        if dist_to_start < 2.0 && self.tick_count > 100 {
            let lap_time = time - self.lap_start_time;

            if lap_time > 5.0 {  // Minimum lap time to avoid false triggers
                self.lap_count += 1;
                self.memory.lap_times.push(lap_time);

                println!("\n=== LAP {} COMPLETE ===", self.lap_count);
                println!("Lap time: {:.2}s", lap_time);

                if self.memory.lap_times.len() > 1 {
                    let best = self.memory.lap_times.iter().cloned()
                        .min_by(|a, b| a.partial_cmp(b).unwrap())
                        .unwrap();
                    let improvement = self.memory.lap_times[self.memory.lap_times.len() - 2] - lap_time;
                    println!("Best: {:.2}s | Improvement: {:+.2}s", best, improvement);
                }

                // Transition phases based on lap count
                match self.lap_count {
                    1 => {
                        println!("\nExploration complete! Building racing line...");
                        self.phase = RacingPhase::Learning;
                        self.build_racing_line();
                        self.phase = RacingPhase::Racing;
                        println!("Phase 2: RACING - Executing optimized line");
                        self.max_speed = 7.0;  // Increase speed limit
                    }
                    3 => {
                        println!("\nIncreasing aggression...");
                        self.max_speed = 8.0;
                    }
                    5 => {
                        println!("\n=== RACE COMPLETE ===");
                        self.print_statistics();
                    }
                    _ => {}
                }

                self.lap_start_time = time;
            }
        }
    }

    fn print_statistics(&self) {
        println!("\n========== RACE STATISTICS ==========");
        println!("Total laps: {}", self.lap_count);
        println!("Total distance: {:.1}m", self.total_distance);
        println!("Collisions: {}", self.collisions);

        if !self.memory.lap_times.is_empty() {
            let best = self.memory.lap_times.iter().cloned()
                .min_by(|a, b| a.partial_cmp(b).unwrap()).unwrap();
            let avg = self.memory.lap_times.iter().sum::<f64>() / self.memory.lap_times.len() as f64;
            let first = self.memory.lap_times[0];
            let last = *self.memory.lap_times.last().unwrap();

            println!("\nLap Times:");
            for (i, time) in self.memory.lap_times.iter().enumerate() {
                println!("  Lap {}: {:.2}s", i + 1, time);
            }

            println!("\nBest lap: {:.2}s", best);
            println!("Average lap: {:.2}s", avg);
            println!("First lap: {:.2}s", first);
            println!("Final lap: {:.2}s", last);
            println!("Total improvement: {:.2}s ({:.1}%)",
                     first - last,
                     (1.0 - last / first) * 100.0);
        }
        println!("=====================================");
    }
}

impl Node for F1TenthRacer {
    fn name(&self) -> &'static str {
        "F1TenthRacer"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        self.tick_count += 1;
        let time = self.tick_count as f64 * 0.01;  // Assuming 100Hz

        // Update odometry
        if let Some(odom) = self.odom_sub.recv(&mut ctx) {
            let prev_pose = self.current_pose;
            self.current_pose = (odom.pose.x, odom.pose.y, odom.pose.theta);
            self.current_speed = (odom.twist.linear[0].powi(2) +
                                  odom.twist.linear[1].powi(2)).sqrt() as f64;

            // Track distance
            let dx = self.current_pose.0 - prev_pose.0;
            let dy = self.current_pose.1 - prev_pose.1;
            self.total_distance += (dx.powi(2) + dy.powi(2)).sqrt();

            // Record position history
            self.position_history.push_back((self.current_pose.0, self.current_pose.1));
            if self.position_history.len() > 100 {
                self.position_history.pop_front();
            }
        }

        // Get LIDAR data
        let ranges = self.lidar_sub.recv(&mut ctx)
            .map(|scan| scan.ranges.clone())
            .unwrap_or_default();

        // Check for collisions (very close readings)
        let min_range = ranges.iter()
            .filter(|&&r| r > 0.05)
            .cloned()
            .min_by(|a, b| a.partial_cmp(b).unwrap())
            .unwrap_or(10.0);

        if min_range < 0.15 {
            self.collisions += 1;
        }

        // Compute control based on current phase
        let (steering, speed) = match self.phase {
            RacingPhase::Exploration => {
                self.record_exploration_point();
                self.find_best_gap(&ranges)
            }
            RacingPhase::Learning => {
                // Shouldn't stay in this phase long
                (0.0, self.min_speed)
            }
            RacingPhase::Racing => {
                // Use pure pursuit with collision avoidance override
                let (pp_steering, pp_speed) = self.pure_pursuit_control();

                // Emergency collision avoidance
                if min_range < self.safety_distance {
                    let (gap_steering, _) = self.find_best_gap(&ranges);
                    (gap_steering, self.min_speed)
                } else {
                    (pp_steering, pp_speed)
                }
            }
        };

        // Send command
        let cmd = CmdVel::new(speed as f32, steering as f32);
        self.cmd_vel.send(cmd, &mut ctx).ok();

        // Update lap tracking
        self.update_lap(time);

        // Periodic status
        if self.tick_count % 500 == 0 {
            println!("[{:?}] Lap {} | Speed: {:.1} m/s | Dist: {:.0}m",
                     self.phase, self.lap_count, self.current_speed, self.total_distance);
        }
    }
}

fn main() -> HorusResult<()> {
    println!("========================================");
    println!("     F1Tenth Adaptive Racing System     ");
    println!("========================================");
    println!();
    println!("This controller learns and improves:");
    println!("  1. First lap: Exploration (gap-following)");
    println!("  2. After lap 1: Builds optimal racing line");
    println!("  3. Laps 2+: Pure pursuit on racing line");
    println!();
    println!("Make sure sim2d is running with an F1Tenth track!");
    println!("  Example: horus sim2d --robot f1tenth_robot.yaml");
    println!();

    let mut scheduler = Scheduler::new();

    scheduler.add(
        Box::new(F1TenthRacer::new()?),
        0,
        Some(true),
    );

    scheduler.run()
}
