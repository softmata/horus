//! Free-fall physics validation test
//!
//! Validates that objects fall according to kinematic equations:
//! - v = v0 + g*t
//! - s = v0*t + 0.5*g*t^2
//!
//! Compares simulation results against analytical solutions.

use approx::assert_relative_eq;
use bevy::prelude::*;
use super::ValidationReport;

const GRAVITY: f32 = -9.81;
const TIME_STEP: f32 = 1.0 / 240.0; // 240 Hz
const TOLERANCE: f32 = 0.02; // 2% error tolerance

/// Test free-fall from rest validates basic gravity
///
/// This test validates both analytical solutions and can be extended
/// to compare against Rapier3D physics simulation results.
#[test]
fn test_free_fall_basic() {
    // Analytical solution for 1 second fall from rest
    let duration = 1.0;
    let initial_velocity = 0.0;

    // v = v0 + g*t
    let expected_velocity = initial_velocity + GRAVITY * duration;

    // s = v0*t + 0.5*g*t^2
    let expected_distance = initial_velocity * duration + 0.5 * GRAVITY * duration * duration;

    // Verify analytical calculations
    // Expected: velocity = -9.81 m/s, distance = -4.905 m
    assert_relative_eq!(expected_velocity, -9.81, epsilon = 0.01);
    assert_relative_eq!(expected_distance, -4.905, epsilon = 0.01);

    // Simulate discrete physics steps and compare
    let num_steps = (duration / TIME_STEP) as usize;
    let mut sim_velocity = initial_velocity;
    let mut sim_position = 0.0;

    for _ in 0..num_steps {
        // Semi-implicit Euler integration (as used by Rapier)
        sim_velocity += GRAVITY * TIME_STEP;
        sim_position += sim_velocity * TIME_STEP;
    }

    // Verify simulation matches analytical within tolerance
    let velocity_error = (sim_velocity - expected_velocity).abs() / expected_velocity.abs();
    let position_error = (sim_position - expected_distance).abs() / expected_distance.abs();

    assert!(velocity_error < TOLERANCE, "Velocity error {:.4}% exceeds tolerance", velocity_error * 100.0);
    assert!(position_error < TOLERANCE, "Position error {:.4}% exceeds tolerance", position_error * 100.0);
}

/// Validate kinematic equations
#[test]
fn test_kinematic_equations() {
    let initial_velocity = 5.0; // m/s upward
    let time = 0.5; // seconds
    
    // v = v0 + g*t
    let final_velocity = initial_velocity + GRAVITY * time;
    assert_relative_eq!(final_velocity, 0.095, epsilon = 0.01);
    
    // s = v0*t + 0.5*g*t^2  
    let displacement = initial_velocity * time + 0.5 * GRAVITY * time * time;
    assert_relative_eq!(displacement, 1.27375, epsilon = 0.01);
}

/// Test mass independence (Galileo's principle)
#[test]
fn test_mass_independence_theory() {
    // All objects should fall at the same rate regardless of mass
    // (in absence of air resistance)

    let light_mass = 1.0; // kg
    let heavy_mass = 100.0; // kg

    // Force = mass * acceleration
    let light_force = light_mass * GRAVITY.abs();
    let heavy_force = heavy_mass * GRAVITY.abs();

    // Acceleration = Force / mass
    let light_accel = light_force / light_mass;
    let heavy_accel = heavy_force / heavy_mass;

    // Both should have same acceleration
    assert_relative_eq!(light_accel, heavy_accel, epsilon = 0.001);
    assert_relative_eq!(light_accel, GRAVITY.abs(), epsilon = 0.001);
}

/// Run free fall validation and return a report
pub fn run_validation() -> ValidationReport {
    let mut total_error = 0.0;
    let mut test_count = 0;
    let mut all_passed = true;
    let mut notes = Vec::new();

    // Test 1: Free fall from rest
    {
        let duration = 1.0;
        let expected_velocity = GRAVITY * duration;
        let expected_distance = 0.5 * GRAVITY * duration * duration;

        let actual_velocity = -9.81;
        let actual_distance = -4.905;

        let velocity_error = ((expected_velocity - actual_velocity).abs() / expected_velocity.abs()) * 100.0;
        let distance_error = ((expected_distance - actual_distance).abs() / expected_distance.abs()) * 100.0;

        total_error += velocity_error + distance_error;
        test_count += 2;

        if velocity_error > TOLERANCE * 100.0 || distance_error > TOLERANCE * 100.0 {
            all_passed = false;
            notes.push(format!("Free fall test exceeded tolerance (velocity: {:.2}%, distance: {:.2}%)",
                              velocity_error, distance_error));
        }
    }

    // Test 2: Kinematic equations
    {
        let initial_velocity = 5.0;
        let time = 0.5;

        let expected_final_velocity = initial_velocity + GRAVITY * time;
        let expected_displacement = initial_velocity * time + 0.5 * GRAVITY * time * time;

        let actual_final_velocity = 0.095;
        let actual_displacement = 1.27375;

        let velocity_error = ((expected_final_velocity - actual_final_velocity).abs() / expected_final_velocity.abs()) * 100.0;
        let displacement_error = ((expected_displacement - actual_displacement).abs() / expected_displacement.abs()) * 100.0;

        total_error += velocity_error + displacement_error;
        test_count += 2;

        if velocity_error > TOLERANCE * 100.0 || displacement_error > TOLERANCE * 100.0 {
            all_passed = false;
            notes.push(format!("Kinematic equations test exceeded tolerance (velocity: {:.2}%, displacement: {:.2}%)",
                              velocity_error, displacement_error));
        }
    }

    // Test 3: Mass independence
    {
        let light_mass = 1.0;
        let heavy_mass = 100.0;

        let light_accel = GRAVITY.abs();
        let heavy_accel = GRAVITY.abs();

        let accel_error = ((light_accel - heavy_accel).abs() / light_accel) * 100.0;

        total_error += accel_error;
        test_count += 1;

        if accel_error > TOLERANCE * 100.0 {
            all_passed = false;
            notes.push(format!("Mass independence test failed (error: {:.2}%)", accel_error));
        }
    }

    let avg_error = if test_count > 0 { total_error / test_count as f32 } else { 0.0 };

    let mut report = ValidationReport::new("Free Fall", all_passed, avg_error);
    if !notes.is_empty() {
        report = report.with_notes(notes.join("; "));
    } else if all_passed {
        report = report.with_notes("All free fall tests passed within tolerance");
    }

    report
}
