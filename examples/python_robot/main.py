"""Differential-drive robot controller in Python.

This is the Python equivalent of the Rust `differential_drive` example.
It demonstrates the core horus Python API: Node, Topic, CmdVel, run().

Nodes:
  - SquareDriver (30Hz): Drives the robot in a square pattern
  - SafetyMonitor (10Hz): Monitors velocity limits

Topics:
  - cmd_vel (CmdVel): Velocity commands
  - odom (dict): Simulated odometry feedback

Usage:
  horus run main.py
"""

import horus
import math

# ============================================================================
# Constants
# ============================================================================

MAX_LINEAR_VEL = 0.5   # m/s
MAX_ANGULAR_VEL = 1.0  # rad/s
SIDE_LENGTH = 1.0       # meters per side
TURN_ANGLE = math.pi / 2  # 90 degrees


# ============================================================================
# Square Driver Node — drives the robot in a square pattern
# ============================================================================

class SquareDriverState:
    """State machine for square driving pattern."""
    def __init__(self):
        self.phase = "forward"  # "forward" or "turning"
        self.phase_ticks = 0
        self.sides_completed = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

state = SquareDriverState()

def square_driver(node):
    """Drive in a square: forward 1m, turn 90 degrees, repeat."""
    s = state
    s.phase_ticks += 1

    if s.phase == "forward":
        # Drive forward at 0.3 m/s
        linear = 0.3
        angular = 0.0

        # Switch to turning after enough ticks (30Hz * ~3.3s ≈ 100 ticks for 1m)
        forward_ticks = int(SIDE_LENGTH / linear * 30)
        if s.phase_ticks >= forward_ticks:
            s.phase = "turning"
            s.phase_ticks = 0
            node.log_info(f"Side {s.sides_completed + 1} complete, turning...")

    elif s.phase == "turning":
        # Turn at 0.5 rad/s
        linear = 0.0
        angular = 0.5

        # Switch to forward after 90-degree turn
        turn_ticks = int(TURN_ANGLE / angular * 30)
        if s.phase_ticks >= turn_ticks:
            s.phase = "forward"
            s.phase_ticks = 0
            s.sides_completed += 1
            node.log_info(f"Turn complete. Sides completed: {s.sides_completed}")

            if s.sides_completed >= 4:
                s.sides_completed = 0
                node.log_info("Square complete! Starting new square.")

    else:
        linear = 0.0
        angular = 0.0

    # Publish velocity command
    node.send("cmd_vel", horus.CmdVel(linear, angular))

    # Simulate odometry (simple dead reckoning)
    dt = 1.0 / 30.0
    s.theta += angular * dt
    s.x += linear * math.cos(s.theta) * dt
    s.y += linear * math.sin(s.theta) * dt

    # Publish simulated odometry
    node.send("odom", {
        "x": round(s.x, 3),
        "y": round(s.y, 3),
        "theta": round(s.theta, 3),
        "linear_vel": linear,
        "angular_vel": angular,
    })


# ============================================================================
# Safety Monitor Node — watches for velocity limit violations
# ============================================================================

def safety_monitor(node):
    """Monitor velocity commands and warn on limit violations."""
    if node.has_msg("cmd_vel"):
        cmd = node.recv("cmd_vel")
        if cmd is not None:
            if abs(cmd.linear) > MAX_LINEAR_VEL:
                node.log_warning(
                    f"Linear velocity {cmd.linear:.2f} exceeds limit {MAX_LINEAR_VEL}"
                )
            if abs(cmd.angular) > MAX_ANGULAR_VEL:
                node.log_warning(
                    f"Angular velocity {cmd.angular:.2f} exceeds limit {MAX_ANGULAR_VEL}"
                )


# ============================================================================
# Main — wire up nodes and run
# ============================================================================

if __name__ == "__main__":
    driver = horus.Node(
        name="square_driver",
        tick=square_driver,
        rate=30,
        order=0,
        pubs=["cmd_vel", "odom"],
    )

    safety = horus.Node(
        name="safety_monitor",
        tick=safety_monitor,
        rate=10,
        order=10,
        subs=["cmd_vel"],
    )

    print("Python Robot: Driving in a square pattern")
    print("  Nodes: square_driver (30Hz), safety_monitor (10Hz)")
    print("  Topics: cmd_vel (CmdVel), odom (dict)")
    print()

    horus.run(driver, safety, duration=30.0)
