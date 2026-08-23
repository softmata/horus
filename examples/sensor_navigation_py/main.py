"""Sensor-based navigation in Python — the `sensor_navigation` example, ported.

The Python counterpart of examples/sensor_navigation. Same four nodes, same
rates, same topics; read the two side by side to see what the language
boundary changes.

Where python_robot is the Python "hello robot", this is the next step: a
multi-rate sensor pipeline rather than a single control loop.

    LidarProcessor (100Hz) --> Navigator (20Hz) --> cmd_vel
           |                       ^
       obstacles               imu.data
                                   |
    SensorFrames (1Hz)      TelemetryLogger (2Hz)

`lidar.scan` and `imu.data` come from the simulator. Without it the pipeline
still runs — the navigator drives forward and the telemetry logger says
"no imu", which is the honest reading of an unfed sensor.

Usage:
  horus run main.py
"""

import math

import horus

# ============================================================================
# Constants
# ============================================================================

MIN_DISTANCE = 0.5  # m — anything closer raises an obstacle alert
TURN_TICKS = 40  # ~2 s of turning at the navigator's 20 Hz
FORWARD = (0.3, 0.0)  # (linear m/s, angular rad/s)
EVADE = (0.05, 0.8)


# ============================================================================
# LidarProcessor (100 Hz) — raw scan in, obstacle alert out
# ============================================================================


def lidar_processor(node):
    scan = node.recv("lidar.scan")
    if scan is None:
        return

    # LaserScan.min_range() walks the 360 readings and skips the invalid ones
    # (a 0.0 entry means "no return", not "something 0 m away"), so this is the
    # Python spelling of the Rust example's filter/fold.
    closest = scan.min_range()
    if closest is None or closest >= MIN_DISTANCE:
        return

    index = min(
        range(len(scan.ranges)),
        key=lambda i: abs(scan.ranges[i] - closest),
    )
    node.send(
        "obstacles",
        {
            "angle_rad": scan.angle_min + index * scan.angle_increment,
            "distance_m": closest,
            "sector": 0,
        },
    )


# ============================================================================
# Navigator (20 Hz) — drive forward, turn away from obstacles
# ============================================================================


class NavigatorState:
    def __init__(self):
        self.turn_ticks = 0
        self.imu_ticks = 0


nav = NavigatorState()


def navigator(node):
    alert = node.recv("obstacles")
    if alert is not None:
        nav.turn_ticks = TURN_TICKS
        node.log_warning(
            f"Obstacle at {alert['distance_m']:.2f}m, "
            f"angle={math.degrees(alert['angle_rad']):.1f}deg - turning"
        )

    if nav.turn_ticks > 0:
        nav.turn_ticks -= 1
        linear, angular = EVADE
    else:
        linear, angular = FORWARD

    node.send("cmd_vel", horus.CmdVel(linear, angular))

    imu = node.recv("imu.data")
    if imu is not None:
        # The Rust example uses `hlog_every!(50, ...)`; Python has no such
        # macro, so the node counts its own ticks. Same effect, one line more.
        nav.imu_ticks += 1
        if nav.imu_ticks % 50 == 0:
            node.log_info(f"heading gyro_z={imu.gyro_z:.3f}")


# ============================================================================
# TelemetryLogger (2 Hz) — low-rate record for post-mortem
# ============================================================================


class TelemetryState:
    def __init__(self):
        self.ticks = 0


telemetry_state = TelemetryState()


def telemetry_logger(node):
    telemetry_state.ticks += 1

    cmd = node.recv("cmd_vel")
    cmd_info = f"lin={cmd.linear:.2f} ang={cmd.angular:.2f}" if cmd else "no cmd"

    imu = node.recv("imu.data")
    imu_info = f"acc=[{imu.accel_x:.1f},{imu.accel_y:.1f},{imu.accel_z:.1f}]" if imu else "no imu"

    node.log_info(f"[T{telemetry_state.ticks}] {cmd_info} | {imu_info}")


# ============================================================================
# SensorFrames (1 Hz) — the static mounts from the URDF
# ============================================================================


def build_frames():
    """Register the frame tree once, before the first tick.

    The Rust example builds this in `SensorFrames::new()`. Python has no
    constructor to hang it on, so it goes in the node's `init` callback —
    which is the same place in the lifecycle.
    """
    tf = horus.TransformFrame.small()
    tf.register_frame("map", None)
    tf.register_frame("odom", "map")
    tf.register_frame("base_link", "odom")
    # Offsets read from robots/sensor_bot.urdf.
    tf.register_static_frame(
        "lidar_link", horus.Transform(translation=(0.05, 0.0, 0.10)), "base_link"
    )
    tf.register_static_frame("imu_link", horus.Transform(translation=(0.0, 0.0, 0.06)), "base_link")
    return tf


frames = {}


def sensor_frames_init(node):
    frames["tf"] = build_frames()
    node.log_info(f"frame tree ready ({frames['tf'].frame_count()} frames)")


def sensor_frames(node):
    """Static frames never change; the node exists to keep them published."""


# ============================================================================
# Main — wire up nodes and run
# ============================================================================

if __name__ == "__main__":
    lidar = horus.Node(
        name="LidarProcessor",
        tick=lidar_processor,
        rate=100,
        order=0,
        subs={"lidar.scan": horus.LaserScan},
        # An untyped topic (a bare name, or `{}`) carries dicts. The alert has
        # no message type of its own, so a dict is the right shape here — but
        # `lidar.scan` and `cmd_vel` name their types, or the subscriber on the
        # other side receives a dict instead of a LaserScan/CmdVel.
        pubs={"obstacles": {}},
    )

    navigator_node = horus.Node(
        name="Navigator",
        tick=navigator,
        rate=20,
        order=1,
        subs={"obstacles": {}, "imu.data": horus.Imu},
        pubs={"cmd_vel": horus.CmdVel},
    )

    sensor_frames_node = horus.Node(
        name="SensorFrames",
        tick=sensor_frames,
        init=sensor_frames_init,
        rate=1,
        order=2,
    )

    telemetry = horus.Node(
        name="TelemetryLogger",
        tick=telemetry_logger,
        rate=2,
        order=10,
        subs={"cmd_vel": horus.CmdVel, "imu.data": horus.Imu},
    )

    print("Sensor Navigation (Python): LiDAR -> obstacle alert -> reactive drive")
    print("  Nodes: LidarProcessor (100Hz), Navigator (20Hz),")
    print("         SensorFrames (1Hz), TelemetryLogger (2Hz)")
    print("  Topics: lidar.scan, imu.data, obstacles, cmd_vel")
    print()

    horus.run(lidar, navigator_node, sensor_frames_node, telemetry, duration=60.0)
