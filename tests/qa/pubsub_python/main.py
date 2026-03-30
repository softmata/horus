"""QA Test: Python 3-node pub/sub pipeline (runs indefinitely).

sensor_node:     publishes CmdVel at 50Hz
controller_node: subscribes CmdVel, republishes at 100Hz
motor_node:      subscribes CmdVel at 10Hz

Terminal 1: python3 tests/qa/pubsub_python/main.py
Terminal 2: horus topic list, horus node list, etc.
"""

import math
import horus
from horus import Node, CmdVel, us


tick_counter = {"sensor": 0, "controller": 0, "motor": 0}


def sensor_tick(node):
    tick_counter["sensor"] += 1
    t = tick_counter["sensor"]
    linear = 0.3 * math.sin(t * 0.02)
    angular = 0.1 * math.cos(t * 0.05)
    node.send("sensor.cmd_vel", {"linear": linear, "angular": angular})


def controller_tick(node):
    tick_counter["controller"] += 1
    msg = node.recv("sensor.cmd_vel")
    if msg is not None:
        scaled = {"linear": msg["linear"] * 0.8, "angular": msg["angular"] * 0.8}
        node.send("motor.cmd_vel", scaled)


def motor_tick(node):
    tick_counter["motor"] += 1
    node.recv("motor.cmd_vel")


def main():
    print("=== QA Python Pub/Sub: 3 nodes (Ctrl+C to stop) ===")

    sensor = Node(
        name="py_sensor",
        tick=sensor_tick,
        rate=50,
        pubs=["sensor.cmd_vel"],
    )

    controller = Node(
        name="py_controller",
        tick=controller_tick,
        rate=100,
        subs=["sensor.cmd_vel"],
        pubs=["motor.cmd_vel"],
    )

    motor = Node(
        name="py_motor",
        tick=motor_tick,
        rate=10,
        subs=["motor.cmd_vel"],
    )

    horus.run(sensor, controller, motor, tick_rate=100)


if __name__ == "__main__":
    main()
