#!/usr/bin/env python3
"""Python publisher -> C++ subscriber cross-process IPC test.

Publishes 20 CmdVel messages via horus_py, then exits.
The C++ subscriber (cross_process_ipc_test sub) should receive them.
"""
import sys
import time
sys.path.insert(0, '/home/neos/softmata/horus/horus_py')

import horus

topic_name = sys.argv[1] if len(sys.argv) > 1 else "py2cpp.test"

pub = horus.Topic(topic_name, horus.CmdVel)
for i in range(20):
    msg = horus.CmdVel()
    msg.timestamp_ns = i
    msg.linear = float(i) * 0.05
    msg.angular = float(i) * -0.02
    pub.send(msg)
    time.sleep(0.02)

print(f"PY_PUB_DONE:20")
