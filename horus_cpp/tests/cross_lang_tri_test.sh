#!/bin/bash
# Cross-Language Integration Test
# Tests: C++→Python, Python→C++, bidirectional field preservation
set -e

HORUS_ROOT="/home/neos/softmata/horus"
export LD_LIBRARY_PATH="$HORUS_ROOT/target/debug"
export PYTHONPATH="$HORUS_ROOT/horus_py"

PASS=0
FAIL=0

check() {
    if [ "$1" = "0" ]; then
        echo "[PASS] $2"
        PASS=$((PASS + 1))
    else
        echo "[FAIL] $2"
        FAIL=$((FAIL + 1))
    fi
}

echo "╔══════════════════════════════════════════════════╗"
echo "║  Cross-Language Integration Test                 ║"
echo "╚══════════════════════════════════════════════════╝"

# ── Test 1: C++ → Python (100 messages) ──────────────────────────────
echo ""
echo "=== Test 1: C++ Publisher → Python Subscriber ==="

python3 -c "
import sys, time
sys.path.insert(0, '$HORUS_ROOT/horus_py')
from horus._horus import Topic, CmdVel
t = Topic(CmdVel)
count = 0
deadline = time.time() + 5
while time.time() < deadline:
    msg = t.recv()
    if msg is not None:
        count += 1
    else:
        time.sleep(0.005)
    if count >= 100: break
print(f'PY_RECV:{count}')
" > /tmp/cl_py_sub.log 2>&1 &
PY=$!
sleep 0.3

"$HORUS_ROOT/target/cpp_tests/cross_process_ipc_test" pub "cmd_vel" > /tmp/cl_cpp_pub.log 2>&1
wait $PY 2>/dev/null

CPP_PUB=$(grep "PUB_DONE" /tmp/cl_cpp_pub.log | sed 's/PUB_DONE://')
PY_RECV=$(grep "PY_RECV" /tmp/cl_py_sub.log | sed 's/PY_RECV://')
echo "  C++ sent: $CPP_PUB, Python received: $PY_RECV"
[ "${PY_RECV:-0}" -ge 50 ]
check $? "C++→Python: >= 50/100 messages"

# ── Test 2: Python → C++ (20 messages) ──────────────────────────────
echo ""
echo "=== Test 2: Python Publisher → C++ Subscriber ==="

"$HORUS_ROOT/target/cpp_tests/cross_process_ipc_test" sub "cmd_vel" > /tmp/cl_cpp_sub.log 2>&1 &
CPP=$!
sleep 0.3

python3 -c "
import sys, time
sys.path.insert(0, '$HORUS_ROOT/horus_py')
from horus._horus import Topic, CmdVel
t = Topic(CmdVel)
for i in range(20):
    t.send(CmdVel(linear=float(i)*0.1, angular=float(i)*-0.05))
    time.sleep(0.05)
print('PY_SENT:20')
" > /tmp/cl_py_pub.log 2>&1
wait $CPP 2>/dev/null

CPP_RECV=$(grep "SUB_DONE" /tmp/cl_cpp_sub.log | sed 's/SUB_DONE:\([0-9]*\).*/\1/')
echo "  Python sent: 20, C++ received: $CPP_RECV"
[ "${CPP_RECV:-0}" -ge 10 ]
check $? "Python→C++: >= 10/20 messages"

# ── Test 3: Field preservation (linear value) ────────────────────────
echo ""
echo "=== Test 3: Field Preservation ==="

LAST_LINEAR=$(grep "SUB_DONE" /tmp/cl_cpp_sub.log | sed 's/.*last_linear=//')
echo "  Expected last_linear ~1.9, got: $LAST_LINEAR"
# Check it's between 1.0 and 2.0
python3 -c "v=float('${LAST_LINEAR:-0}'); exit(0 if 1.0 <= v <= 2.0 else 1)" 2>/dev/null
check $? "Field precision preserved across Python→C++"

# ── Test 4: C++↔C++ cross-process (baseline) ────────────────────────
echo ""
echo "=== Test 4: C++↔C++ Cross-Process Baseline ==="

"$HORUS_ROOT/target/cpp_tests/cross_process_ipc_test" sub "cpp_baseline" > /tmp/cl_base_sub.log 2>&1 &
BASE=$!
sleep 0.3
"$HORUS_ROOT/target/cpp_tests/cross_process_ipc_test" pub "cpp_baseline" > /tmp/cl_base_pub.log 2>&1
wait $BASE 2>/dev/null

BASE_RECV=$(grep "SUB_DONE" /tmp/cl_base_sub.log | sed 's/SUB_DONE:\([0-9]*\).*/\1/')
echo "  C++ sent: 100, C++ received: $BASE_RECV"
[ "${BASE_RECV:-0}" -ge 80 ]
check $? "C++↔C++: >= 80/100 messages"

# ── Summary ──────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════╗"
printf "║  Results: %d passed, %d failed                   ║\n" $PASS $FAIL
echo "╚══════════════════════════════════════════════════╝"
exit $FAIL
