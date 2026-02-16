"""
HORUS Python Benchmarks Suite

Comprehensive performance testing for HORUS Python bindings.

Benchmarks:
- ipc_latency: IPC latency with real robotics message types
- stress_test: Multi-threaded stress testing
- throughput: Large message throughput testing
- comparison: Comparison against alternatives (multiprocessing, ZeroMQ)

Run all benchmarks:
    python -m horus_py.benchmarks.run_all

Run individual benchmark:
    python -m horus_py.benchmarks.ipc_latency
"""

__version__ = "0.1.9"
