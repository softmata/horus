#!/usr/bin/env python3
"""Classify recent CI failures on the default branch.

Most red CI in this repo is environmental rather than a code defect, and an agent
that cannot tell the difference will "fix" code that was never broken. This script
sorts failures into KNOWN-ENVIRONMENTAL and UNEXPLAINED, and only the unexplained
ones — and only when they RECUR — are worth a human's or an agent's attention.

Read-only: it inspects workflow runs and prints a report. It changes nothing.
"""

from __future__ import annotations

import json
import subprocess
import sys
from collections import defaultdict

REPO = "softmata/horus"
BRANCH = "main"
RUNS_TO_SCAN = 25
# A single red run is usually noise on shared runners. Two of the same job is a
# pattern worth reporting.
RECURRENCE_THRESHOLD = 2

# Signatures of failures that are NOT code defects. Each entry is
# (label, explanation, [substrings that identify it]).
ENVIRONMENTAL = [
    (
        "tmp-quota",
        "/tmp per-user quota exhausted. `df` shows free space because it is a "
        "quota, not a full filesystem. Retry with TMPDIR off /tmp.",
        ["Disk quota exceeded", "os error 122"],
    ),
    (
        "disk-full",
        "Runner ran out of disk.",
        ["No space left on device", "os error 28"],
    ),
    (
        "shm-exhaustion",
        "/dev/shm filled by leaked test namespaces. Causes unrelated tests to "
        "fail in a rotating pattern. Clean with `rm -rf /dev/shm/horus_*`.",
        ["/dev/shm", "Cannot allocate memory"],
    ),
    (
        "coverage-instrumentation",
        "A tick-rate assertion under llvm-cov. Instrumentation costs roughly 18x "
        "on a 10kHz loop, which eats the margin. These are skipped in the "
        "coverage job on purpose; do not 'fix' the test.",
        ["tick count too low", "not auto-bumped"],
    ),
    (
        "tsan-lossy-contract",
        "ThreadSanitizer on the topic ring. Topic::send is send_lossy and "
        "overwrites the oldest unconsumed slot by design; the reader's stamp "
        "re-validation discards the torn result and TSan cannot see that.",
        ["ThreadSanitizer: reported", "__tsan_memcpy"],
    ),
    (
        "network-transient",
        "Network/registry transient, not a code change.",
        [
            "429 Too Many Requests",
            "Connection reset by peer",
            "Temporary failure in name resolution",
            "503 Service Unavailable",
        ],
    ),
]


def gh(*args: str) -> str:
    """Run gh with list args (never a shell string) and return stdout."""
    out = subprocess.run(
        ["gh", *args], capture_output=True, text=True, check=False, timeout=300
    )
    if out.returncode != 0:
        print(f"::warning::gh {' '.join(args)} failed: {out.stderr.strip()[:300]}")
        return ""
    return out.stdout


def failed_runs() -> list[dict]:
    raw = gh(
        "run", "list",
        "--repo", REPO,
        "--branch", BRANCH,
        "--status", "failure",
        "--limit", str(RUNS_TO_SCAN),
        "--json", "databaseId,name,conclusion,createdAt,headSha,url",
    )
    try:
        return json.loads(raw) if raw else []
    except json.JSONDecodeError:
        return []


def classify(run: dict) -> tuple[str | None, str]:
    """Return (environmental_label, evidence) or (None, '') if unexplained."""
    log = gh("run", "view", str(run["databaseId"]), "--repo", REPO, "--log-failed")
    if not log:
        return None, "could not read failed-job log"
    for label, explanation, needles in ENVIRONMENTAL:
        for n in needles:
            if n in log:
                return label, f"matched {n!r} — {explanation}"
    return None, extract_evidence(log)


# Post-job cleanup dominates the tail of every failed-job log and says nothing
# about the failure, so a naive tail reports git credential teardown.
NOISE = (
    "git config",
    "git-credentials",
    "git submodule foreach",
    "Cleaning up orphan processes",
    "Post job cleanup",
    "Removing credentials",
    "safe.directory",
    "Temporarily overriding HOME",
    "includeif.gitdir",
    "Removing SSH",
    "Removing HTTP",
)

ERROR_MARKERS = (
    "##[error]",
    "error[",
    "error:",
    "panicked at",
    "assertion",
    "FAILED",
    "failures:",
    "Error:",
    "not found",
    "missing",
)


def extract_evidence(log: str, want: int = 14) -> str:
    """Pull the lines that actually describe the failure."""
    lines = [ln for ln in log.splitlines() if ln.strip()]
    lines = [ln for ln in lines if not any(n in ln for n in NOISE)]
    hits = [ln for ln in lines if any(m in ln for m in ERROR_MARKERS)]
    chosen = hits[-want:] if hits else lines[-want:]
    # Strip the runner's "job\tstep\ttimestamp" prefix for readability.
    cleaned = []
    for ln in chosen:
        parts = ln.split("\t")
        cleaned.append(parts[-1].strip() if len(parts) > 1 else ln.strip())
    return "\n".join(cleaned) or "(no distinguishing lines found)"


def main() -> int:
    runs = failed_runs()
    if not runs:
        print("No failed runs on the default branch. Nothing to triage.")
        return 0

    env_hits: dict[str, int] = defaultdict(int)
    unexplained: dict[str, list[dict]] = defaultdict(list)

    for run in runs:
        label, evidence = classify(run)
        if label:
            env_hits[label] += 1
            continue
        unexplained[run["name"]].append({**run, "evidence": evidence})

    print(f"## Autopilot triage — {len(runs)} failed run(s) on {BRANCH}\n")

    if env_hits:
        print("### Explained by known environment issues (no action)\n")
        for label, n in sorted(env_hits.items(), key=lambda kv: -kv[1]):
            print(f"- `{label}` x{n}")
        print()

    recurring = {k: v for k, v in unexplained.items() if len(v) >= RECURRENCE_THRESHOLD}
    one_offs = {k: v for k, v in unexplained.items() if len(v) < RECURRENCE_THRESHOLD}

    if one_offs:
        print("### Unexplained, single occurrence (watching, not reporting)\n")
        for name, items in one_offs.items():
            print(f"- **{name}** — {items[0]['url']}")
        print()

    if not recurring:
        print("### No recurring unexplained failures. Nothing to escalate.\n")
        return 0

    print("### Recurring unexplained failures — these need attention\n")
    for name, items in recurring.items():
        print(f"#### `{name}` — failed {len(items)} times\n")
        for it in items[:3]:
            print(f"- {it['createdAt']} {it['headSha'][:8]} {it['url']}")
        print(f"\n<details><summary>log tail</summary>\n\n```\n{items[0]['evidence']}\n```\n\n</details>\n")

    # Signal to the workflow that there is something to report.
    return 10


if __name__ == "__main__":
    sys.exit(main())
