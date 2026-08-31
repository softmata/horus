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
import re
import subprocess
import sys
import time
from collections import defaultdict

REPO = "softmata/horus"
BRANCH = "main"
RUNS_TO_SCAN = 25
# A single red run is usually noise on shared runners. Two of the same job is a
# pattern worth reporting.
RECURRENCE_THRESHOLD = 2
# Fetching a full failed-job log per run is the slow part (~43s for 25 runs
# normally). The job allows 20 minutes; stop well short of that and report what we
# have rather than getting killed mid-run and producing nothing at all.
TIME_BUDGET_S = 600
_START = time.monotonic()


def out_of_time() -> bool:
    return (time.monotonic() - _START) > TIME_BUDGET_S

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
        "/dev/shm filled by leaked test namespaces. Clean with "
        "`rm -rf /dev/shm/horus_*`.",
        # Deliberately narrow. An earlier version matched the bare string
        # "/dev/shm", which every job echoes as part of `SHM_DIR=...` during
        # setup -- so it fired on almost every failure and buried real ones
        # (a genuine TSan failure, run 33393284819, was discarded that way).
        # Only a real allocation failure counts now.
        [
            "shm_open failed",
            "Failed to create shared memory",
            "No space left on device (os error 28)",
        ],
    ),
    (
        "coverage-instrumentation",
        "A tick-rate assertion under llvm-cov. Instrumentation costs roughly 18x "
        "on a 10kHz loop, which eats the margin. These are skipped in the "
        "coverage job on purpose; do not 'fix' the test.",
        ["tick count too low", "not auto-bumped"],
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

# Lines that are present on healthy runs and must never drive a classification.
# HORUS logs the mlockall warning on every runner because RT memory locking is
# not permitted there, and it contains "Cannot allocate memory" -- which an
# earlier version of this file matched as shm exhaustion.
BENIGN = (
    "mlockall failed",
    "memory lock failed",
)

# GitHub Actions renders echoed COMMANDS in bold cyan. Those lines are the
# workflow's own source text, not program output, so matching against them
# classifies on what a script says rather than on what happened.
COMMAND_ECHO = "\x1b[36;1m"


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
    """Failed runs that actually ran ON main, from this repo.

    `--branch main` filters on head_branch, which for a pull_request run is the
    PR's SOURCE branch -- and a fork's default branch is usually called `main`.
    So the naive filter lets a fork PR's logs into a report that is fed to an
    autonomous fixer. Restrict to push/schedule events from this repository.
    """
    raw = gh(
        "run", "list",
        "--repo", REPO,
        "--branch", BRANCH,
        "--status", "failure",
        "--limit", str(RUNS_TO_SCAN * 2),
        "--json", "databaseId,name,conclusion,createdAt,headSha,url,event",
    )
    try:
        runs = json.loads(raw) if raw else []
    except json.JSONDecodeError:
        return []
    # Excluding pull_request events IS the fork protection: a fork PR's run is a
    # pull_request event, so it can never reach the report regardless of what its
    # head branch is called.
    trusted = [
        r for r in runs
        if r.get("event") in ("push", "schedule", "workflow_dispatch")
    ]
    return trusted[:RUNS_TO_SCAN]


def split_jobs(log: str) -> dict[str, list[str]]:
    """`--log-failed` concatenates every failed job. Field 1 is the job name."""
    jobs: dict[str, list[str]] = defaultdict(list)
    for ln in log.splitlines():
        parts = ln.split("\t")
        jobs[parts[0].strip() if len(parts) > 1 else "(unknown job)"].append(ln)
    return jobs


def usable_lines(lines: list[str]) -> list[str]:
    out = []
    for ln in lines:
        if COMMAND_ECHO in ln:
            continue  # echoed workflow source, not output
        if any(b in ln for b in BENIGN):
            continue
        out.append(ln)
    return out


def classify_job(lines: list[str]) -> tuple[str | None, str]:
    body = "\n".join(usable_lines(lines))
    for label, explanation, needles in ENVIRONMENTAL:
        for n in needles:
            if n in body:
                return label, f"matched {n!r} — {explanation}"
    return None, extract_evidence(body)


def classify(run: dict) -> tuple[str | None, str, dict[str, int]]:
    """Classify each FAILED JOB separately.

    Classifying a whole run collapses a matrix: one genuinely environmental job
    would explain away a real code failure sitting beside it. A run counts as
    environmental only when EVERY failed job in it does.
    """
    log = gh("run", "view", str(run["databaseId"]), "--repo", REPO, "--log-failed")
    if not log:
        return None, "could not read failed-job log", {}

    jobs = split_jobs(log)
    labels: dict[str, int] = defaultdict(int)
    unexplained_evidence = ""
    all_explained = bool(jobs)

    for _name, lines in jobs.items():
        label, evidence = classify_job(lines)
        if label:
            labels[label] += 1
        else:
            all_explained = False
            if not unexplained_evidence:
                unexplained_evidence = evidence

    if all_explained:
        return "environmental", "", dict(labels)
    return None, unexplained_evidence, dict(labels)


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
    return sanitize("\n".join(cleaned) or "(no distinguishing lines found)")


def sanitize(text: str) -> str:
    """Neutralise log text before it is embedded in an issue body.

    That body is, by design, instructions to an autonomous fixer, so a log line
    is untrusted input on an instruction channel. Three things matter: a line
    containing a fence would close the code block and let the remainder render
    as markdown; ANSI escapes make the result unreadable; and unbounded length
    lets one run flood the issue.
    """
    # Two forms: a real escape byte, and the caret notation GitHub's log API
    # returns instead (literal '^' '[' — no ESC byte is present, which is why
    # matching only \x1b left 21 sequences of noise in the first report).
    text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text)
    text = re.sub(r"\^\[\[[0-9;]*[A-Za-z]", "", text)
    text = text.replace("`", "'")  # cannot close the fence
    # Defuse anything that reads as a directive to the agent downstream.
    text = re.sub(
        r"(?i)\b(ignore (all )?previous instructions|you are now|system:|assistant:)",
        "[redacted-directive]",
        text,
    )
    if len(text) > 4000:
        text = text[:4000] + "\n[truncated]"
    return text


def main() -> int:
    runs = failed_runs()
    if not runs:
        print("No failed runs on the default branch. Nothing to triage.")
        return 0

    env_hits: dict[str, int] = defaultdict(int)
    unexplained: dict[str, list[dict]] = defaultdict(list)

    skipped = 0
    for run in runs:
        if out_of_time():
            skipped += 1
            continue
        label, evidence, job_labels = classify(run)
        for k, v in job_labels.items():
            env_hits[k] += v
        if label == "environmental":
            continue
        unexplained[run["name"]].append({**run, "evidence": evidence})

    print(f"## Autopilot triage — {len(runs)} failed run(s) on {BRANCH}\n")

    # Never let a cap be silent: a truncated scan that looks complete is worse
    # than one that admits it ran out of time.
    if skipped:
        print(
            f"> Ran out of the {TIME_BUDGET_S}s log-fetch budget with {skipped} "
            f"run(s) unexamined. This report is incomplete.\n"
        )

    if env_hits:
        print(
            "### Environment signatures seen (a run is only dismissed when EVERY "
            "failed job in it is explained)\n"
        )
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
