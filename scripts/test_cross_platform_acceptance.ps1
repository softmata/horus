# ═══════════════════════════════════════════════════════════════════════════
# HORUS Cross-Platform CLI Acceptance Tests (Windows)
#
# PowerShell version of the acceptance test suite.
# Verifies the complete user workflow works on Windows.
#
# Usage: powershell -ExecutionPolicy Bypass -File scripts\test_cross_platform_acceptance.ps1
# ═══════════════════════════════════════════════════════════════════════════

$ErrorActionPreference = "Continue"
$Errors = 0
$Passed = 0
$Total = 0
$TmpDir = Join-Path $env:TEMP "horus_acceptance_test_$PID"

function Pass($msg) { $script:Passed++; $script:Total++; Write-Host "  ✓ PASS: $msg" -ForegroundColor Green }
function Fail($msg) { $script:Errors++; $script:Total++; Write-Host "  ✗ FAIL: $msg" -ForegroundColor Red }

Write-Host "═══════════════════════════════════════════════════════════════"
Write-Host "  HORUS Cross-Platform CLI Acceptance Tests (Windows)"
Write-Host "  Temp dir: $TmpDir"
Write-Host "═══════════════════════════════════════════════════════════════"

# ─── Build ────────────────────────────────────────────────────────────────

Write-Host ""
Write-Host "── Building horus CLI ──"
$buildResult = cargo build --no-default-features -p horus_manager 2>&1
if ($LASTEXITCODE -eq 0) { Pass "horus CLI builds" }
else { Fail "horus CLI build failed"; exit 1 }

# ─── Story 1: Project Creation ────────────────────────────────────────────

Write-Host ""
Write-Host "── Story 1: Project Creation ──"

$ProjectDir = Join-Path $TmpDir "test_project"
$newResult = cargo run --no-default-features -p horus_manager -- new $ProjectDir --lang rust 2>&1
if ($LASTEXITCODE -eq 0) { Pass "horus new creates project" }
else { Fail "horus new failed" }

if (Test-Path (Join-Path $ProjectDir "horus.toml")) { Pass "horus.toml exists" }
else { Fail "horus.toml missing" }

if (Test-Path (Join-Path $ProjectDir "src\main.rs")) { Pass "src\main.rs exists" }
else { Fail "src\main.rs missing" }

# ─── Story 2: Diagnostics ─────────────────────────────────────────────────

Write-Host ""
Write-Host "── Story 2: Diagnostics ──"

$doctorOutput = cargo run --no-default-features -p horus_manager -- doctor 2>&1 | Out-String
if ($doctorOutput -match "memory|shm|Memory") { Pass "horus doctor checks shared memory" }
else { Fail "horus doctor missing SHM check" }

$checkOutput = cargo run --no-default-features -p horus_manager -- check $ProjectDir 2>&1 | Out-String
if ($checkOutput.Length -gt 0) { Pass "horus check produces output" }
else { Fail "horus check produced no output" }

# ─── Story 3: CLI Help ────────────────────────────────────────────────────

Write-Host ""
Write-Host "── Story 3: CLI Help ──"

$helpOutput = cargo run --no-default-features -p horus_manager -- --help 2>&1 | Out-String
if ($helpOutput -match "new|build|run|test") { Pass "horus --help shows commands" }
else { Fail "horus --help missing expected commands" }

# ─── Cleanup ──────────────────────────────────────────────────────────────

if (Test-Path $TmpDir) { Remove-Item -Recurse -Force $TmpDir }

# ─── Summary ──────────────────────────────────────────────────────────────

Write-Host ""
Write-Host "═══════════════════════════════════════════════════════════════"
Write-Host "  Results: $Passed/$Total passed, $Errors failed"
Write-Host "═══════════════════════════════════════════════════════════════"

exit $Errors
