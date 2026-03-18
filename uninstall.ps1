# HORUS Uninstallation Script v2.7.0 (Windows PowerShell)
# Complete removal of HORUS CLI, libraries, binaries, cache, and artifacts
# Native Windows PowerShell version — no bash/MSYS2 required

$ErrorActionPreference = "Continue"
$ScriptVersion = "2.7.0"

# ============================================================================
# COLORS
# ============================================================================

function Write-OK($msg) { Write-Host "  [+] $msg" -ForegroundColor Green }
function Write-Err($msg) { Write-Host "  [-] $msg" -ForegroundColor Red }
function Write-Warn($msg) { Write-Host "  [!] $msg" -ForegroundColor Yellow }
function Write-Info($msg) { Write-Host "  [*] $msg" -ForegroundColor Cyan }

# ============================================================================
# PATHS
# ============================================================================

$CargoHome = if ($env:CARGO_HOME) { $env:CARGO_HOME } else { "$env:USERPROFILE\.cargo" }
$InstallDir = "$CargoHome\bin"
$HorusDir = "$env:USERPROFILE\.horus"
$HorusAppData = "$env:APPDATA\horus"
$HorusLocalAppData = "$env:LOCALAPPDATA\horus"
$CacheDir = "$HorusDir\cache"
$ShmDir = "$env:TEMP\horus_*"

$Binaries = @("horus.exe")

Write-Host ""
Write-Host "============================================" -ForegroundColor Blue
Write-Host "   HORUS Uninstallation Script v$ScriptVersion" -ForegroundColor White
Write-Host "   Platform: Windows (PowerShell)" -ForegroundColor White
Write-Host "============================================" -ForegroundColor Blue
Write-Host ""

# ============================================================================
# SHOW WHAT WILL BE REMOVED
# ============================================================================

Write-Host "  Components to remove:" -ForegroundColor Magenta
Write-Host ""

# Binaries
Write-Host "  Binaries:" -ForegroundColor Cyan
$BinaryCount = 0
foreach ($bin in $Binaries) {
    $path = Join-Path $InstallDir $bin
    if (Test-Path $path) {
        $size = (Get-Item $path).Length / 1MB
        Write-Host "    [x] $path ($([math]::Round($size, 1)) MB)"
        $BinaryCount++
    }
}
if ($BinaryCount -eq 0) { Write-Warn "(no binaries found)" }

# HORUS directory
Write-Host ""
Write-Host "  HORUS Data:" -ForegroundColor Cyan
if (Test-Path $HorusDir) {
    $size = (Get-ChildItem $HorusDir -Recurse -ErrorAction SilentlyContinue | Measure-Object Length -Sum).Sum / 1MB
    Write-Host "    [x] $HorusDir ($([math]::Round($size, 1)) MB)"
} else {
    Write-Warn "(~\.horus not found)"
}

# AppData
if (Test-Path $HorusAppData) { Write-Host "    [x] $HorusAppData" }
if (Test-Path $HorusLocalAppData) { Write-Host "    [x] $HorusLocalAppData" }

# Shared memory
Write-Host ""
Write-Host "  Shared Memory:" -ForegroundColor Cyan
$ShmDirs = Get-ChildItem $env:TEMP -Directory -Filter "horus_*" -ErrorAction SilentlyContinue
if ($ShmDirs) {
    foreach ($d in $ShmDirs) { Write-Host "    [x] $($d.FullName)" }
} else {
    Write-Warn "(no shared memory data)"
}

# ============================================================================
# CONFIRMATION
# ============================================================================

Write-Host ""
Write-Host "--------------------------------------------" -ForegroundColor Blue
Write-Host ""
$reply = Read-Host "  ? Are you sure you want to uninstall HORUS? [y/N]"
if ($reply -ne "y" -and $reply -ne "Y") {
    Write-Host "`n  Uninstallation cancelled." -ForegroundColor Green
    exit 0
}

$Removed = 0
$Skipped = 0

# ============================================================================
# 1. REMOVE BINARIES
# ============================================================================

Write-Host "`n  Removing binaries..." -ForegroundColor Cyan
foreach ($bin in $Binaries) {
    $path = Join-Path $InstallDir $bin
    if (Test-Path $path) {
        Remove-Item $path -Force
        Write-OK "Removed $bin"
        $Removed++
    }
}

# ============================================================================
# 2. REMOVE SHELL COMPLETIONS (PowerShell profile)
# ============================================================================

Write-Host "`n  Removing completions..." -ForegroundColor Cyan
$profilePath = $PROFILE
if (Test-Path $profilePath) {
    $content = Get-Content $profilePath -Raw
    if ($content -match "horus") {
        $content = $content -replace '.*horus.*completion.*\r?\n', ''
        $content = $content -replace '.*Register-ArgumentCompleter.*horus.*\r?\n', ''
        Set-Content $profilePath $content
        Write-OK "Cleaned horus from PowerShell profile"
        $Removed++
    }
}

# ============================================================================
# 3. REMOVE SHARED MEMORY
# ============================================================================

Write-Host "`n  Cleaning shared memory..." -ForegroundColor Cyan
$ShmDirs = Get-ChildItem $env:TEMP -Directory -Filter "horus_*" -ErrorAction SilentlyContinue
foreach ($d in $ShmDirs) {
    Remove-Item $d.FullName -Recurse -Force -ErrorAction SilentlyContinue
    Write-OK "Removed $($d.Name)"
    $Removed++
}

# ============================================================================
# 4. REMOVE HORUS DIRECTORY
# ============================================================================

Write-Host "`n  Removing HORUS data..." -ForegroundColor Cyan
if (Test-Path $HorusDir) {
    $hasConfig = Test-Path "$HorusDir\config.toml"
    $hasCreds = (Test-Path "$HorusDir\credentials") -or (Test-Path "$HorusDir\auth.json")

    if ($hasConfig -or $hasCreds) {
        Write-Warn "Found user data in ~\.horus"
        $reply = Read-Host "  ? Remove configuration and credentials? [y/N]"
        if ($reply -eq "y" -or $reply -eq "Y") {
            Remove-Item $HorusDir -Recurse -Force
            Write-OK "Removed entire ~\.horus directory"
            $Removed++
        } else {
            if (Test-Path $CacheDir) {
                Remove-Item $CacheDir -Recurse -Force
                Write-OK "Removed cache/"
            }
            Write-Info "Kept user configuration files"
            $Removed++; $Skipped++
        }
    } else {
        Remove-Item $HorusDir -Recurse -Force
        Write-OK "Removed ~\.horus directory"
        $Removed++
    }
}

# ============================================================================
# 5. WINDOWS-SPECIFIC CLEANUP
# ============================================================================

Write-Host "`n  Platform cleanup..." -ForegroundColor Cyan

# AppData
if (Test-Path $HorusAppData) {
    Remove-Item $HorusAppData -Recurse -Force
    Write-OK "Removed AppData\Roaming\horus"
    $Removed++
}
if (Test-Path $HorusLocalAppData) {
    Remove-Item $HorusLocalAppData -Recurse -Force
    Write-OK "Removed AppData\Local\horus"
    $Removed++
}

# Python package
$pipCheck = & pip show horus-robotics 2>&1
if ($LASTEXITCODE -eq 0) {
    $reply = Read-Host "  ? Uninstall Python package horus-robotics? [Y/n]"
    if ($reply -ne "n" -and $reply -ne "N") {
        & pip uninstall -y horus-robotics 2>&1 | Out-Null
        Write-OK "Uninstalled Python horus-robotics"
        $Removed++
    }
}

# ============================================================================
# SUMMARY
# ============================================================================

Write-Host ""
Write-Host "============================================" -ForegroundColor Blue
Write-Host "   Uninstallation Complete" -ForegroundColor White
Write-Host "============================================" -ForegroundColor Blue
Write-Host ""
Write-Host "  Removed: $Removed component(s)" -ForegroundColor Green
Write-Host "  Skipped: $Skipped component(s)" -ForegroundColor Yellow
Write-Host ""

# Check for running processes
$procs = Get-Process -Name "horus*" -ErrorAction SilentlyContinue
if ($procs) {
    Write-Warn "Some HORUS processes may still be running."
    Write-Host "    Run: Stop-Process -Name horus -Force" -ForegroundColor Cyan
    Write-Host ""
}

Write-Host "  HORUS has been uninstalled. Goodbye!" -ForegroundColor Green
Write-Host ""
Write-Host "  Notes:" -ForegroundColor Cyan
Write-Host "  - Project-local .horus\ directories were NOT removed"
Write-Host "  - System packages were NOT removed"
Write-Host "  - To reinstall: .\install.sh (Git Bash) or cargo install horus"
Write-Host ""
