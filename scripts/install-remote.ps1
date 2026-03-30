# HORUS One-Line Installer (Windows PowerShell)
#
# Usage:
#   irm https://horus.dev/install.ps1 | iex
#   irm https://gitlab.com/softmata/horus/-/raw/main/scripts/install-remote.ps1 | iex

$ErrorActionPreference = "Stop"
$Repo = "softmata/horus"

Write-Host ""
Write-Host "  HORUS — Deterministic Real-Time Robotics Framework" -ForegroundColor Cyan
Write-Host ""

# Detect architecture
$Arch = if ([Environment]::Is64BitOperatingSystem) { "x86_64" } else { "x86" }
Write-Host "  OS:      Windows" -ForegroundColor Green
Write-Host "  Arch:    $Arch" -ForegroundColor Green

# Install directory
$CargoHome = if ($env:CARGO_HOME) { $env:CARGO_HOME } else { "$env:USERPROFILE\.cargo" }
$InstallDir = "$CargoHome\bin"
if (!(Test-Path $InstallDir)) { New-Item -ItemType Directory -Path $InstallDir -Force | Out-Null }

Write-Host "  Install: $InstallDir" -ForegroundColor Green
Write-Host ""

# Try downloading pre-built binary
$AssetName = "horus-windows-${Arch}.exe"
$ReleaseUrl = "https://github.com/$Repo/releases/latest/download/$AssetName"
$TargetPath = "$InstallDir\horus.exe"

Write-Host "[1/3] Downloading horus..." -ForegroundColor Cyan

try {
    Invoke-WebRequest -Uri $ReleaseUrl -OutFile $TargetPath -UseBasicParsing
    Write-Host "[1/3] Downloaded pre-built binary" -ForegroundColor Green
} catch {
    Write-Host "[1/3] No pre-built binary, building from source..." -ForegroundColor Yellow

    # Check for Rust
    if (!(Get-Command cargo -ErrorAction SilentlyContinue)) {
        Write-Host "[1/3] Installing Rust..." -ForegroundColor Cyan
        $RustupUrl = "https://win.rustup.rs/x86_64"
        $RustupExe = "$env:TEMP\rustup-init.exe"
        Invoke-WebRequest -Uri $RustupUrl -OutFile $RustupExe -UseBasicParsing
        & $RustupExe -y --default-toolchain stable
        $env:PATH = "$CargoHome\bin;$env:PATH"
    }

    Write-Host "[1/3] Building from source (this takes a few minutes)..." -ForegroundColor Cyan
    cargo install --git "https://github.com/$Repo" horus_manager --no-default-features --locked
}

# Verify
Write-Host "[2/3] Verifying..." -ForegroundColor Cyan
if (Test-Path $TargetPath) {
    $Version = & $TargetPath --version 2>&1
    Write-Host "[2/3] Installed: $Version" -ForegroundColor Green
} elseif (Get-Command horus -ErrorAction SilentlyContinue) {
    $Version = horus --version
    Write-Host "[2/3] Installed: $Version" -ForegroundColor Green
} else {
    Write-Host "[2/3] Installation failed" -ForegroundColor Red
    exit 1
}

# PATH check
Write-Host "[3/3] Checking PATH..." -ForegroundColor Cyan
if ($env:PATH -notlike "*$InstallDir*") {
    [Environment]::SetEnvironmentVariable("PATH", "$InstallDir;$([Environment]::GetEnvironmentVariable('PATH', 'User'))", "User")
    Write-Host "[3/3] Added to PATH (restart terminal to take effect)" -ForegroundColor Green
}

Write-Host ""
Write-Host "  Installation complete!" -ForegroundColor Green
Write-Host ""
Write-Host "  Get started:" -ForegroundColor White
Write-Host "    horus new my_robot -r     Create a new Rust project" -ForegroundColor Cyan
Write-Host "    horus new my_robot -p     Create a new Python project" -ForegroundColor Cyan
Write-Host "    horus doctor              Check your environment" -ForegroundColor Cyan
Write-Host ""
Write-Host "  Documentation: https://docs.horusrobotics.dev" -ForegroundColor Cyan
Write-Host ""
