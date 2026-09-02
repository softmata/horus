# HORUS Uninstallation Script v2.7.0 (Windows PowerShell)
# Complete removal of HORUS CLI, libraries, binaries, cache, and artifacts
# Native Windows PowerShell version — no bash/MSYS2 required
#
# Local:
#   .\uninstall.ps1             interactive
#   .\uninstall.ps1 -DryRun     list what would be removed, remove nothing
#   .\uninstall.ps1 -Yes        unattended
#
# One-line uninstall. `irm ... | iex` cannot pass parameters, so wrap the
# download in a scriptblock — piped to iex, -Yes never arrives and the run
# stops on the first Read-Host with nobody there to answer it:
#   & ([scriptblock]::Create((irm https://github.com/softmata/horus/raw/main/uninstall.ps1))) -Yes
#
# An install that set HORUS_PREFIX has to be uninstalled with the same value —
# install.sh puts bin\, cache\ and both state files under that prefix instead of
# under ~\.horus, and records the prefix nowhere outside it:
#   $env:HORUS_PREFIX = "C:\horus"; .\uninstall.ps1 -Yes
#
# -Yes is itself the answer to "are you sure"; every later prompt then takes the
# default printed in its [brackets], so an unattended run removes HORUS but
# KEEPS ~\.horus\config.toml and ~\.horus\credentials. Same contract as
# uninstall.sh --yes; the two have to agree or the same command means two
# different things depending on which shell the operator happened to use.

param(
    [switch]$DryRun,
    [switch]$Yes
)

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
# PROMPTS
# ============================================================================

# Ask a yes/no question. $default ("y" or "n") is both the answer shown
# capitalised in the hint and the answer -Yes takes. Write-Host, not Write-Output:
# anything on the pipeline would be returned alongside the boolean.
function Confirm-Removal($question, $default) {
    $hint = if ($default -eq "y") { "[Y/n]" } else { "[y/N]" }
    if ($Yes) {
        Write-Host "  ? $question $hint -Yes" -ForegroundColor Yellow
        return ($default -eq "y")
    }
    $reply = Read-Host "  ? $question $hint"
    if ([string]::IsNullOrWhiteSpace($reply)) { $reply = $default }
    return ($reply -eq "y" -or $reply -eq "Y")
}

# ============================================================================
# PATHS
# ============================================================================

$CargoHome = if ($env:CARGO_HOME) { $env:CARGO_HOME } else { "$env:USERPROFILE\.cargo" }
$InstallDir = "$CargoHome\bin"
$HorusDir = "$env:USERPROFILE\.horus"
$HorusAppData = "$env:APPDATA\horus"
$HorusLocalAppData = "$env:LOCALAPPDATA\horus"
$CacheDir = "$HorusDir\cache"
$ManifestFile = "$HorusDir\install_manifest.toml"
$VersionFile = "$HorusDir\installed_version"
$ProfileFile = "$HorusDir\install_profile"
$ShmDir = "$env:TEMP\horus_*"

$Binaries = @("horus.exe")

# The installer falls back to ~\.local\bin when Rust is absent — which is exactly
# the pre-built-binary user — so checking only ~\.cargo\bin leaves that binary
# behind. Same list uninstall.sh keeps, for the same reason.
$InstallDirs = @($InstallDir, "$env:USERPROFILE\.local\bin")

function Add-InstallDir($dir) {
    if ($dir -and ($script:InstallDirs -notcontains $dir)) {
        $script:InstallDirs = @($dir) + $script:InstallDirs
    }
}

# HORUS_PREFIX relocates the install root, so neither default above need hold
# the binary — and the state files move with it, so the manifest is read from
# there too. Only the READ location moves: $ManifestFile stays the ~\.horus copy
# that the step-4 cleanup removes, and the prefix copy is removed with the rest
# of the prefix further down.
$ManifestRead = $ManifestFile
if ($env:HORUS_PREFIX) {
    Add-InstallDir (Join-Path $env:HORUS_PREFIX "bin")
    $prefixManifest = Join-Path $env:HORUS_PREFIX "install_manifest.toml"
    if (Test-Path $prefixManifest) { $ManifestRead = $prefixManifest }
}

# The installer and `horus self update` record the binary they actually
# installed in install_manifest.toml. Prefer that over guessing. Only its
# *directory* is taken, and only horus.exe inside that directory is ever
# removed — the recorded path is never deleted verbatim, because this file is
# writable by anyone who can write the profile directory. Absence is normal and
# must stay non-fatal: installs predating the manifest have no such file.
if (Test-Path $ManifestRead) {
    $binaryLine = Select-String -Path $ManifestRead -Pattern '^\s*binary\s*=\s*"(.+)"' |
                  Select-Object -First 1
    if ($binaryLine) {
        # The manifest is TOML: a Windows path is stored as a basic string, so
        # every separator arrives here doubled ("C:\\Users\\..."). Handing that
        # to Split-Path yields a directory that is textually distinct from the
        # real one, so the binary gets listed twice and the copy that is removed
        # is the one that never existed. Undo TOML's escaping first.
        $recorded = $binaryLine.Matches[0].Groups[1].Value -replace '\\\\', '\'
        Add-InstallDir (Split-Path -Parent $recorded)
    }
}

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
foreach ($dir in $InstallDirs) {
    foreach ($bin in $Binaries) {
        $path = Join-Path $dir $bin
        if (Test-Path $path) {
            $size = (Get-Item $path).Length / 1MB
            Write-Host "    [x] $path ($([math]::Round($size, 1)) MB)"
            $BinaryCount++
        }
    }
}
if ($BinaryCount -eq 0) { Write-Warn "(no binaries found)" }

# HORUS directory
Write-Host ""
Write-Host "  HORUS Data:" -ForegroundColor Cyan
if (Test-Path $HorusDir) {
    $size = (Get-ChildItem $HorusDir -Recurse -ErrorAction SilentlyContinue | Measure-Object Length -Sum).Sum / 1MB
    Write-Host "    [x] $HorusDir ($([math]::Round($size, 1)) MB)"
    if (Test-Path $CacheDir)    { Write-Host "        - cache\ - library cache & pre-compiled deps" }
    if (Test-Path "$HorusDir\completions") { Write-Host "        - completions\ - generated shell completion scripts" }
    if (Test-Path $ProfileFile) { Write-Host "        - install_profile - installation type" }
    if (Test-Path $VersionFile) { Write-Host "        - installed_version - version gate state" }
    if (Test-Path $ManifestFile){ Write-Host "        - install_manifest.toml - install record (version, tag, commit)" }
} else {
    Write-Warn "(~\.horus not found)"
}

# AppData
if (Test-Path $HorusAppData) { Write-Host "    [x] $HorusAppData" }
if (Test-Path $HorusLocalAppData) { Write-Host "    [x] $HorusLocalAppData" }

# HORUS_PREFIX install root
if ($env:HORUS_PREFIX -and (Test-Path $env:HORUS_PREFIX) -and ($env:HORUS_PREFIX -ne $HorusDir)) {
    Write-Host "    [x] $env:HORUS_PREFIX - HORUS_PREFIX install root: bin\horus.exe, cache\, target\, completions\, env files and the state files. Anything else in it is left alone."
}

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

# -DryRun stops here. Everything above this line only reads, so this is exactly
# the non-destructive half of the script, and it is the path CI can run. The
# bash side had no such path, which is how a fatal bash-3.2 incompatibility
# (uninstall.sh:264) shipped in a script no automated test could execute.
if ($DryRun) {
    Write-Info "Dry run: nothing was removed."
    Write-Host "  Re-run without -DryRun (add -Yes to skip the prompts) to uninstall."
    Write-Host ""
    exit 0
}

# -Yes IS the answer to this question, which is why it is not routed through
# Confirm-Removal: that answers with the bracketed default, and the default
# here is "no".
if (-not $Yes) {
    $reply = Read-Host "  ? Are you sure you want to uninstall HORUS? [y/N]"
    if ($reply -ne "y" -and $reply -ne "Y") {
        Write-Host "`n  Uninstallation cancelled." -ForegroundColor Green
        exit 0
    }
}

$Removed = 0
$Skipped = 0

# ============================================================================
# 1. REMOVE BINARIES
# ============================================================================

Write-Host "`n  Removing binaries..." -ForegroundColor Cyan
foreach ($dir in $InstallDirs) {
    foreach ($bin in $Binaries) {
        $path = Join-Path $dir $bin
        if (Test-Path $path) {
            Remove-Item $path -Force
            Write-OK "Removed $bin from $dir"
            $Removed++
        }
    }
}

# ============================================================================
# 2. REMOVE SHELL COMPLETIONS (PowerShell profile)
# ============================================================================

Write-Host "`n  Removing completions..." -ForegroundColor Cyan
$profilePath = $PROFILE
if ($profilePath -and (Test-Path $profilePath)) {
    $content = Get-Content $profilePath -Raw
    if ($content -match "horus") {
        # Back the profile up first, the way uninstall.sh does for .bashrc and
        # .zshrc. These regexes rewrite a file this script did not fully write,
        # and a PowerShell profile is often the only copy of the user's setup.
        Copy-Item $profilePath "$profilePath.horus-backup" -Force -ErrorAction SilentlyContinue
        $content = $content -replace '.*horus.*completion.*\r?\n', ''
        $content = $content -replace '.*Register-ArgumentCompleter.*horus.*\r?\n', ''
        Set-Content $profilePath $content
        Write-OK "Cleaned horus from PowerShell profile (backup: $profilePath.horus-backup)"
        $Removed++
    }
}

# install.ps1 writes the generated completion to <state dir>\completions\horus.ps1
# and dot-sources it from the profile. The profile line is gone above; the script
# itself is what the bash side has always removed here and this one never did, so
# `. "...horus.ps1"` kept working out of a stale file after an uninstall. Both
# state roots, because HORUS_PREFIX moves the whole directory.
foreach ($compRoot in @($HorusDir, $env:HORUS_PREFIX)) {
    if ([string]::IsNullOrWhiteSpace($compRoot)) { continue }
    $compDir = Join-Path $compRoot "completions"
    if (Test-Path $compDir) {
        Remove-Item $compDir -Recurse -Force -ErrorAction SilentlyContinue
        Write-OK "Removed $compDir"
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
        if (Confirm-Removal "Remove configuration and credentials?" "n") {
            Remove-Item $HorusDir -Recurse -Force
            Write-OK "Removed entire ~\.horus directory"
            $Removed++
        } else {
            if (Test-Path $CacheDir) {
                Remove-Item $CacheDir -Recurse -Force
                Write-OK "Removed cache/"
            }
            # Both state files, always together. The installer and `horus self
            # update` write installed_version and install_manifest.toml on every
            # successful install; leaving either behind points the version gate
            # in version.rs at a HORUS that is no longer on the machine, and the
            # manifest is the richer of the two (it carries topic_version, the
            # field that actually decides whether the CLI can read its libs).
            foreach ($state in @($VersionFile, $ManifestFile, $ProfileFile)) {
                if (Test-Path $state) { Remove-Item $state -Force }
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

# HORUS_PREFIX install roots.
# When the installer was given HORUS_PREFIX it put everything there — bin\,
# cache\ and both state files — instead of under ~\.horus, so the block above
# cleans nothing for those installs. Enumerate what the installer writes rather
# than deleting the prefix wholesale: it is a directory the operator named, it
# can predate HORUS, and a mistyped value must not take the rest of it along.
# config.toml and credentials are left for the same reason they are in ~\.horus.
if ($env:HORUS_PREFIX -and (Test-Path $env:HORUS_PREFIX) -and ($env:HORUS_PREFIX -ne $HorusDir)) {
    foreach ($leaf in @("cache", "target", "installed_version",
                        "install_manifest.toml", "install_profile", "env.sh", "env.fish")) {
        $artifact = Join-Path $env:HORUS_PREFIX $leaf
        if (Test-Path $artifact) {
            Remove-Item $artifact -Recurse -Force -ErrorAction SilentlyContinue
            Write-OK "Removed $artifact"
            $Removed++
        }
    }
    # Only if empty — Remove-Item on a non-empty directory without -Recurse is
    # exactly the "leave anything the operator put here" behaviour we want.
    foreach ($dir in @((Join-Path $env:HORUS_PREFIX "bin"), $env:HORUS_PREFIX)) {
        if ((Test-Path $dir) -and -not (Get-ChildItem $dir -Force -ErrorAction SilentlyContinue)) {
            Remove-Item $dir -Force -ErrorAction SilentlyContinue
        }
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

# Python package.
# Get-Command first: with no pip on PATH, `& pip` never runs, so $LASTEXITCODE
# keeps whatever the previous external command left there — which is 0 often
# enough to offer to uninstall a package that is not installed.
if (Get-Command pip -ErrorAction SilentlyContinue) {
    & pip show horus-robotics 2>&1 | Out-Null
    if ($LASTEXITCODE -eq 0) {
        if (Confirm-Removal "Uninstall Python package horus-robotics?" "y") {
            & pip uninstall -y horus-robotics 2>&1 | Out-Null
            Write-OK "Uninstalled Python horus-robotics"
            $Removed++
        }
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
Write-Host "  - To reinstall: .\install.sh (Git Bash), or"
Write-Host "    curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash"
Write-Host ""

# Exit explicitly. Without this the script's status is whatever $LASTEXITCODE
# the last external command happened to leave behind, and the last one is the
# `pip show horus-robotics` PROBE above -- which returns 1 precisely when the
# package is not installed, i.e. on the ordinary path. A completely successful
# uninstall then reports failure to anything that checks, with no message to
# say why: `pwsh -Command`, a CI step, or a user's own wrapper script. This
# script reports its problems by printing them, so the status is 0.
exit 0
