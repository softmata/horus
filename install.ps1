# HORUS Installer (Windows, PowerShell)
#
# One-line install:
#   irm https://github.com/softmata/horus/raw/main/install.ps1 | iex
#
# A piped script cannot be given parameters, so the knobs are environment
# variables — the same surface install.sh takes. PowerShell has no
# `VAR=value command` prefix form, so they are set as their own statement:
#   $env:HORUS_VERSION = 'v0.4.0'
#   irm https://github.com/softmata/horus/raw/main/install.ps1 | iex
#
# Environment:
#   HORUS_VERSION=v0.4.0          Install exactly this tag, binary AND source.
#                                 Accepted with or without the leading "v".
#   HORUS_BUILD_FROM_SOURCE=1     Skip the pre-built binary; compile the source
#                                 at the resolved tag.
#   HORUS_INSTALL_BRANCH=<branch> Developer escape hatch: build from source at
#                                 that branch. Implies HORUS_BUILD_FROM_SOURCE=1
#                                 — a branch tree and a tagged binary are the
#                                 skew described below.
#   HORUS_LOCAL_SOURCE=C:\src\horus
#                                 Offline/air-gapped: build an existing local
#                                 tree. No clone, no source download.
#   HORUS_PREFIX=C:\opt\horus     Install root override: <prefix>\bin for the
#                                 binary, <prefix> in place of ~\.horus for the
#                                 state files and the source cache.
#   HORUS_NO_SHELL_INTEGRATION=1  Do not touch the PowerShell profile.
#
# The same knobs are also parameters — -Version, -BuildFromSource,
# -InstallBranch, -LocalSource, -Prefix, -NoShellIntegration — for a saved copy
# (.\install.ps1 -Version v0.4.0) or, one-line, wrapped in a scriptblock the way
# uninstall.ps1 documents:
#   & ([scriptblock]::Create((irm https://github.com/softmata/horus/raw/main/install.ps1))) -Version v0.4.0
#
# Flow:
#   1. Resolve ONE ref: a release tag, or the tag/branch/tree the caller pinned
#   2. Clone the source at that ref into <state>\cache\horus@<version>
#   3. Download horus-windows-amd64.zip for the SAME tag and verify it against
#      that tag's SHA256SUMS (fail closed)
#   4. Install horus.exe, record what was installed, put it on the user PATH
#
# Steps 1-3 are one decision on purpose. The binary used to come from
# releases/latest/download/... (a tag) while the source was cloned from main
# HEAD — 93 commits apart, both trees calling themselves 0.4.0, with
# TOPIC_VERSION 3 at the tag and 4 on main. The installed CLI then could not
# read the shared memory its own libraries wrote: `horus topic list` reported 0
# messages on a live topic and `horus launch` failed with "Incompatible topic
# version: 4 (expected 3)". Nothing below may mix two refs.
#
# Why this file exists: Windows had no supported install path at all. The
# documented one was install.sh under Git Bash, which dies at
# "unzip: command not found" (exit 127) one line after printing "Checksum
# verified" — unzip is not in Git for Windows' bundled MSYS2 set — while
# release.yml has been building and publishing horus-windows-amd64.zip the whole
# time. uninstall.ps1 has always been native PowerShell; this is its peer.

param(
    # Accepted as "0.4.0" or "v0.4.0"; the tags themselves are v-prefixed.
    [string]$Version = $env:HORUS_VERSION,

    [string]$Prefix = $env:HORUS_PREFIX,

    # Compared against '1' rather than cast, because every non-empty string
    # casts to $true in PowerShell — including '0', which is how someone turns
    # a flag *off*.
    [switch]$BuildFromSource = ($env:HORUS_BUILD_FROM_SOURCE -eq '1'),

    [string]$LocalSource = $env:HORUS_LOCAL_SOURCE,

    [string]$InstallBranch = $env:HORUS_INSTALL_BRANCH,

    # install.sh treats any non-empty value as "set" (`[ -n "$..." ]`); match it.
    [switch]$NoShellIntegration = (-not [string]::IsNullOrEmpty($env:HORUS_NO_SHELL_INTEGRATION))
)

# ============================================================================
# CONFIG
# ============================================================================

$Repo = 'softmata/horus'
$BinaryName = 'horus.exe'
# release.yml's matrix builds exactly one Windows target,
# x86_64-pc-windows-msvc, published as horus-windows-amd64.zip (release.yml:58).
$AssetName = 'horus-windows-amd64'
$AssetFile = "$AssetName.zip"
$UserAgent = 'horus-installer'

# ============================================================================
# COLORS — same four shapes uninstall.ps1 prints, so a session that runs both
# reads as one tool.
# ============================================================================

function Write-OK($msg) { Write-Host "  [+] $msg" -ForegroundColor Green }
function Write-Err($msg) { Write-Host "  [-] $msg" -ForegroundColor Red }
function Write-Warn($msg) { Write-Host "  [!] $msg" -ForegroundColor Yellow }
function Write-Info($msg) { Write-Host "  [*] $msg" -ForegroundColor Cyan }

# ============================================================================
# HELPERS
# ============================================================================

function Test-IsWindows {
    # $IsWindows exists only in PowerShell 6+. In Windows PowerShell 5.1 — the
    # shell every Windows box actually ships with — it is undefined, so reading
    # it directly would report "not Windows" on Windows.
    $v = Get-Variable -Name 'IsWindows' -ValueOnly -ErrorAction SilentlyContinue
    if ($null -eq $v) { return ($env:OS -eq 'Windows_NT') }
    return [bool]$v
}

# Native commands do not participate in $ErrorActionPreference: a failing git or
# cargo sets $LASTEXITCODE and execution carries on. Worse, in Windows
# PowerShell 5.1 a *redirected* stderr stream is converted into ErrorRecords, so
# with the preference on Stop the first line of ordinary git progress chatter
# becomes a terminating NativeCommandError. Both helpers below therefore relax
# the preference for the duration of the call and judge the tool by its exit
# code, which is the only thing it actually promised.
function Invoke-Native {
    param([Parameter(Mandatory = $true)][string]$FilePath, [string[]]$Arguments = @())
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    # Primed to a failure, because a command that never starts — a binary for the
    # wrong architecture, a missing DLL — leaves $LASTEXITCODE at whatever the
    # last command that *did* run set it to. Reading a stale 0 there is how an
    # installer reports success for something it never executed.
    $global:LASTEXITCODE = -1
    try {
        & $FilePath @Arguments
        return $LASTEXITCODE
    } finally { $ErrorActionPreference = $prev }
}

function Get-NativeOutput {
    param([Parameter(Mandatory = $true)][string]$FilePath, [string[]]$Arguments = @(), [string]$StdErrFile)
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $global:LASTEXITCODE = -1
    try {
        if ($StdErrFile) { $out = & $FilePath @Arguments 2>$StdErrFile }
        else { $out = & $FilePath @Arguments 2>$null }
        # -join, not Out-String: Out-String re-formats through the host and wraps
        # at the console width, which silently breaks the long lines of a
        # generated completion script.
        return [pscustomobject]@{ ExitCode = $LASTEXITCODE; Output = (($out -join "`n").Trim()) }
    } finally { $ErrorActionPreference = $prev }
}

function Invoke-HorusFetch {
    param([Parameter(Mandatory = $true)][string]$Uri, [Parameter(Mandatory = $true)][string]$OutFile)
    # -UseBasicParsing because Windows PowerShell 5.1 otherwise hands the body to
    # the Internet Explorer DOM parser, which throws on a machine where IE first
    # run has never been completed — i.e. a fresh Windows Server. A User-Agent
    # because github.com answers 403 to some requests without one.
    Invoke-WebRequest -Uri $Uri -OutFile $OutFile -UseBasicParsing -UserAgent $UserAgent -TimeoutSec 300
}

# The ref becomes a URL path segment and a `git clone --branch` argument. One
# beginning with "-" is read as an option by git, and a "/" or ".." would walk
# out of the release URL. Release tags are v*.*.* (release.yml:6). Mirrors
# install.sh's valid_ref().
function Test-ValidRef {
    param([string]$Ref)
    if ([string]::IsNullOrEmpty($Ref)) { return $false }
    if ($Ref.StartsWith('-')) { return $false }
    if ($Ref.Contains('..')) { return $false }
    # \z, not $: in .NET "$" also matches immediately before a trailing
    # newline, so a ref ending in one would pass a check meant to keep it out of
    # a URL and a git argument.
    return ($Ref -match '^[A-Za-z0-9._-]+\z')
}

# Resolution goes through the plain redirect rather than
# api.github.com/repos/.../releases/latest: the API allows 60 unauthenticated
# requests per hour *per IP*, which one lab behind one NAT (or a CI matrix)
# exhausts in minutes. The redirect from /releases/latest to /releases/tag/<T>
# is not rate limited. The API is kept only as a second opinion.
function Resolve-LatestTag {
    param([Parameter(Mandatory = $true)][string]$Repository)

    try {
        $resp = Invoke-WebRequest -Uri "https://github.com/$Repository/releases/latest" `
            -UseBasicParsing -UserAgent $UserAgent -TimeoutSec 60
        $final = $null
        $base = $resp.BaseResponse
        if ($base) {
            # Windows PowerShell 5.1 hands back a System.Net.HttpWebResponse and
            # PowerShell 7 a System.Net.Http.HttpResponseMessage; they spell the
            # URL the redirects ended at differently.
            if ($base.PSObject.Properties['ResponseUri'] -and $base.ResponseUri) {
                $final = [string]$base.ResponseUri
            } elseif ($base.PSObject.Properties['RequestMessage'] -and $base.RequestMessage) {
                $final = [string]$base.RequestMessage.RequestUri
            }
        }
        # A repository with no published release redirects to /releases instead,
        # so there is no tag to strip and nothing to install.
        if ($final -and $final -match '/releases/tag/([^/?#]+)') { return $Matches[1] }
    } catch {
        Write-Warn "github.com/$Repository/releases/latest: $($_.Exception.Message)"
    }

    try {
        $rel = Invoke-RestMethod -Uri "https://api.github.com/repos/$Repository/releases/latest" `
            -UseBasicParsing -UserAgent $UserAgent -TimeoutSec 60
        if ($rel -and $rel.tag_name) { return [string]$rel.tag_name }
    } catch {
        Write-Warn "api.github.com: $($_.Exception.Message)"
    }

    return $null
}

# `sha256sum *.tar.gz *.zip > SHA256SUMS` (release.yml:185) writes
# "<64 hex>  <name>", with a "*" in front of the name for anything hashed in
# binary mode. Match the name exactly: a substring test would let
# horus-windows-amd64.zip.bak satisfy the check for horus-windows-amd64.zip.
function Get-Sha256SumsEntry {
    param([Parameter(Mandatory = $true)][string]$SumsPath, [Parameter(Mandatory = $true)][string]$FileName)
    foreach ($line in [System.IO.File]::ReadAllLines($SumsPath)) {
        if ($line -match '^\s*([0-9a-fA-F]{64})\s+\*?(.+?)\s*$') {
            if ([string]::Equals($Matches[2], $FileName, [System.StringComparison]::Ordinal)) {
                return $Matches[1].ToLowerInvariant()
            }
        }
    }
    return $null
}

function Expand-HorusZip {
    param([Parameter(Mandatory = $true)][string]$ZipPath, [Parameter(Mandatory = $true)][string]$Destination)
    New-Item -ItemType Directory -Path $Destination -Force | Out-Null
    # A zip that carries a mark-of-the-web makes Expand-Archive fail with
    # "cannot be loaded because running scripts is disabled"-shaped noise on
    # locked-down machines. We fetched it ourselves and verified its digest one
    # step ago, so clearing the mark costs nothing.
    # try/catch and not just -ErrorAction: this cmdlet throws outright on a host
    # that has no zone identifier to clear, and -ErrorAction covers only the
    # non-terminating kind. Clearing the mark is a convenience either way.
    try { Unblock-File -LiteralPath $ZipPath -ErrorAction SilentlyContinue } catch { }
    if (Get-Command Expand-Archive -ErrorAction SilentlyContinue) {
        Expand-Archive -LiteralPath $ZipPath -DestinationPath $Destination -Force
    } else {
        # Expand-Archive arrived in PowerShell 5.0; Windows 7 and 8.1 still ship
        # 4.0, and this is the only zip reader guaranteed to be there.
        Add-Type -AssemblyName System.IO.Compression.FileSystem
        [System.IO.Compression.ZipFile]::ExtractToDirectory($ZipPath, $Destination)
    }
}

function Remove-CachedSourceDir {
    param([Parameter(Mandatory = $true)][string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) { return }
    $item = Get-Item -LiteralPath $Path -Force
    if ($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) {
        # `Remove-Item -Recurse` walks *into* a junction on Windows PowerShell
        # 5.1 and deletes the tree it points at. Here that tree is the user's own
        # checkout, linked in by a previous HORUS_LOCAL_SOURCE install — so the
        # installer would silently delete the source it was handed. Delete the
        # link itself and nothing behind it.
        [System.IO.Directory]::Delete($Path, $false)
        return
    }
    Remove-Item -LiteralPath $Path -Recurse -Force
}

# Windows paths are full of backslashes and a TOML basic string reads "\U" as an
# escape, so "C:\Users\..." is not merely ugly, it fails to parse. Escape rather
# than switching to a literal string, which would break on the apostrophe in a
# name like C:\Users\O'Brien.
function ConvertTo-TomlString {
    param([string]$Value)
    if ($null -eq $Value) { $Value = '' }
    # A -replace replacement string is literal apart from "$", so two characters
    # in gives two backslashes out; '\\\\' would emit four.
    return '"' + ($Value -replace '\\', '\\' -replace '"', '\"') + '"'
}

# First `^version = "..."` in a Cargo.toml, which in a manifest is [package]'s.
# Anchored with no leading whitespace so an indented version under
# [dependencies.foo] cannot win, the same way install.sh's `grep -m1 '^version'`
# cannot. `version.workspace = true` has no "=" after "version" and is skipped.
function Get-CrateVersion {
    param([Parameter(Mandatory = $true)][string]$ManifestPath)
    if (-not (Test-Path -LiteralPath $ManifestPath)) { return $null }
    foreach ($line in [System.IO.File]::ReadAllLines($ManifestPath)) {
        if ($line -match '^version\s*=\s*"([^"]+)"') { return $Matches[1] }
    }
    return $null
}

function Get-TopicVersion {
    param([Parameter(Mandatory = $true)][string]$SourceDir)
    # Wrapped because this runs after a verified binary is already on disk: an
    # unreadable header must leave the manifest one key short, never abort the
    # install between placing the binary and recording it. A tag old enough not
    # to have this file is a supported input, not an error.
    try {
        $header = Join-Path $SourceDir 'horus_core\src\communication\topic\header.rs'
        if (-not (Test-Path -LiteralPath $header)) { return $null }
        foreach ($line in [System.IO.File]::ReadAllLines($header)) {
            if ($line -match 'TOPIC_VERSION\s*:\s*u32\s*=\s*(\d+)') { return $Matches[1] }
        }
    } catch { }
    return $null
}

# Refuse early on a toolchain that cannot build HORUS, rather than several
# minutes into a cargo run as an error about a package the user has never heard
# of. The floor is read from the workspace manifest so there is one number and
# it lives in one place. Mirrors install.sh's check_rust_version().
function Assert-RustVersion {
    param([Parameter(Mandatory = $true)][string]$SourceDir)
    $required = $null
    $workspaceManifest = Join-Path $SourceDir 'Cargo.toml'
    if (Test-Path -LiteralPath $workspaceManifest) {
        foreach ($line in [System.IO.File]::ReadAllLines($workspaceManifest)) {
            if ($line -match '^rust-version\s*=\s*"([^"]+)"') { $required = $Matches[1]; break }
        }
    }
    if (-not $required) { return }

    if (-not (Get-Command rustc -ErrorAction SilentlyContinue)) { return }
    $rustc = Get-NativeOutput -FilePath 'rustc' -Arguments @('--version')
    if ($rustc.ExitCode -ne 0 -or -not $rustc.Output) { return }
    # "rustc 1.90.0 (hash date)" -> 1.90.0, and drop any -beta/-nightly suffix.
    $found = ($rustc.Output -split '\s+')[1]
    $found = ($found -split '-')[0]

    # Compare as version numbers, not as strings: 1.100 is newer than 1.9.
    $rv = $null; $fv = $null
    if (-not [version]::TryParse($required, [ref]$rv)) { return }
    if (-not [version]::TryParse($found, [ref]$fv)) { return }
    if ($fv -lt $rv) {
        throw "Rust $required or newer is required; found $found. Run: rustup update stable"
    }
    Write-OK "Rust $found (>= $required required)"
}

function Install-HorusBinaryFile {
    param([Parameter(Mandatory = $true)][string]$Source, [Parameter(Mandatory = $true)][string]$Destination)

    $dir = Split-Path -Parent $Destination
    $leaf = Split-Path -Leaf $Destination

    # Sweep up what a previous run left behind (see below), now that nothing
    # holds those files open.
    Get-ChildItem -LiteralPath $dir -Filter "$leaf.old-*" -ErrorAction SilentlyContinue |
        ForEach-Object { Remove-Item -LiteralPath $_.FullName -Force -ErrorAction SilentlyContinue }

    if (Test-Path -LiteralPath $Destination) {
        # Windows refuses to overwrite or delete a running image, and horus.exe
        # may well be running — a `horus monitor` in another pane, a daemon, an
        # editor terminal. It does allow *renaming* one, so move the old binary
        # aside and drop the new one in; the rename takes effect for every future
        # launch and the stale file goes on the next install.
        $stamp = [DateTime]::UtcNow.ToString('yyyyMMddHHmmss', [System.Globalization.CultureInfo]::InvariantCulture)
        try {
            Move-Item -LiteralPath $Destination -Destination "$Destination.old-$stamp" -Force
        } catch {
            throw "Could not replace $Destination ($($_.Exception.Message)). Close any running 'horus' process and re-run."
        }
    }
    Move-Item -LiteralPath $Source -Destination $Destination -Force
}

# A direct registry write tells no running process anything, so even a *newly
# opened* console — which inherits its environment from the Explorer that was
# already running — would still carry the old PATH, and "open a new terminal"
# would be false advice. .NET's SetEnvironmentVariable broadcasts this for us;
# when we have to write the registry ourselves (below) we broadcast ourselves.
function Send-EnvironmentChanged {
    try {
        if (-not ('HorusInstaller.NativeMethods' -as [type])) {
            Add-Type -Namespace 'HorusInstaller' -Name 'NativeMethods' -MemberDefinition @'
[System.Runtime.InteropServices.DllImport("user32.dll", SetLastError = true, CharSet = System.Runtime.InteropServices.CharSet.Auto)]
public static extern System.IntPtr SendMessageTimeout(
    System.IntPtr hWnd, uint Msg, System.IntPtr wParam, string lParam,
    uint fuFlags, uint uTimeout, out System.UIntPtr lpdwResult);
'@
        }
        $result = [System.UIntPtr]::Zero
        # HWND_BROADCAST, WM_SETTINGCHANGE, SMTO_ABORTIFHUNG, 5s — a hung tray
        # app must not be able to hang an installer.
        [void][HorusInstaller.NativeMethods]::SendMessageTimeout(
            [System.IntPtr]0xffff, 0x1A, [System.IntPtr]::Zero, 'Environment', 2, 5000, [ref]$result)
    } catch {
        # Compiling a P/Invoke needs a C# compiler and is refused outright under
        # constrained language mode. The user is told to open a new terminal
        # either way, so this is not worth failing an install over.
    }
}

function Add-ToUserPath {
    param([Parameter(Mandatory = $true)][string]$Directory)

    # HKCU\Environment, not $env:Path. $env:Path is the *merged* machine+user
    # PATH: writing it back to the user scope copies every machine entry into the
    # user's own PATH, and the copy then shadows the machine one forever.
    #
    # DoNotExpandEnvironmentNames because the user PATH Windows 10/11 ships is
    # REG_EXPAND_SZ holding %USERPROFILE%\AppData\Local\Microsoft\WindowsApps.
    # [Environment]::GetEnvironmentVariable(...,'User') hands back the *expanded*
    # text, and writing that back bakes today's profile path in permanently.
    #
    # And not `setx`, which truncates the value it writes at 1024 characters.
    $key = $null
    $current = $null
    $kind = [Microsoft.Win32.RegistryValueKind]::String
    try {
        $key = [Microsoft.Win32.Registry]::CurrentUser.OpenSubKey('Environment', $true)
        if ($key) {
            $current = $key.GetValue('Path', $null, [Microsoft.Win32.RegistryValueOptions]::DoNotExpandEnvironmentNames)
            try { $kind = $key.GetValueKind('Path') } catch { $kind = [Microsoft.Win32.RegistryValueKind]::String }
        }
    } catch {
        $key = $null
    }
    if ($null -eq $current) {
        $current = [Environment]::GetEnvironmentVariable('Path', 'User')
    }
    if ($null -eq $current) { $current = '' }

    # Idempotent, and by entry rather than by substring: C:\opt\horus\bin must not
    # count as present because C:\opt\horus\bin2 is. Trailing separators and case
    # differ between hand-edited PATHs; neither makes it a different directory.
    $needle = ([string]$Directory).TrimEnd('\', '/')
    foreach ($entry in ($current -split ';')) {
        $e = $entry.Trim().Trim('"').TrimEnd('\', '/')
        if ($e -and [string]::Equals($e, $needle, [System.StringComparison]::OrdinalIgnoreCase)) {
            if ($key) { $key.Close() }
            return $false
        }
    }

    $updated = if ([string]::IsNullOrWhiteSpace($current)) { $Directory } else { $current.TrimEnd(';') + ';' + $Directory }

    if ($key -and $kind -eq [Microsoft.Win32.RegistryValueKind]::ExpandString) {
        # Preserve REG_EXPAND_SZ: [Environment]::SetEnvironmentVariable writes
        # REG_SZ, which would turn the %USERPROFILE% entries Windows ships with
        # into literal text for good.
        $key.SetValue('Path', $updated, [Microsoft.Win32.RegistryValueKind]::ExpandString)
        $key.Close()
        Send-EnvironmentChanged
    } else {
        if ($key) { $key.Close() }
        [Environment]::SetEnvironmentVariable('Path', $updated, 'User')
    }
    return $true
}

# Two files, and nothing has written the first one since v0.2.0: version.rs:32-44
# reads ~\.horus\installed_version and uninstall.ps1 deletes the directory that
# holds it, but no installer ever created it — so the version gate has been dead
# code and the remedy it printed could not have fixed anything.
#
# installed_version stays a bare version string with a trailing newline because
# that is what read_to_string().trim() there expects. install_manifest.toml is
# the richer record doctor.rs and `horus self update` read; readers must tolerate
# its absence (older installs) and the absence of individual keys — an air-gapped
# tree has no commit, and a tag old enough may not have the header TOPIC_VERSION
# is parsed from.
function Write-InstallState {
    param(
        [Parameter(Mandatory = $true)][string]$StateDir,
        [Parameter(Mandatory = $true)][string]$SourceVersion,
        [string]$Tag,
        [string]$Commit,
        [Parameter(Mandatory = $true)][string]$SourceDir,
        [Parameter(Mandatory = $true)][string]$BinaryPath,
        [Parameter(Mandatory = $true)][string]$Method
    )

    try {
        New-Item -ItemType Directory -Path $StateDir -Force | Out-Null
    } catch {
        Write-Warn "Could not create $StateDir - version state not recorded"
        return
    }

    # UTF8Encoding($false): Set-Content -Encoding UTF8 writes a BOM in Windows
    # PowerShell 5.1, and version.rs compares the file's contents to
    # CARGO_PKG_VERSION after nothing more than .trim() — a leading U+FEFF makes
    # every install look like a mismatch against itself.
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)

    [System.IO.File]::WriteAllText((Join-Path $StateDir 'installed_version'), "$SourceVersion`n", $utf8NoBom)

    $topicVersion = Get-TopicVersion -SourceDir $SourceDir
    $lines = New-Object System.Collections.Generic.List[string]
    $lines.Add('# Written by install.ps1. Describes the tree this install came from;')
    $lines.Add('# horus doctor and horus self update read it. Do not hand-edit.')
    $lines.Add("version = $(ConvertTo-TomlString $SourceVersion)")
    $lines.Add("tag = $(ConvertTo-TomlString $Tag)")
    $lines.Add("commit = $(ConvertTo-TomlString $Commit)")
    if ($topicVersion) {
        # The number the two halves actually have to agree on: a CLI built at
        # TOPIC_VERSION 3 cannot read a topic written by libraries at 4.
        $lines.Add("topic_version = $topicVersion")
    } else {
        $lines.Add('# topic_version could not be read from the source tree')
    }
    $lines.Add("source_dir = $(ConvertTo-TomlString $SourceDir)")
    $lines.Add("binary = $(ConvertTo-TomlString $BinaryPath)")
    $lines.Add("install_method = $(ConvertTo-TomlString $Method)")
    $lines.Add("installed_at = $(ConvertTo-TomlString ([DateTime]::UtcNow.ToString('yyyy-MM-ddTHH:mm:ssZ', [System.Globalization.CultureInfo]::InvariantCulture)))")

    [System.IO.File]::WriteAllText((Join-Path $StateDir 'install_manifest.toml'), (($lines -join "`r`n") + "`r`n"), $utf8NoBom)
    Write-OK "Recorded $SourceVersion in $StateDir\installed_version"
}

# ============================================================================
# MAIN
# ============================================================================

# Saved and restored below: run through `irm ... | iex` these assignments land in
# the *caller's* session, and leaving the preference on Stop changes how every
# later command in that console behaves.
$PrevErrorActionPreference = $ErrorActionPreference
$PrevProgressPreference = $ProgressPreference
$ErrorActionPreference = 'Stop'
# Invoke-WebRequest repaints a progress bar per chunk in Windows PowerShell 5.1,
# which costs several times the download itself on a release asset.
$ProgressPreference = 'SilentlyContinue'

$HorusTmp = $null
$StageDir = $null
$PrevReleaseLto = $null
$LtoOverridden = $false
$StateRecorded = $false
$InstallStart = Get-Date

try {
    if (-not (Test-IsWindows)) {
        throw "install.ps1 is the Windows installer. On Linux and macOS use install.sh: curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | bash"
    }

    # TLS 1.2 is not the default in Windows PowerShell 5.1 on an unpatched
    # machine, and github.com has refused everything below it since 2018 — the
    # symptom is "The request was aborted: Could not create SSL/TLS secure
    # channel", which reads like a proxy problem. PowerShell 7 negotiates on its
    # own through HttpClient and ignores this.
    if ($PSVersionTable.PSEdition -ne 'Core') {
        try {
            [Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12
        } catch { }
    }

    # $env:USERPROFILE is what uninstall.ps1 uses and what dirs::home_dir()
    # resolves to for the CLI (paths.rs:14), so all three agree on where ~ is.
    $HomeDir = $env:USERPROFILE
    if ([string]::IsNullOrWhiteSpace($HomeDir)) { $HomeDir = $HOME }
    if ([string]::IsNullOrWhiteSpace($HomeDir)) { $HomeDir = [Environment]::GetFolderPath('UserProfile') }
    if ([string]::IsNullOrWhiteSpace($HomeDir)) { throw 'Could not determine the home directory (USERPROFILE is unset).' }

    # Reported for the banner. Only amd64 is published for Windows, so that is
    # the asset either way; a 32-bit PowerShell on 64-bit Windows reports x86 in
    # PROCESSOR_ARCHITECTURE and the truth in PROCESSOR_ARCHITEW6432.
    $Arch = $env:PROCESSOR_ARCHITEW6432
    if ([string]::IsNullOrWhiteSpace($Arch)) { $Arch = $env:PROCESSOR_ARCHITECTURE }
    if ([string]::IsNullOrWhiteSpace($Arch)) { $Arch = 'AMD64' }

    # HORUS_PREFIX relocates everything the installer writes: <prefix>\bin for the
    # binary and <prefix> in place of ~\.horus for the state files and the source
    # cache. Without it the root is ~\.horus, which is what version.rs and
    # uninstall.ps1 already read, and ~\.cargo\bin, which is the only directory
    # uninstall.ps1 looks in for the binary (uninstall.ps1:21-22).
    if (-not [string]::IsNullOrWhiteSpace($Prefix)) {
        $StateDir = $Prefix
        $InstallDir = Join-Path $Prefix 'bin'
    } else {
        $CargoHome = if ($env:CARGO_HOME) { $env:CARGO_HOME } else { Join-Path $HomeDir '.cargo' }
        $StateDir = Join-Path $HomeDir '.horus'
        $InstallDir = Join-Path $CargoHome 'bin'
    }
    $CacheDir = Join-Path $StateDir 'cache'
    $BinaryPath = Join-Path $InstallDir $BinaryName

    Write-Host ''
    Write-Host '============================================' -ForegroundColor Blue
    Write-Host '   HORUS Installation Script (Windows)' -ForegroundColor White
    Write-Host '   Deterministic Real-Time Robotics Framework' -ForegroundColor White
    Write-Host '============================================' -ForegroundColor Blue
    Write-Host ''
    Write-Host "  OS:      Windows ($Arch)"
    Write-Host "  Install: $InstallDir"
    Write-Host "  State:   $StateDir"
    Write-Host ''

    if (-not [Environment]::Is64BitOperatingSystem) {
        throw "HORUS publishes no 32-bit Windows build (release.yml builds x86_64-pc-windows-msvc only), and the workspace is not tested on x86."
    }
    if ($Arch -eq 'ARM64') {
        Write-Warn 'No windows-arm64 release asset exists; installing the amd64 build, which runs under emulation.'
        Write-Host "      For a native binary instead: `$env:HORUS_BUILD_FROM_SOURCE = '1'"
    }

    # --- Resolve the one ref that drives this whole install ---
    $SourceRef = ''        # the ref cloned; empty for a local tree
    $ReleaseTag = ''       # the tag the binary comes from
    $UseSource = [bool]$BuildFromSource
    $InstallMethod = 'release-binary'

    if (-not [string]::IsNullOrWhiteSpace($LocalSource)) {
        # The offline/air-gapped path. Every other route through this script
        # needs github.com, so a machine without it could not install at all. A
        # local tree has no tag, hence no matching release binary: pairing one
        # with it would recreate the exact skew this section exists to prevent.
        $UseSource = $true
        $InstallMethod = 'local-source'
    } elseif (-not [string]::IsNullOrWhiteSpace($InstallBranch)) {
        # The developer escape hatch used to change only the clone while the
        # binary still came from releases/latest — so the one documented way to
        # "pin" an install was the surest way to manufacture the skew. A branch
        # has no release binary; it forces a source build.
        if (-not (Test-ValidRef $InstallBranch)) {
            throw "HORUS_INSTALL_BRANCH='$InstallBranch' is not a usable git ref"
        }
        $SourceRef = $InstallBranch
        $UseSource = $true
        $InstallMethod = 'source-build'
        Write-Warn "HORUS_INSTALL_BRANCH=$InstallBranch - building from source at that branch"
        Write-Host '      No release binary is downloaded: a branch tree and a tagged binary'
        Write-Host '      are different code, and installing both is what broke topic IPC.'
    } else {
        if (-not [string]::IsNullOrWhiteSpace($Version)) {
            # Accept 0.4.0 and v0.4.0 alike; the tags themselves are v-prefixed.
            $ReleaseTag = 'v' + ($Version -replace '^v', '')
            Write-Info "Installing pinned release $ReleaseTag"
        } else {
            Write-Info 'Resolving the latest release...'
            $ReleaseTag = Resolve-LatestTag -Repository $Repo
            if ([string]::IsNullOrWhiteSpace($ReleaseTag)) {
                # Deliberately no fallback to main: cloning main while the binary
                # came from a tag is the bug. Every alternative below names one ref.
                Write-Err "Could not resolve the latest release of $Repo."
                Write-Host '      github.com may be unreachable, or there may be no published release.'
                Write-Host "      Pin a release:       `$env:HORUS_VERSION = 'v0.4.0'"
                Write-Host "      Build from a branch: `$env:HORUS_INSTALL_BRANCH = 'main'"
                Write-Host "      Use a local tree:    `$env:HORUS_LOCAL_SOURCE = 'C:\src\horus'"
                throw "No release tag could be resolved for $Repo."
            }
        }
        if (-not (Test-ValidRef $ReleaseTag)) {
            throw "Refusing to use '$ReleaseTag' as a release tag"
        }
        $SourceRef = $ReleaseTag
        if ($UseSource) { $InstallMethod = 'source-build' }
        Write-OK "Release $ReleaseTag"
    }

    # Pinned to the resolved tag rather than /releases/latest/download/: "latest"
    # names whatever is newest at the moment of each request, so it can move
    # between the asset fetch and the SHA256SUMS fetch, and it is a different
    # tree from the source unless the tag is written out explicitly. Both halves,
    # one tag.
    $ReleaseUrl = "https://github.com/$Repo/releases/download/$ReleaseTag/$AssetFile"
    $ChecksumUrl = "https://github.com/$Repo/releases/download/$ReleaseTag/SHA256SUMS"

    # --- Source tree ---
    #
    # Required regardless of how the binary is obtained: `horus run`/`horus build`
    # generate .horus\Cargo.toml with horus as *path* dependencies (cargo_gen.rs
    # -> find_horus_source_dir). A binary-only install produces a CLI that cannot
    # build a single Rust project. So: always place the source in the cache, and
    # treat a pre-built binary purely as a way to skip the compile step.
    New-Item -ItemType Directory -Path $CacheDir -Force | Out-Null

    if (-not [string]::IsNullOrWhiteSpace($LocalSource)) {
        # Used where it lies. Copying it would duplicate a checkout that carries a
        # multi-GB target\, and an installer must never point a recursive delete
        # at a path the user handed it.
        if (-not (Test-Path -LiteralPath $LocalSource -PathType Container)) {
            throw "HORUS_LOCAL_SOURCE='$LocalSource' is not a directory"
        }
        $SrcTree = (Resolve-Path -LiteralPath $LocalSource).ProviderPath
        # Same two markers the clone is checked against: horus\Cargo.toml is what
        # find_horus_source_dir() looks for and horus_core\Cargo.toml is what the
        # version is parsed from, so a wrong path fails here, not at the build.
        if (-not (Test-Path -LiteralPath (Join-Path $SrcTree 'horus\Cargo.toml')) -or
            -not (Test-Path -LiteralPath (Join-Path $SrcTree 'horus_core\Cargo.toml'))) {
            throw "HORUS_LOCAL_SOURCE='$SrcTree' is not a HORUS source tree (no horus\Cargo.toml)"
        }
        Write-Info "Using local source at $SrcTree (no network)"
    } else {
        if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
            throw 'git is required. Install it with: winget install --id Git.Git'
        }

        Write-Info "Fetching HORUS source ($SourceRef)..."
        # Staged inside the cache rather than under $env:TEMP so the move below is
        # a rename on one volume: TEMP is routinely redirected to another drive,
        # and a cross-volume Move-Item copies a full checkout. The name does not
        # start with "horus@", so find_horus_source_dir()'s scan of the cache
        # ignores a stage directory an interrupted run left behind.
        $StageDir = Join-Path $CacheDir ('.stage-' + [guid]::NewGuid().ToString('N'))
        # --branch takes a tag as readily as a branch name, which is the whole
        # point: $SourceRef is the same tag the binary is downloaded from.
        $code = Invoke-Native -FilePath 'git' -Arguments @(
            'clone', '--depth', '1', '--branch', $SourceRef, "https://github.com/$Repo.git", $StageDir)
        if ($code -ne 0) {
            throw "Failed to clone https://github.com/$Repo.git at ref '$SourceRef' (git exited $code)"
        }
        # A clone can also exit 0 with a tree that is unusable to `horus run`.
        # Both markers are checked *before* the destructive delete below, so a bad
        # fetch can never remove a working cached source tree.
        if (-not (Test-Path -LiteralPath (Join-Path $StageDir 'horus\Cargo.toml')) -or
            -not (Test-Path -LiteralPath (Join-Path $StageDir 'horus_core\Cargo.toml'))) {
            throw "Fetched tree for ref '$SourceRef' is incomplete (no horus\Cargo.toml)"
        }
        $SrcTree = $StageDir
    }

    # Version the cache dir by the crate version so multiple installs coexist and
    # find_horus_source_dir() can prefer the tree matching the running CLI.
    $SrcVersion = Get-CrateVersion -ManifestPath (Join-Path $SrcTree 'horus_core\Cargo.toml')
    # The parsed string becomes a path component below, and the delete further
    # down removes whatever it names. It is read out of a tree that was just
    # downloaded, so a "\" or ".." in it would point that delete outside the
    # cache. A crate version is alphanumerics with `. + - _` and nothing else.
    if (-not $SrcVersion -or $SrcVersion -notmatch '^[A-Za-z0-9+_-][A-Za-z0-9.+_-]*\z') {
        throw "Refusing to use version '$SrcVersion' from horus_core\Cargo.toml as a directory name"
    }

    # horus@<version> is the name run_rust.rs:1062 builds from CARGO_PKG_VERSION
    # and registry/helpers.rs:1135 hardcodes. Renaming it breaks both.
    $HorusSrcDir = Join-Path $CacheDir "horus@$SrcVersion"
    if ($StageDir) {
        Remove-CachedSourceDir -Path $HorusSrcDir
        Move-Item -LiteralPath $StageDir -Destination $HorusSrcDir -Force
        $StageDir = $null
    } elseif (-not [string]::Equals($SrcTree, $HorusSrcDir, [System.StringComparison]::OrdinalIgnoreCase)) {
        # Link, do not copy, and never recursively delete the user's own tree. The
        # link is what lets find_horus_source_dir() resolve an air-gapped install
        # without the user exporting HORUS_SOURCE by hand. A junction rather than
        # a symlink because creating a symlink needs either elevation or developer
        # mode, and a junction needs neither.
        Remove-CachedSourceDir -Path $HorusSrcDir
        $linked = $false
        foreach ($kind in @('Junction', 'SymbolicLink')) {
            try {
                New-Item -ItemType $kind -Path $HorusSrcDir -Target $SrcTree -ErrorAction Stop | Out-Null
                Write-Info "Linked $HorusSrcDir -> $SrcTree ($kind)"
                $linked = $true
                break
            } catch { }
        }
        if (-not $linked) {
            Write-Warn "Could not link $HorusSrcDir -> $SrcTree"
            Write-Host "      Set HORUS_SOURCE=$SrcTree so 'horus run' can find the source."
            $HorusSrcDir = $SrcTree
        }
    }

    # Record what was actually cached. A tag alone does not identify a tree once
    # tags move, and the commit is the only thing that survives being compared
    # against a bug report.
    $SrcCommit = ''
    if (Get-Command git -ErrorAction SilentlyContinue) {
        $rev = Get-NativeOutput -FilePath 'git' -Arguments @('-C', $HorusSrcDir, 'rev-parse', 'HEAD')
        if ($rev.ExitCode -eq 0) { $SrcCommit = $rev.Output.Trim() }
    }
    Write-OK "Source cached at $HorusSrcDir"
    $refLabel = if ($SourceRef) { $SourceRef } else { '<local tree>' }
    $commitLabel = if ($SrcCommit) { $SrcCommit } else { 'unknown' }
    Write-Info "  ref $refLabel, commit $commitLabel"

    # Not $env:TEMP itself and never assigned back to it: TEMP is the variable
    # cargo, rustc, link.exe and git all consult, and pointing it at a directory
    # this script deletes on the way out takes every child process's scratch space
    # with it. Own private directory, removed from the one finally below so it
    # also goes on the error paths.
    $HorusTmp = Join-Path ([System.IO.Path]::GetTempPath()) ('horus-install-' + [guid]::NewGuid().ToString('N'))
    New-Item -ItemType Directory -Path $HorusTmp -Force | Out-Null

    # --- Binary ---
    $HaveBinary = $false
    if ($UseSource) {
        if ($env:HORUS_BUILD_FROM_SOURCE -eq '1' -or $BuildFromSource) {
            Write-Info 'Building from source - skipping the pre-built binary'
        }
    } else {
        Write-Info "Checking for pre-built binary ($ReleaseTag)..."
        $zipPath = Join-Path $HorusTmp $AssetFile
        try {
            Invoke-HorusFetch -Uri $ReleaseUrl -OutFile $zipPath
            $HaveBinary = (Test-Path -LiteralPath $zipPath) -and ((Get-Item -LiteralPath $zipPath).Length -gt 0)
        } catch {
            # Not an error: the release matrix does not cover every platform, and
            # a tag old enough may predate the Windows job. Say what happened and
            # fall through to the compile.
            Write-Warn "No usable $AssetFile at $ReleaseTag ($($_.Exception.Message))"
            $HaveBinary = $false
        }
    }

    if ($HaveBinary) {
        # --- Fast path: pre-built binary, skip the compile ---
        #
        # Verify the download against the release's published SHA256SUMS before
        # installing it. The release workflow has always published this file
        # (release.yml:185) and SECURITY.md claims "Package Verification: ...
        # checksum verification" — but no installer ever fetched it, so a tampered
        # or truncated asset was executed unchecked. TLS alone does not cover a
        # compromised or substituted asset. Every branch here fails closed.
        Write-Info 'Verifying checksum...'
        $sumsPath = Join-Path $HorusTmp 'SHA256SUMS'
        try {
            Invoke-HorusFetch -Uri $ChecksumUrl -OutFile $sumsPath
        } catch {
            throw "Could not fetch SHA256SUMS from $ChecksumUrl ($($_.Exception.Message)) - refusing to install an unverified binary. Build from source instead: `$env:HORUS_BUILD_FROM_SOURCE = '1'"
        }
        if (-not (Test-Path -LiteralPath $sumsPath) -or (Get-Item -LiteralPath $sumsPath).Length -eq 0) {
            throw "SHA256SUMS at $ChecksumUrl is empty - refusing to install an unverified binary."
        }

        $expected = Get-Sha256SumsEntry -SumsPath $sumsPath -FileName $AssetFile
        if (-not $expected) {
            throw "SHA256SUMS for $ReleaseTag has no entry for $AssetFile. Refusing to install an unverified binary."
        }
        if (-not (Get-Command Get-FileHash -ErrorAction SilentlyContinue)) {
            # Get-FileHash arrived in PowerShell 4.0. Older hosts get the same
            # answer install.sh gives a machine with no sha256sum: build instead.
            throw "Get-FileHash is unavailable (PowerShell $($PSVersionTable.PSVersion)), so the download cannot be verified. Build from source with `$env:HORUS_BUILD_FROM_SOURCE = '1'."
        }
        $actual = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
        if (-not [string]::Equals($expected, $actual, [System.StringComparison]::Ordinal)) {
            Write-Err "Checksum MISMATCH for $AssetFile"
            Write-Err "  expected: $expected"
            Write-Err "  actual:   $actual"
            throw 'Refusing to install. This asset does not match the published release.'
        }
        Write-OK 'Checksum verified'

        Write-Info 'Extracting binary...'
        $unpackDir = Join-Path $HorusTmp 'unpacked'
        Expand-HorusZip -ZipPath $zipPath -Destination $unpackDir
        # Compress-Archive puts horus.exe at the root of the zip (release.yml:131),
        # but an archive that unpacked without the binary in it is not an install:
        # a layout change would otherwise surface as a bare "Move-Item: cannot find
        # path". Search below the root as a courtesy, then insist.
        $extracted = Join-Path $unpackDir $BinaryName
        if (-not (Test-Path -LiteralPath $extracted)) {
            $found = Get-ChildItem -LiteralPath $unpackDir -Filter $BinaryName -Recurse -File -ErrorAction SilentlyContinue |
                Select-Object -First 1
            if (-not $found) { throw "$AssetFile does not contain $BinaryName" }
            $extracted = $found.FullName
        }

        New-Item -ItemType Directory -Path $InstallDir -Force | Out-Null
        Install-HorusBinaryFile -Source $extracted -Destination $BinaryPath
        Write-OK "Downloaded pre-built binary ($ReleaseTag)"
    } else {
        # --- Slow path: compile the cached source ---
        if (-not $UseSource) {
            Write-Warn "No pre-built binary for windows-amd64 at $ReleaseTag - building from source (~5-10 min)"
            $InstallMethod = 'source-build'
        }

        if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
            # install.sh pipes rustup into sh here. The Windows equivalent is
            # downloading and running rustup-init.exe, which then wants the MSVC
            # build tools — a multi-GB Visual Studio component install with its own
            # licence prompt. That is not something to start unattended from a
            # one-liner with no TTY to ask, so say what is needed and stop.
            Write-Err 'cargo was not found, and building HORUS from source needs a Rust toolchain.'
            Write-Host '      Install Rust: winget install --id Rustlang.Rustup'
            Write-Host '      rustup offers to install the MSVC build tools it needs; accept that.'
            Write-Host '      Or see https://rustup.rs, then re-run this installer.'
            throw 'A Rust toolchain is required to build from source.'
        }
        Assert-RustVersion -SourceDir $HorusSrcDir
        Write-OK 'Dependencies ready'

        Write-Host ''
        Write-Info 'Building from source (this takes a few minutes)...'
        Write-Host ''
        $buildStart = Get-Date
        Push-Location -LiteralPath $HorusSrcDir
        try {
            # Force stable — nightly may have compiler bugs. First try with LTO
            # (smaller binary); if LLVM crashes (SIGILL, a known bug on some CPUs),
            # retry without it.
            $code = Invoke-Native -FilePath 'cargo' -Arguments @('+stable', 'build', '--release', '-p', 'horus_manager')
            if ($code -ne 0) {
                Write-Host ''
                Write-Warn 'Release build failed (possible LLVM/LTO bug), retrying without LTO...'
                Write-Host ''
                # Restored in the finally below: through `irm ... | iex` this
                # assignment outlives the install and would quietly change how
                # every later cargo build in that console links.
                $PrevReleaseLto = $env:CARGO_PROFILE_RELEASE_LTO
                $LtoOverridden = $true
                $env:CARGO_PROFILE_RELEASE_LTO = 'off'
                $code = Invoke-Native -FilePath 'cargo' -Arguments @('+stable', 'build', '--release', '-p', 'horus_manager')
            }
        } finally {
            Pop-Location
        }
        if ($code -ne 0) {
            throw "Build failed (cargo exited $code). Report issues: https://github.com/$Repo/issues"
        }

        # CARGO_TARGET_DIR may redirect the output tree, so ask where cargo
        # actually put things rather than assuming .\target.
        $targetRoot = if ($env:CARGO_TARGET_DIR) { $env:CARGO_TARGET_DIR } else { Join-Path $HorusSrcDir 'target' }
        $builtBin = Join-Path $targetRoot "release\$BinaryName"
        if (-not (Test-Path -LiteralPath $builtBin)) {
            throw "Build succeeded but the binary is not at $builtBin. Report issues: https://github.com/$Repo/issues"
        }
        New-Item -ItemType Directory -Path $InstallDir -Force | Out-Null
        Copy-Item -LiteralPath $builtBin -Destination (Join-Path $HorusTmp $BinaryName) -Force
        Install-HorusBinaryFile -Source (Join-Path $HorusTmp $BinaryName) -Destination $BinaryPath
        Write-OK ("Built and installed in {0:N0}s" -f ((Get-Date) - $buildStart).TotalSeconds)
    }

    # --- Verify ---
    #
    # The exit status, not just the file's existence. A binary that unpacked but
    # cannot start — the wrong architecture, a missing Visual C++ runtime — used
    # to be reported as a successful install on the Unix side, and the user found
    # out at their first `horus run`.
    if (-not (Test-Path -LiteralPath $BinaryPath)) {
        throw "Installation failed: nothing at $BinaryPath"
    }
    $versionErr = Join-Path $HorusTmp 'version.err'
    $versionRun = Get-NativeOutput -FilePath $BinaryPath -Arguments @('--version') -StdErrFile $versionErr
    if ($versionRun.ExitCode -ne 0) {
        Write-Err 'The installed binary does not run on this system.'
        if (Test-Path -LiteralPath $versionErr) {
            Get-Content -LiteralPath $versionErr -Tail 5 | ForEach-Object { Write-Host "      $_" }
        }
        Write-Host '      A missing VCRUNTIME140.dll here means the Visual C++ redistributable'
        Write-Host '      is not installed: winget install --id Microsoft.VCRedist.2015+.x64'
        Write-Host "      The binary that failed is at $BinaryPath."
        throw "$BinaryName exited $($versionRun.ExitCode) on --version."
    }

    # `horus --version` prints "horus <x.y.z>" (clap, main.rs:14 — the
    # CARGO_PKG_VERSION of horus_manager). Keep only the version so the messages
    # below do not read "horus horus 0.4.0".
    $InstalledVersion = (($versionRun.Output -split '\s+') | Where-Object { $_ } | Select-Object -Last 1)

    # The binary and the cached source must be one tree. If they are not, the CLI
    # cannot read the shared memory its own libraries write and the symptom
    # appears hours later as "Incompatible topic version". Compare here, where it
    # is one string comparison, rather than there. horus_manager is what --version
    # reports; horus_core is what named the cache directory. They are the same
    # number in this workspace, and a build where they are not is a repo bug, not
    # a skew — so prefer horus_manager's and fall back to the cache version.
    $expectedVersion = Get-CrateVersion -ManifestPath (Join-Path $HorusSrcDir 'horus_manager\Cargo.toml')
    if (-not $expectedVersion) { $expectedVersion = $SrcVersion }
    if (-not [string]::Equals($InstalledVersion, $expectedVersion, [System.StringComparison]::Ordinal)) {
        Write-Err "Version skew: the binary reports $InstalledVersion, the cached source is $expectedVersion."
        Write-Host "      Binary: $BinaryPath"
        Write-Host "      Source: $HorusSrcDir (ref $refLabel)"
        Write-Host '      These must be one tree. Build both halves from the same tag:'
        Write-Host "        `$env:HORUS_BUILD_FROM_SOURCE = '1'"
        throw 'The installed binary and the cached source are not the same tree.'
    }
    Write-OK "Verified: horus $InstalledVersion"

    # --- Record what was installed ---
    Write-InstallState -StateDir $StateDir -SourceVersion $SrcVersion -Tag $ReleaseTag -Commit $SrcCommit `
        -SourceDir $HorusSrcDir -BinaryPath $BinaryPath -Method $InstallMethod
    $StateRecorded = $true

    # --- PATH ---
    if (Add-ToUserPath -Directory $InstallDir) {
        Write-OK "Added $InstallDir to your user PATH"
        $PathWasChanged = $true
    } else {
        Write-OK 'PATH already configured'
        $PathWasChanged = $false
    }
    # The user PATH is read at process start, so this session only picks the
    # change up if we put it there too.
    if (($env:Path -split ';' | ForEach-Object { $_.TrimEnd('\', '/') }) -notcontains $InstallDir.TrimEnd('\', '/')) {
        $env:Path = "$InstallDir;$env:Path"
    }

    # --- Completions ---
    #
    # `horus completion powershell` emits a Register-ArgumentCompleter block and
    # has worked since the CLI shipped, but nothing on Windows ever placed it. The
    # profile gets one dot-source line, deliberately shaped so uninstall.ps1's
    # cleanup (uninstall.ps1:120-122, which strips lines matching both "horus" and
    # "completion") removes exactly this line and nothing else of the user's.
    if ($NoShellIntegration) {
        Write-Info 'Skipping shell completions (HORUS_NO_SHELL_INTEGRATION is set)'
    } elseif ($PROFILE) {
        try {
            $compDir = Join-Path $StateDir 'completions'
            $compFile = Join-Path $compDir 'horus.ps1'
            $comp = Get-NativeOutput -FilePath $BinaryPath -Arguments @('completion', 'powershell')
            if ($comp.ExitCode -eq 0 -and $comp.Output) {
                New-Item -ItemType Directory -Path $compDir -Force | Out-Null
                [System.IO.File]::WriteAllText($compFile, $comp.Output + "`r`n", (New-Object System.Text.UTF8Encoding($false)))
                New-Item -ItemType Directory -Path (Split-Path -Parent $PROFILE) -Force | Out-Null
                $profileText = if (Test-Path -LiteralPath $PROFILE) { Get-Content -LiteralPath $PROFILE -Raw } else { '' }
                if ($null -eq $profileText) { $profileText = '' }
                if ($profileText -notmatch [regex]::Escape($compFile)) {
                    # Appended with a leading newline in case the profile does not
                    # end with one — otherwise the dot-source would be glued to the
                    # user's last statement.
                    Add-Content -LiteralPath $PROFILE -Value "`r`n. `"$compFile`"  # horus completions"
                }
                Write-OK "Shell completions installed: $compFile"
            }
        } catch {
            # Never fail an install over completions.
            Write-Warn "Could not install completions: $($_.Exception.Message)"
        }
    }

    # --- Done ---
    $elapsed = ((Get-Date) - $InstallStart).TotalSeconds
    Write-Host ''
    Write-Host ("  Installation complete!  horus $InstalledVersion  ({0:N0}s)" -f $elapsed) -ForegroundColor Green
    Write-Host ''
    Write-Host '  Get started:'
    Write-Host '    horus new my_robot -r     Create a Rust project' -ForegroundColor Cyan
    Write-Host '    horus new my_robot -p     Create a Python project' -ForegroundColor Cyan
    Write-Host '    horus doctor              Check your environment' -ForegroundColor Cyan
    Write-Host ''
    Write-Host '  Docs: https://docs.horusrobotics.dev' -ForegroundColor Cyan
    Write-Host ''
    if ($PathWasChanged) {
        # An already-open console keeps the environment it started with, so the
        # PATH entry written above is invisible to every window that exists now,
        # including this one beyond the in-process fix-up.
        Write-Warn 'Open a NEW terminal for horus to be on your PATH.'
        Write-Host ''
    }
    if (-not [string]::IsNullOrWhiteSpace($Prefix)) {
        # uninstall.ps1 finds a relocated binary through $env:HORUS_PREFIX, and
        # the install record it otherwise reads lives under the prefix too — not
        # in ~\.horus where it looks. So the variable has to be set for the
        # uninstall as well, or it cleans up a different install than this one.
        Write-Info "Installed under $Prefix - set `$env:HORUS_PREFIX = '$Prefix' before running uninstall.ps1."
    }
} catch {
    Write-Host ''
    Write-Err $_.Exception.Message
    Write-Host ''
    # installed_version and install_manifest.toml are written only after the
    # binary has run, so a failure before that point leaves the version gate
    # saying nothing rather than naming a release that is not there. Said only
    # when it is true: a failure after the record is written must not claim the
    # opposite.
    if (-not $StateRecorded) {
        Write-Host '  No install was recorded, so nothing claims this version is installed.'
    }
    Write-Host "  Report issues: https://github.com/$Repo/issues"
    Write-Host ''
    # Rethrown, not `exit 1`: a bare `exit` closes the console window when this
    # script is run through `irm ... | iex`, and a terminating error is what both
    # `pwsh -File install.ps1` and `pwsh -Command "irm ... | iex"` report as a
    # nonzero exit status to CI.
    throw
} finally {
    if ($StageDir -and (Test-Path -LiteralPath $StageDir)) {
        Remove-Item -LiteralPath $StageDir -Recurse -Force -ErrorAction SilentlyContinue
    }
    if ($HorusTmp -and (Test-Path -LiteralPath $HorusTmp)) {
        Remove-Item -LiteralPath $HorusTmp -Recurse -Force -ErrorAction SilentlyContinue
    }
    if ($LtoOverridden) { $env:CARGO_PROFILE_RELEASE_LTO = $PrevReleaseLto }
    $ErrorActionPreference = $PrevErrorActionPreference
    $ProgressPreference = $PrevProgressPreference
}
