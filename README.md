# HORUS Install Scripts

This branch contains only the installation scripts. Used by the one-line installer.

## Install

```bash
# Linux/macOS/WSL
curl -fsSL https://horusrobotics.dev/install | bash

# Windows (PowerShell)
irm https://horusrobotics.dev/install.ps1 | iex
```

## Files

- `scripts/install-remote.sh` — One-line installer (Linux/macOS/WSL)
- `scripts/install-remote.ps1` — One-line installer (Windows)
- `install.sh` — Full source build installer
- `uninstall.sh` — Uninstaller (Linux/macOS)
- `uninstall.ps1` — Uninstaller (Windows)
