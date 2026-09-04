# PyPI Setup Guide for HORUS

This document explains how to set up automatic PyPI publishing for the HORUS Python bindings (`horus-robotics` package).

> **Note**: Only the Python bindings are published to PyPI. The main HORUS framework
> is distributed through the GitHub `release` branch and one-line installer.
> Rust workspace crates are NOT published to crates.io.

## One-Time Setup

### 1. Create PyPI Account

1. Go to https://pypi.org/account/register/
2. Create an account
3. Verify your email

### 2. Register the Package Name (First Release Only)

Option A: Manual registration through web interface
- Go to https://pypi.org/manage/projects/
- Click "Add a new project"
- Follow the upload process with your first wheel

Option B: Automatic registration
- The first release will automatically register the name
- The package is published as "horus-robotics" on PyPI

### 3. Configure Trusted Publishing

**There is no token to create.** `build-wheels.yml` publishes with PyPI's
trusted publishing (OIDC): the `release` job declares `environment: pypi` and
`id-token: write`, and `pypa/gh-action-pypi-publish` exchanges that for a
short-lived credential at upload time. Nothing is stored in GitHub Secrets,
so there is nothing to leak or rotate.

This is not optional or an "alternative" — the workflow reads no token, so a
`PYPI_TOKEN` secret would sit unread. That mismatch is what broke v0.4.0: the
job was already configured for trusted publishing and then authenticated the
old way with `maturin upload`, which read `MATURIN_PYPI_TOKEN` from a secret
that was never set, and every upload returned

```
⛔ 403 Invalid or non-existent authentication information.
```

so 0.2.x, 0.3.0 and 0.4.0 were built, tested on twelve platform/Python
combinations, and never published. PyPI stayed on 0.1.9.

**One-time setup**, at
https://pypi.org/manage/project/horus-robotics/settings/publishing/ — the
project already exists, so this is the project's own publishing settings, not
the "pending publisher" form for unclaimed names:

| Field | Value |
| --- | --- |
| Owner | `softmata` |
| Repository | `horus` |
| Workflow name | `build-wheels.yml` |
| Environment name | `pypi` |

All four must match exactly. The environment name is the one on the `release`
job in `build-wheels.yml`; changing either without the other silently breaks
publishing again, and the failure only shows up on a tag.

## Troubleshooting

### Build Fails
- Check GitHub Actions logs for specific errors
- Common issues:
  - Rust compilation errors (fix code)
  - Missing dependencies (update workflow)
  - Version conflicts (check version numbers match)

### Upload Fails
- `403 Invalid or non-existent authentication information` means the trusted
  publisher does not match. Check all four fields above — owner, repository,
  workflow filename, environment — against the `release` job.
- A version already on PyPI means the tag did not bump
  `horus_py/pyproject.toml`. The job deliberately sets no `skip-existing`, so
  this is a red run rather than a quiet pass. PyPI versions are immutable;
  bump and cut a new tag.

### Wheel Not Found for Platform
- Check the build matrix in `.github/workflows/build-wheels.yml`
- Ensure all target platforms built successfully
- May need to add platform-specific dependencies

## Security Notes

### Token Safety
- ✅ Never commit tokens to git
- ✅ Use GitHub Secrets for storage
- ✅ Regenerate tokens if exposed
- ✅ Use project-scoped tokens when possible

## Version Management

### Semantic Versioning
- `0.1.x` → `0.1.y`: Bug fixes, small changes
- `0.1.x` → `0.2.0`: New features (backward compatible)
- `0.x.y` → `1.0.0`: Breaking changes

### Pre-releases
For testing before official release:
```bash
git tag v0.2.0-rc1    # Release candidate
git tag v0.2.0-beta1  # Beta release
git tag v0.2.0-alpha1 # Alpha release
```

Users can install with:
```bash
pip install --pre horus-robotics  # Install pre-release versions
```

## Yanking Releases

If you publish a broken version:

1. Go to https://pypi.org/project/horus-robotics/
2. Click on the version
3. Click "Options" → "Yank version"
4. Provide a reason

This hides the version but doesn't delete it (PyPI policy).

## Support

For issues:
- GitHub Actions problems: Check workflow logs
- PyPI problems: https://pypi.org/help/
- Package issues: Open GitHub issue
