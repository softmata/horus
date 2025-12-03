# PyPI Setup Guide for HORUS

This document explains how to set up automatic PyPI publishing for the HORUS Python bindings.

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

### 3. Generate PyPI API Token

1. Log in to https://pypi.org
2. Go to Account Settings → API tokens
3. Click "Add API token"
4. Configure:
   - Token name: `GitHub Actions - HORUS`
   - Scope: Select "Entire account" (or "Project: horus-robotics" after first release)
5. Click "Create token"
6. **IMPORTANT:** Copy the token immediately (starts with `pypi-`)
   - You won't be able to see it again!

### 4. Add Token to GitHub Secrets

1. Go to your GitHub repository
2. Navigate to: Settings → Secrets and variables → Actions
3. Click "New repository secret"
4. Configure:
   - Name: `PYPI_TOKEN`
   - Value: Paste the token from step 3 (pypi-...)
5. Click "Add secret"

### 5. Optional: Set Up TestPyPI (Recommended)

TestPyPI allows you to test releases before publishing to production PyPI.

1. Go to https://test.pypi.org/account/register/
2. Create an account (separate from PyPI)
3. Generate an API token (same process as above)
4. Add to GitHub secrets as `TEST_PYPI_TOKEN`

To publish to TestPyPI, modify the workflow or create a separate test workflow.

## Release Process

Once set up, releasing is automatic:

```bash
# 1. Update version numbers
vim horus_py/Cargo.toml      # version = "0.1.6"
vim horus_py/pyproject.toml  # version = "0.1.6"

# 2. Commit changes
git add horus_py/Cargo.toml horus_py/pyproject.toml
git commit -m "Bump version to 0.1.6"

# 3. Create and push tag
git tag v0.1.6
git push origin main --tags

# 4. Wait ~10-15 minutes
# GitHub Actions will:
# - Build wheels for all platforms
# - Test installation
# - Publish to PyPI automatically
```

## Monitoring Releases

### Check GitHub Actions
1. Go to: Actions → Build Python Wheels
2. Watch the build progress
3. Each platform should show green checkmarks

### Verify on PyPI
1. Go to: https://pypi.org/project/horus-robotics/
2. New version should appear within minutes
3. Check that all platforms are available

### Test Installation
```bash
pip install horus-robotics==0.1.6
python -c "import horus; print(horus.__version__)"
```

## Troubleshooting

### Build Fails
- Check GitHub Actions logs for specific errors
- Common issues:
  - Rust compilation errors (fix code)
  - Missing dependencies (update workflow)
  - Version conflicts (check version numbers match)

### Upload Fails
- Check that `PYPI_TOKEN` is set correctly in GitHub secrets
- Verify token has correct permissions
- Ensure version number hasn't been used before (PyPI versions are immutable)

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

### Trusted Publishing (Alternative)

For enhanced security, you can use PyPI's trusted publishing instead of tokens:

1. Go to PyPI → Your project → Settings → Publishing
2. Add a trusted publisher:
   - Owner: your-github-username
   - Repository: horus
   - Workflow: build-wheels.yml
   - Environment: pypi
3. Update workflow to use `id-token: write` (already configured)
4. Remove `PYPI_TOKEN` secret

This is more secure as it doesn't require long-lived tokens.

## Version Management

### Semantic Versioning
- `0.1.6` → `0.1.6`: Bug fixes, small changes
- `0.1.6` → `0.2.0`: New features (backward compatible)
- `0.1.6` → `1.0.0`: Breaking changes

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
