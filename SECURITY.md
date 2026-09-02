# Security Policy

## Supported Versions

Security updates will be provided for the following versions:

| Version | Supported                    |
| ------- | ---------------------------- |
| 0.4.x   | Yes                          |
| 0.3.x   | No (tagged, never published) |
| 0.2.x   | No                           |
| < 0.2   | No                           |

### Knowing what you are running

A supported-version table is only useful if a machine can answer which version it
is on, so an install records that:

```bash
horus --version                     # the CLI binary
cat ~/.horus/install_manifest.toml  # version, tag, commit and source tree of the last install
```

The manifest is what makes "0.4.x" a fact rather than a guess: the CLI and the
cached source tree it compiles your projects against are both installed from one
release tag, and the manifest records that tag and its commit. An install made
before the manifest existed has none — `horus --version` is all it can tell you.
Re-run the installer, or `horus self update`, to get one.

The CLI compares those two itself and warns when they have drifted apart. Set
`HORUS_STRICT_VERSION=1` to make the mismatch fatal instead — worth doing on a
build machine, where a silent skew between the CLI and the libraries it compiles
against is a supply-chain question, not a convenience one.

### Getting a patch

```bash
horus self update
```

That replaces the CLI binary and refreshes `~/.horus/cache/horus@<version>` at the
same release tag, verifying each download against that release's `SHA256SUMS`. To
land on a specific release instead:

```bash
curl -fsSL https://github.com/softmata/horus/raw/main/install.sh | HORUS_VERSION=v0.5.0 bash
```

Both halves have to move together. A patched CLI running against the previous
release's cached libraries is not a patched install — your project is compiled
against that cached source.

Python users: the `horus-robotics` package on PyPI is behind the releases here
(0.1.9 at the time of writing), so `pip install -U horus-robotics` does not
deliver fixes to the Python bindings. Build them from the tree the installer
cached — `pip install ~/.horus/cache/horus@<version>/horus_py` — until 0.4.x is
published.

## Reporting a Vulnerability

We take the security of HORUS seriously. If you discover a security issue, please report it responsibly.

### How to Report

**DO NOT** open a public GitHub issue for security vulnerabilities.

Instead, please open a private security advisory via GitHub:

1. Go to the repository's Security tab
2. Click "Report a vulnerability"
3. Fill out the private advisory form

### What to Include

When reporting a security issue, please include:

- Description of the vulnerability
- Steps to reproduce the issue
- Affected versions
- Potential impact
- Any suggested fixes (optional)

### Response Timeline

- **Initial Response**: Within 48 hours
- **Status Update**: Within 7 days
- **Fix Timeline**: Varies based on severity and complexity

### Disclosure Policy

We follow responsible disclosure practices:

1. You report the issue privately
2. We confirm receipt and begin investigation
3. We develop and test a fix
4. We release the fix and publish a security advisory
5. You receive credit in the advisory (if desired)

### Security Best Practices

When using HORUS in production:

- Apply releases with `horus self update` (or the pinned installer one-liner
  above), and check the result with `horus --version` against
  [the latest release](https://github.com/softmata/horus/releases/latest)
- Pin `HORUS_VERSION` on fleet installs, so every robot is provably on the same
  tag and a patch is a deliberate, auditable step rather than whatever the
  installer resolved that day
- Review and validate all packages before installation
- Use authentication for registry operations
- Limit access to shared memory regions (`/dev/shm/horus_*/`)
- Monitor system logs for unusual activity
- Follow principle of least privilege for node permissions

### Scope

This security policy covers:

- HORUS core framework (horus_core)
- HORUS CLI tool (horus_manager)
- Official language bindings (horus_py)
- The installer and update path (`install.sh`, `horus self update`) and the
  release assets they fetch
- The registry and plugin client code in the CLI — the code that fetches,
  verifies and installs packages

It does not currently cover the hosted package registry and marketplace: that
service is not in operation (`api.horusrobotics.dev` returns 503, and
`plugins.horusrobotics.dev` does not resolve), so there is nothing running to
report a vulnerability in. This line comes back when the service does.

Third-party packages in the HORUS ecosystem are the responsibility of their respective maintainers.

## Security Features

HORUS includes the following security features:

- **Memory Safety**: Rust's ownership system prevents memory corruption
- **Type Safety**: Fixed-size message structures prevent buffer overflows
- **Process Isolation**: Shared memory with proper permissions
- **Authentication**: GitHub OAuth for package publishing (against the registry service named in Scope, which is not currently in operation)
- **Install Verification**: `install.sh` and `horus self update` check every release asset against that release's published `SHA256SUMS` before it is made executable — a digest that is missing, unfetchable or mismatched aborts the install rather than warning
- **Package Verification**: Manifest validation and checksum verification

## Acknowledgments

We appreciate the security research community's efforts to improve HORUS. Security researchers who responsibly disclose vulnerabilities will be acknowledged in our security advisories (with permission).
