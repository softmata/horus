# Security Policy

## Supported Versions

Security updates will be provided for the following versions:

| Version | Supported |
| ------- | --------- |
| 0.1.x   | Yes       |
| < 0.1   | No        |

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

- Keep HORUS updated to the latest version
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
- HORUS package registry and marketplace

Third-party packages in the HORUS ecosystem are the responsibility of their respective maintainers.

## Security Features

HORUS includes the following security features:

- **Memory Safety**: Rust's ownership system prevents memory corruption
- **Type Safety**: Fixed-size message structures prevent buffer overflows
- **Process Isolation**: Shared memory with proper permissions
- **Authentication**: GitHub OAuth for package publishing
- **Package Verification**: Manifest validation and checksum verification

## Acknowledgments

We appreciate the security research community's efforts to improve HORUS. Security researchers who responsibly disclose vulnerabilities will be acknowledged in our security advisories (with permission).
