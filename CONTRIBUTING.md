# Contributing to HORUS

Thank you for your interest in contributing to HORUS! This document provides guidelines for contributing to the project.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/your-username/horus.git
   cd horus
   ```
3. **Create a branch from `dev`** for your feature:
   ```bash
   git checkout dev
   git pull origin dev
   git checkout -b feature/your-feature-name
   ```

**Important:** All pull requests should target the `dev` branch, not `main`. The `main` branch is used for stable releases only.

## Development Setup

### Prerequisites

- Rust 1.92+ (`rustup update`)
- Python 3.9+ with `pip` (optional)
- Build tools (GCC/Clang)
- Node.js 18+ for documentation site

### Building

```bash
# Build all Rust components
cargo build --release

# Build Python bindings (optional)
# First install maturin: cargo install maturin
cd horus_py
maturin develop --release

# Build documentation site (optional)
cd docs-site
npm install
npm run dev
```

## Testing

### Unit and Integration Tests

```bash
# Rust unit tests
cargo test

# Specific component
cargo test -p horus_core

# Python tests
cd horus_py
pytest tests/

# Benchmarks
cd benchmarks
cargo bench
```

### Acceptance Tests

User acceptance tests are in `tests/acceptance/` and document expected behavior.

**Before submitting a PR:**
1. Review relevant acceptance test files
2. Ensure changes align with documented behavior
3. Update tests if changing functionality
4. Add new scenarios for new features

```bash
# Review test documentation
cat tests/acceptance/README.md

# Check specific tests
cat tests/acceptance/horus_manager/01_new_command.md

# Manually verify scenarios
horus new test_project
cd test_project
horus run
```

## Code Style

### Rust

Follow standard conventions:
- Format code when possible: `cargo fmt` (optional, as it may fail in some cases)
- Run `clippy` and address critical issues: `cargo clippy` (warnings are acceptable)
- Document public APIs with `///` comments

```rust
/// Creates a new Topic for inter-process communication.
///
/// # Arguments
///
/// * `topic_name` - The topic name for this channel
///
/// # Examples
///
/// ```
/// let topic = Topic::<f32>::new("temperature")?;
/// ```
pub fn new(topic_name: &str) -> Result<Self> {
    // implementation
}
```

### Python

Follow PEP 8:
- Use `black` for formatting
- Use `mypy` for type checking
- Use descriptive variable names

## What to Contribute

### Good First Issues

Look for issues labeled `good-first-issue`:
- Documentation improvements
- Example programs
- Bug fixes
- Test coverage improvements

### Feature Requests

Before implementing a major feature:
1. Open an issue to discuss the proposal
2. Wait for maintainer feedback
3. Implement with tests and documentation

### Bug Reports

When reporting bugs, include:
- HORUS version (`horus --version`)
- Operating system and version
- Minimal reproducible example
- Expected vs actual behavior
- Relevant logs or error messages

## Pull Request Process

1. **Ensure tests pass**:
   ```bash
   cargo test
   ```

   Optionally run `cargo fmt` (may fail in some cases) and `cargo clippy` (warnings are acceptable).

2. **Check acceptance tests**:
   - Review relevant test files in `tests/acceptance/`
   - Manually verify key scenarios
   - Update test scenarios if behavior changed
   - Add new scenarios for new features

3. **Update documentation**:
   - Update README.md for user-facing changes
   - Update inline code documentation
   - Add examples for new features
   - Update CHANGELOG.md

4. **Write clear commit messages**:
   ```
   Add feature: Brief description

   Detailed explanation of what changed and why.

   - Updated acceptance tests in tests/acceptance/...
   - Added new scenarios for ...

   Fixes #123
   ```

5. **Submit PR** with:
   - **Base branch: `dev`** (not `main`)
   - Clear title and description
   - Link to related issues
   - Screenshots/examples if UI changes
   - List of verified acceptance test scenarios

6. **Address review feedback** promptly

### PR Description Template

```markdown
## Description
Brief description of changes

**Note:** This PR targets the `dev` branch.

## Changes
- Added feature X
- Fixed bug Y
- Updated tests

## Testing
- [ ] Unit tests pass (`cargo test`)
- [ ] Code formatted if possible (`cargo fmt`, optional)
- [ ] Clippy checked (`cargo clippy`, warnings acceptable)
- [ ] Verified acceptance test scenarios
- [ ] Updated/added acceptance tests

## Related Issues
Fixes #123
```

## Architecture Guidelines

### Core Principles

1. **Zero-copy when possible**: Use shared memory
2. **Type safety**: Leverage Rust's type system
3. **Minimal latency**: Profile and optimize hot paths
4. **Multi-language**: Ensure features work across Rust/Python

### Code Organization

```
horus/
├── horus/              # Unified entry crate (use horus::prelude::*)
├── horus_core/         # Core framework (Node, Topic, Scheduler, types, memory)
├── horus_macros/       # Procedural macros (node!, message!)
├── horus_library/      # Standard message types and HFrame transforms
├── horus_manager/      # CLI tool (horus command)
├── horus_py/           # Python bindings
├── benchmarks/         # Performance benchmarks
└── tests/              # Integration tests
```

## What Not to Do

- Break existing APIs without migration path
- Add dependencies without discussion
- Commit without running tests
- Submit PRs without description or testing
- Ignore test failures

## Code Review

All contributions go through code review:
- Be respectful and constructive
- Respond to feedback promptly
- Ask questions if unclear
- Maintainers have final say

## License

By contributing, you agree that your contributions will be licensed under the Apache License 2.0.

All contributors must agree to the [Contributor License Agreement (CLA)](.github/CLA.md). When submitting your first pull request, add a comment:

```
I have read and agree to the Contributor License Agreement.
```

## Thank You!

Every contribution helps make HORUS better. Thank you for being part of the community!

---

Questions? Open an issue or start a discussion on GitHub!
