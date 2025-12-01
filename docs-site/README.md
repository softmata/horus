# HORUS Documentation Site

Open-source documentation for the HORUS robotics framework.

## Overview

This is the official documentation site for HORUS - a production-grade, open-source robotics framework built in Rust. The site provides comprehensive guides, API references, and performance benchmarks.

## Running Locally

```bash
# Install dependencies
npm install

# Start development server (port 3009)
npm run dev

# Build for production
npm run build

# Start production server
npm start
```

Visit `http://localhost:3009` to view the documentation.

## Content Structure

```
content/
── docs/                              # Core documentation (30+ pages)
   ── getting-started.mdx
   ── installation.mdx
   ── quick-start.mdx
   ── node-macro.mdx
   ── dashboard.mdx
   ── parameters.mdx
   ── cli-reference.mdx
   ── package-management.mdx        # Package install/publish
   ── environment-management.mdx    # Freeze/restore environments
   ── marketplace.mdx                # Registry and marketplace
   ── authentication.mdx             # GitHub OAuth, API keys
   ── remote-deployment.mdx          # Deploy to robots
   ── library-reference.mdx          # Standard library components
   ── core-concepts-nodes.mdx
   ── core-concepts-hub.mdx
   ── core-concepts-scheduler.mdx
   ── core-concepts-shared-memory.mdx
   ── api-node.mdx                   # Node API reference
   ── api-hub.mdx                    # Hub API reference
   ── api-scheduler.mdx              # Scheduler API reference
   ── message-types.mdx
   ── examples.mdx
   ── performance.mdx
   ── multi-language.mdx             # Python bindings
   ── architecture.mdx
── assets/         # Images and media
```

### Documentation Categories

**Getting Started**
- Installation, Quick Start, node! Macro

**Core Concepts**
- Nodes, Hub (MPMC), Scheduler, Shared Memory

**Guides**
- Dashboard, Parameters, CLI Reference
- Package Management, Environment Management
- Marketplace & Registry, Authentication
- Remote Deployment, Library Reference
- Message Types, Examples, Performance, Multi-Language

**API Reference**
- Node, Hub, Scheduler APIs

## Tech Stack

- **Next.js 14** - React framework with App Router
- **MDX** - Markdown with React components
- **Tailwind CSS** - Utility-first styling
- **Shiki** - Syntax highlighting
- **TypeScript** - Type safety

## Open Source

This documentation site is part of the HORUS open-source project:

- **License**: Apache-2.0
- **Repository**: https://github.com/softmata/horus
- **Framework**: `/HORUS` directory in the main repository

## Contributing

We welcome contributions! To contribute to the documentation:

1. Fork the repository
2. Create a feature branch
3. Make your changes in `content/`
4. Test locally with `npm run dev`
5. Submit a pull request

### Writing Guidelines

**IMPORTANT**: Before editing any `.mdx` files, read [MDX_GUIDELINES.md](./MDX_GUIDELINES.md) to avoid common rendering errors!

Common mistakes to avoid:
- Using `<` directly in text (e.g., `<1%` should be `&lt;1%`)
- Writing generic types without backticks (e.g., `Hub<T>` should be `` `Hub<T>` ``)
- Starting headings with numbers

General guidelines:
- Use clear, concise language
- Include code examples
- Test all code snippets
- Follow existing formatting
- Update navigation if adding new pages
- Run `npm run build` before committing to catch MDX errors

## Performance Focus

The documentation emphasizes HORUS's production-grade performance:

- **87ns-313ns** latency for real robotics messages (Link wait-free / Hub lock-free)
- Production benchmarks with serde serialization
- Real-world message types (CmdVel, LaserScan, IMU, etc.)

## Links

- **Main Repository**: https://github.com/softmata/horus
- **Discord Community**: https://discord.gg/hEZC3ev2Nf
- **Issues**: https://github.com/softmata/horus/issues
- **Discussions**: https://github.com/softmata/horus/discussions
- **Crates.io**: https://crates.io/search?q=horus

## License

Documentation content is licensed under Apache-2.0, matching the HORUS framework license.

---

**Built with ❤️ by the open-source community**
