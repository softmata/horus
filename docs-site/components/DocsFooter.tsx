import Link from "next/link";
import { FiGithub, FiMessageCircle, FiExternalLink } from "react-icons/fi";

export function DocsFooter() {
  return (
    <footer className="border-t border-[var(--border)] bg-[var(--surface)]">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-12">
        {/* Main Footer Content */}
        <div className="grid grid-cols-2 md:grid-cols-4 gap-8 lg:gap-12">
          {/* Getting Started */}
          <div>
            <h4 className="font-semibold text-[var(--text)] mb-4 text-sm">
              Getting Started
            </h4>
            <ul className="space-y-3 text-sm">
              <li>
                <Link href="/getting-started/installation" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Installation
                </Link>
              </li>
              <li>
                <Link href="/getting-started/quick-start" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Quick Start
                </Link>
              </li>
              <li>
                <Link href="/complete-beginners-guide" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Beginner&apos;s Guide
                </Link>
              </li>
              <li>
                <Link href="/basic-examples" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Examples
                </Link>
              </li>
            </ul>
          </div>

          {/* Documentation */}
          <div>
            <h4 className="font-semibold text-[var(--text)] mb-4 text-sm">
              Documentation
            </h4>
            <ul className="space-y-3 text-sm">
              <li>
                <Link href="/architecture" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Architecture
                </Link>
              </li>
              <li>
                <Link href="/development/cli-reference" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  CLI Reference
                </Link>
              </li>
              <li>
                <Link href="/built-in-nodes" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Built-in Nodes
                </Link>
              </li>
              <li>
                <Link href="/api" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  API Reference
                </Link>
              </li>
            </ul>
          </div>

          {/* Resources */}
          <div>
            <h4 className="font-semibold text-[var(--text)] mb-4 text-sm">
              Resources
            </h4>
            <ul className="space-y-3 text-sm">
              <li>
                <Link href="/performance/benchmarks" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Benchmarks
                </Link>
              </li>
              <li>
                <a
                  href="https://marketplace.horus-registry.dev/"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors inline-flex items-center gap-1"
                >
                  Marketplace
                  <FiExternalLink className="w-3 h-3" />
                </a>
              </li>
              <li>
                <Link href="/goals" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Goals & Vision
                </Link>
              </li>
              <li>
                <Link href="/troubleshooting" className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors">
                  Troubleshooting
                </Link>
              </li>
            </ul>
          </div>

          {/* Community */}
          <div>
            <h4 className="font-semibold text-[var(--text)] mb-4 text-sm">
              Community
            </h4>
            <ul className="space-y-3 text-sm">
              <li>
                <a
                  href="https://github.com/softmata/horus"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors inline-flex items-center gap-1"
                >
                  <FiGithub className="w-4 h-4" />
                  GitHub
                </a>
              </li>
              <li>
                <a
                  href="https://discord.gg/hEZC3ev2Nf"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors inline-flex items-center gap-1"
                >
                  <FiMessageCircle className="w-4 h-4" />
                  Discord
                </a>
              </li>
              <li>
                <a
                  href="https://github.com/softmata/horus/issues"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors inline-flex items-center gap-1"
                >
                  Report Issues
                  <FiExternalLink className="w-3 h-3" />
                </a>
              </li>
              <li>
                <a
                  href="https://github.com/softmata/horus/blob/main/CONTRIBUTING.md"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors inline-flex items-center gap-1"
                >
                  Contributing
                  <FiExternalLink className="w-3 h-3" />
                </a>
              </li>
            </ul>
          </div>
        </div>

        {/* Bottom Bar */}
        <div className="mt-10 pt-6 border-t border-[var(--border)] flex flex-col sm:flex-row justify-between items-center gap-4">
          <div className="flex items-center gap-2">
            <span className="font-bold text-[var(--text)]">HORUS</span>
            <span className="text-sm text-[var(--text-tertiary)]">
              &copy; {new Date().getFullYear()} HORUS Contributors
            </span>
          </div>
          <div className="flex items-center gap-6 text-sm text-[var(--text-tertiary)]">
            <a
              href="https://github.com/softmata/horus/blob/main/LICENSE"
              target="_blank"
              rel="noopener noreferrer"
              className="hover:text-[var(--text-secondary)] transition-colors"
            >
              Apache-2.0 License
            </a>
            <span className="hidden sm:inline">|</span>
            <span className="hidden sm:inline">Built with Rust</span>
          </div>
        </div>
      </div>
    </footer>
  );
}
