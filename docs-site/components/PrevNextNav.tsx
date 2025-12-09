"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { FiArrowLeft, FiArrowRight } from "react-icons/fi";

interface DocLink {
  title: string;
  href: string;
  children?: DocLink[];
}

// Flattened list of all doc pages in order
const allPages: DocLink[] = [
  // Getting Started
  { title: "What is HORUS?", href: "/concepts/what-is-horus" },
  { title: "Goals & Vision", href: "/concepts/goals" },
  { title: "Complete Beginner's Guide", href: "/getting-started/complete-beginners-guide" },
  { title: "Installation", href: "/getting-started/installation" },
  { title: "Quick Start", href: "/getting-started/quick-start" },
  { title: "Second Application", href: "/getting-started/second-application" },
  { title: "Architecture", href: "/concepts/architecture" },
  { title: "Troubleshooting", href: "/troubleshooting" },

  // Core Concepts
  { title: "Concepts Overview", href: "/concepts" },
  { title: "Nodes", href: "/concepts/core-concepts-nodes" },
  { title: "Communication Overview", href: "/concepts/communication-overview" },
  { title: "Hub (MPMC)", href: "/concepts/core-concepts-hub" },
  { title: "Link (SPSC)", href: "/concepts/core-concepts-link" },
  { title: "Communication Transport", href: "/concepts/communication-transport" },
  { title: "Shared Memory", href: "/concepts/core-concepts-shared-memory" },
  { title: "Network Communication", href: "/concepts/network-communication" },
  { title: "Communication Configuration", href: "/concepts/communication-configuration" },
  { title: "Scheduler", href: "/concepts/core-concepts-scheduler" },
  { title: "node! Macro", href: "/concepts/node-macro" },
  { title: "message! Macro", href: "/concepts/message-macro" },
  { title: "Message Types", href: "/concepts/message-types" },

  // Rust
  { title: "Rust Overview", href: "/rust" },
  { title: "API Overview", href: "/rust/api" },
  { title: "horus_core", href: "/rust/api/core" },
  { title: "horus_macros", href: "/rust/api/macros" },

  // Python
  { title: "Python Overview", href: "/python" },
  { title: "Python Bindings", href: "/python/api/python-bindings" },
  { title: "Async Nodes", href: "/python/api/async-nodes" },

  // Simulators
  { title: "Simulators Overview", href: "/simulators" },
  { title: "Sim2D Overview", href: "/simulators/sim2d" },
  { title: "Sim3D Overview", href: "/simulators/sim3d" },

  // Development
  { title: "CLI Reference", href: "/development/cli-reference" },
  { title: "Monitor", href: "/development/monitor" },
  { title: "Testing", href: "/development/testing" },

  // Advanced
  { title: "Scheduler Configuration", href: "/advanced/scheduler-configuration" },
  { title: "Execution Modes", href: "/advanced/execution-modes" },
  { title: "Deterministic Execution", href: "/advanced/deterministic-execution" },

  // Performance
  { title: "Optimization Guide", href: "/performance/performance" },
  { title: "Benchmarks", href: "/performance/benchmarks" },
];

export function PrevNextNav() {
  const pathname = usePathname();

  const currentIndex = allPages.findIndex(page => page.href === pathname);
  const prevPage = currentIndex > 0 ? allPages[currentIndex - 1] : null;
  const nextPage = currentIndex < allPages.length - 1 ? allPages[currentIndex + 1] : null;

  if (!prevPage && !nextPage) {
    return null;
  }

  return (
    <nav className="mt-12 pt-6 border-t border-[var(--border)] flex justify-between items-center gap-4">
      {prevPage ? (
        <Link
          href={prevPage.href}
          className="flex items-center gap-2 px-4 py-3 text-sm text-[var(--text-secondary)] hover:text-[var(--text)] hover:bg-[var(--surface)] border border-[var(--border)] transition-colors group"
        >
          <FiArrowLeft className="w-4 h-4 group-hover:-translate-x-1 transition-transform" />
          <div className="text-left">
            <div className="text-xs text-[var(--text-muted)]">Previous</div>
            <div className="font-medium">{prevPage.title}</div>
          </div>
        </Link>
      ) : (
        <div />
      )}

      {nextPage ? (
        <Link
          href={nextPage.href}
          className="flex items-center gap-2 px-4 py-3 text-sm text-[var(--text-secondary)] hover:text-[var(--text)] hover:bg-[var(--surface)] border border-[var(--border)] transition-colors group"
        >
          <div className="text-right">
            <div className="text-xs text-[var(--text-muted)]">Next</div>
            <div className="font-medium">{nextPage.title}</div>
          </div>
          <FiArrowRight className="w-4 h-4 group-hover:translate-x-1 transition-transform" />
        </Link>
      ) : (
        <div />
      )}
    </nav>
  );
}
