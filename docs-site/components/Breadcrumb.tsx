"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { FiChevronRight, FiHome } from "react-icons/fi";

interface BreadcrumbItem {
  label: string;
  href: string;
}

// Mapping of URL segments to human-readable labels
const segmentLabels: Record<string, string> = {
  // Directories
  "core-concepts": "Core Concepts",
  "development": "Development",
  "package-management": "Package Management",
  "multi-language": "Multi-Language",
  "performance": "Performance",
  "advanced": "Advanced Topics",
  "api": "API Reference",
  "built-in-nodes": "Built-in Nodes",
  "getting-started": "Getting Started",

  // Common file names
  "what-is-horus": "What is HORUS?",
  "goals": "Goals & Vision",
  "complete-beginners-guide": "Complete Beginner's Guide",
  "installation": "Installation",
  "quick-start": "Quick Start",
  "second-application": "Second Application",
  "architecture": "Architecture",
  "migration-guide-ctx-api": "Migration Guide",
  "troubleshooting": "Troubleshooting",
  "troubleshooting-runtime": "Runtime Errors",
  "examples": "Examples",
  "basic-examples": "Basic Examples",
  "advanced-examples": "Advanced Examples",

  // Core concepts
  "core": "Overview",
  "core-concepts-nodes": "Nodes",
  "core-concepts-hub": "Hub (MPMC)",
  "core-concepts-link": "Link (SPSC)",
  "core-concepts-scheduler": "Scheduler",
  "core-concepts-shared-memory": "Shared Memory",
  "communication-overview": "Communication Patterns",
  "communication-transport": "Communication Transport",
  "communication-configuration": "Configuration",
  "network-communication": "Network",
  "node-macro": "node! Macro",
  "message-macro": "message! Macro",
  "message-types": "Message Types",
  "realtime-nodes": "Real-Time Nodes",

  // Development
  "cli-reference": "CLI Reference",
  "monitor": "Monitor",
  "simulation": "Simulation",
  "testing": "Testing",
  "parameters": "Parameters",
  "library-reference": "Library Reference",

  // Package management files
  "using-prebuilt-nodes": "Using Prebuilt Nodes",
  "environment-management": "Environment Management",
  "configuration": "Configuration",

  // Multi-language files
  "python-bindings": "Python Bindings",
  "python-message-library": "Python Message Library",
  "cpp-bindings": "C++ Bindings",
  "ai-integration": "AI Integration",

  // Performance files
  "benchmarks": "Benchmarks",

  // API files
  "api-node": "Node",
  "api-hub": "Hub",
  "api-link": "Link",
  "api-scheduler": "Scheduler",
};

function formatSegment(segment: string): string {
  return segmentLabels[segment] || segment
    .split("-")
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join(" ");
}

export function Breadcrumb() {
  const pathname = usePathname();

  // Don't show breadcrumbs on home page
  if (pathname === "/") {
    return null;
  }

  const segments = pathname.split("/").filter(Boolean);
  const breadcrumbs: BreadcrumbItem[] = [
    { label: "Home", href: "/" }
  ];

  // Build breadcrumb trail
  let currentPath = "";
  segments.forEach((segment, index) => {
    currentPath += `/${segment}`;
    breadcrumbs.push({
      label: formatSegment(segment),
      href: currentPath
    });
  });

  return (
    <nav className="flex items-center space-x-2 text-sm mb-6 text-[var(--text-secondary)]" aria-label="Breadcrumb">
      {breadcrumbs.map((crumb, index) => {
        const isLast = index === breadcrumbs.length - 1;
        const isHome = index === 0;

        return (
          <div key={crumb.href} className="flex items-center">
            {index > 0 && (
              <FiChevronRight className="w-4 h-4 mx-2 text-[var(--border)]" />
            )}
            {isLast ? (
              <span className="text-[var(--text)] font-medium" aria-current="page">
                {isHome ? <FiHome className="w-4 h-4" /> : crumb.label}
              </span>
            ) : (
              <Link
                href={crumb.href}
                className="hover:text-[var(--text)] transition-colors"
              >
                {isHome ? <FiHome className="w-4 h-4" /> : crumb.label}
              </Link>
            )}
          </div>
        );
      })}
    </nav>
  );
}
