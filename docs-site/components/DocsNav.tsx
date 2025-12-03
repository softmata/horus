"use client";

import Link from "next/link";
import { FiGithub, FiMenu, FiSearch, FiX } from "react-icons/fi";
import { ThemeToggle } from "./ThemeToggle";
import { SearchModal } from "./SearchModal";
import { useState, useEffect } from "react";

interface DocsNavProps {
  onMenuClick?: () => void;
}

export function DocsNav({ onMenuClick }: DocsNavProps) {
  const [isSearchOpen, setIsSearchOpen] = useState(false);
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false);

  // Keyboard shortcut for search (Cmd+K / Ctrl+K)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.metaKey || e.ctrlKey) && e.key === "k") {
        e.preventDefault();
        setIsSearchOpen(true);
      }
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => document.removeEventListener("keydown", handleKeyDown);
  }, []);

  const navLinks = [
    { href: "/getting-started/installation", label: "Get Started" },
    { href: "/architecture", label: "Architecture" },
    { href: "/built-in-nodes", label: "Nodes" },
    { href: "/performance/benchmarks", label: "Benchmarks" },
  ];

  return (
    <>
      <nav className="sticky top-0 z-50 w-full border-b border-[var(--border)] bg-[var(--bg-primary)]/95 backdrop-blur-md">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex h-14 items-center justify-between">
            {/* Left: Logo + Nav Links */}
            <div className="flex items-center gap-8">
              <div className="flex items-center gap-3">
                {/* Hamburger menu for mobile */}
                {onMenuClick ? (
                  <button
                    onClick={onMenuClick}
                    className="lg:hidden p-2 -ml-2 hover:bg-[var(--surface)] rounded-md transition-colors"
                    aria-label="Open sidebar"
                  >
                    <FiMenu className="w-5 h-5 text-[var(--text-secondary)]" />
                  </button>
                ) : (
                  <button
                    onClick={() => setIsMobileMenuOpen(!isMobileMenuOpen)}
                    className="md:hidden p-2 -ml-2 hover:bg-[var(--surface)] rounded-md transition-colors"
                    aria-label="Toggle menu"
                  >
                    {isMobileMenuOpen ? (
                      <FiX className="w-5 h-5 text-[var(--text-secondary)]" />
                    ) : (
                      <FiMenu className="w-5 h-5 text-[var(--text-secondary)]" />
                    )}
                  </button>
                )}

                <Link
                  href="/"
                  className="flex items-center gap-2 font-bold text-lg text-[var(--text-primary)] hover:text-[var(--accent)] transition-colors"
                >
                  <span className="text-[var(--accent)]">HORUS</span>
                  <span className="text-[var(--text-tertiary)] font-normal text-sm">Docs</span>
                </Link>
              </div>

              {/* Desktop Nav Links */}
              <div className="hidden md:flex items-center gap-1">
                {navLinks.map((link) => (
                  <Link
                    key={link.href}
                    href={link.href}
                    className="px-3 py-1.5 text-sm text-[var(--text-secondary)] hover:text-[var(--text-primary)] hover:bg-[var(--surface)] rounded-md transition-colors"
                  >
                    {link.label}
                  </Link>
                ))}
              </div>
            </div>

            {/* Right: Search + Actions */}
            <div className="flex items-center gap-2">
              {/* Search Button */}
              <button
                onClick={() => setIsSearchOpen(true)}
                className="flex items-center gap-2 px-3 py-1.5 text-sm bg-[var(--surface)] border border-[var(--border)] rounded-lg text-[var(--text-tertiary)] hover:text-[var(--text-secondary)] hover:border-[var(--border-hover)] transition-colors"
                aria-label="Search documentation"
              >
                <FiSearch className="w-4 h-4" />
                <span className="hidden sm:inline">Search</span>
                <kbd className="hidden lg:inline-flex items-center gap-0.5 ml-2 px-1.5 py-0.5 text-[10px] font-medium bg-[var(--bg-primary)] border border-[var(--border)] rounded text-[var(--text-tertiary)]">
                  <span>⌘</span>K
                </kbd>
              </button>

              {/* Marketplace - gradient button */}
              <a
                href="https://marketplace.horus-registry.dev/"
                target="_blank"
                rel="noopener noreferrer"
                className="hidden sm:inline-flex items-center px-3 py-1.5 text-sm font-medium bg-gradient-to-r from-[var(--accent)] to-[var(--success)] text-white rounded-md hover:opacity-90 transition-opacity"
              >
                Marketplace
              </a>

              <div className="flex items-center gap-1 ml-1">
                <ThemeToggle />
                <a
                  href="https://github.com/softmata/horus"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="p-2 text-[var(--text-tertiary)] hover:text-[var(--text-primary)] hover:bg-[var(--surface)] rounded-md transition-colors"
                  title="GitHub"
                  aria-label="GitHub Repository"
                >
                  <FiGithub className="w-5 h-5" />
                </a>
              </div>
            </div>
          </div>
        </div>

        {/* Mobile Menu Dropdown */}
        {isMobileMenuOpen && !onMenuClick && (
          <div className="md:hidden border-t border-[var(--border)] bg-[var(--bg-primary)]">
            <div className="px-4 py-3 space-y-1">
              {navLinks.map((link) => (
                <Link
                  key={link.href}
                  href={link.href}
                  onClick={() => setIsMobileMenuOpen(false)}
                  className="block px-3 py-2 text-sm text-[var(--text-secondary)] hover:text-[var(--text-primary)] hover:bg-[var(--surface)] rounded-md transition-colors"
                >
                  {link.label}
                </Link>
              ))}
              <a
                href="https://marketplace.horus-registry.dev/"
                target="_blank"
                rel="noopener noreferrer"
                className="block px-3 py-2 text-sm text-[var(--accent)] hover:bg-[var(--surface)] rounded-md transition-colors"
              >
                Marketplace
              </a>
            </div>
          </div>
        )}
      </nav>

      {/* Search Modal */}
      <SearchModal isOpen={isSearchOpen} onClose={() => setIsSearchOpen(false)} />
    </>
  );
}
