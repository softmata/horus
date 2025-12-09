"use client";

import Link from "next/link";
import { FiGithub, FiMenu, FiSearch } from "react-icons/fi";
import { ThemeToggle } from "./ThemeToggle";
import { SearchModal } from "./SearchModal";
import { useState, useEffect } from "react";

interface DocsNavProps {
  onMenuClick?: () => void;
}

export function DocsNav({ onMenuClick }: DocsNavProps) {
  const [isSearchOpen, setIsSearchOpen] = useState(false);

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

  return (
    <>
      <nav className="sticky top-0 z-50 w-full border-b border-[var(--border)] bg-[var(--bg)]/95 backdrop-blur-sm">
        <div className="px-4 sm:px-6">
          <div className="flex h-14 items-center justify-between">
            {/* Left: Logo */}
            <div className="flex items-center gap-3">
              {/* Hamburger menu for mobile */}
              {onMenuClick && (
                <button
                  onClick={onMenuClick}
                  className="lg:hidden p-2 -ml-2 hover:bg-[var(--surface)] transition-colors"
                  aria-label="Open sidebar"
                >
                  <FiMenu className="w-5 h-5 text-[var(--text-secondary)]" />
                </button>
              )}

              <Link
                href="/"
                className="flex items-center gap-2 font-bold text-lg text-[var(--text)] transition-colors"
              >
                <span className="text-[var(--text)]">HORUS</span>
                <span className="text-[var(--text-tertiary)] font-normal text-sm">Docs</span>
              </Link>
            </div>

            {/* Right: Search + Actions */}
            <div className="flex items-center gap-2">
              {/* Search Button */}
              <button
                onClick={() => setIsSearchOpen(true)}
                className="flex items-center gap-2 px-3 py-1.5 text-sm bg-[var(--surface)] border border-[var(--border)] text-[var(--text-tertiary)] hover:text-[var(--text-secondary)] hover:border-[var(--border-hover)] transition-colors"
                aria-label="Search documentation"
              >
                <FiSearch className="w-4 h-4" />
                <span className="hidden sm:inline">Search</span>
                <kbd className="hidden lg:inline-flex items-center gap-0.5 ml-2 px-1.5 py-0.5 text-[10px] font-medium bg-[var(--bg)] border border-[var(--border)] text-[var(--text-tertiary)]">
                  <span>⌘</span>K
                </kbd>
              </button>

              {/* Marketplace */}
              <a
                href="https://marketplace.horus-registry.dev/"
                target="_blank"
                rel="noopener noreferrer"
                className="hidden sm:inline-flex items-center px-3 py-1.5 text-sm font-medium text-[var(--text)] border border-[var(--border)] hover:border-[var(--border-hover)] hover:bg-[var(--surface)] transition-colors"
              >
                Marketplace
              </a>

              <div className="flex items-center gap-1 ml-1">
                <ThemeToggle />
                <a
                  href="https://github.com/softmata/horus"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="p-2 text-[var(--text-tertiary)] hover:text-[var(--text)] hover:bg-[var(--surface)] transition-colors"
                  title="GitHub"
                  aria-label="GitHub Repository"
                >
                  <FiGithub className="w-5 h-5" />
                </a>
              </div>
            </div>
          </div>
        </div>

      </nav>

      {/* Search Modal */}
      <SearchModal isOpen={isSearchOpen} onClose={() => setIsSearchOpen(false)} />
    </>
  );
}
