'use client';

import Link from 'next/link';
import { useState } from 'react';
import { DocsNav } from '@/components/DocsNav';
import { FiGithub, FiPlay, FiCopy, FiCheck } from 'react-icons/fi';

// Terminal/code snippet component
function InstallSnippet() {
  const [copied, setCopied] = useState(false);
  const commands = [
    'git clone https://github.com/softmata/horus.git',
    'cd horus && ./install.sh'
  ];

  const copyToClipboard = () => {
    navigator.clipboard.writeText(commands.join(' && '));
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div className="relative rounded-xl overflow-hidden border border-[var(--border)] bg-[#0a0e14] shadow-2xl shadow-black/20">
      {/* Terminal header */}
      <div className="flex items-center justify-between px-4 py-3 bg-[#0d1117] border-b border-[var(--border)]">
        <div className="flex items-center gap-2">
          <div className="flex gap-1.5">
            <div className="w-3 h-3 rounded-full bg-[#ff5f56]" />
            <div className="w-3 h-3 rounded-full bg-[#ffbd2e]" />
            <div className="w-3 h-3 rounded-full bg-[#27ca40]" />
          </div>
          <span className="text-xs text-[var(--text-tertiary)] font-mono ml-2">terminal</span>
        </div>
        <button
          onClick={copyToClipboard}
          className="flex items-center gap-1.5 text-xs text-[var(--text-tertiary)] hover:text-[var(--accent)] transition-all px-2 py-1 rounded hover:bg-white/5 active:scale-95"
        >
          {copied ? (
            <FiCheck className="w-3.5 h-3.5 text-green-400" />
          ) : (
            <FiCopy className="w-3.5 h-3.5" />
          )}
          {copied ? 'Copied!' : 'Copy'}
        </button>
      </div>
      {/* Terminal content */}
      <div className="p-4 font-mono text-sm space-y-2">
        {commands.map((cmd, i) => (
          <div key={i} className="flex">
            <span className="text-[var(--accent)] mr-2 select-none">$</span>
            <span className="text-[#e6edf3]">{cmd}</span>
          </div>
        ))}
      </div>
    </div>
  );
}

export default function DocsLandingPage() {
  return (
    <div className="min-h-screen flex flex-col bg-[var(--bg-primary)]">
      <DocsNav />

      <main className="flex-grow relative overflow-hidden">
        {/* Background grid - static, no animation flash */}
        <div
          className="absolute inset-0 opacity-[0.03]"
          style={{
            backgroundImage: `linear-gradient(var(--accent) 1px, transparent 1px), linear-gradient(90deg, var(--accent) 1px, transparent 1px)`,
            backgroundSize: '50px 50px'
          }}
        />

        {/* Gradient overlay */}
        <div className="absolute inset-0 bg-gradient-to-b from-[var(--accent)]/5 via-transparent to-transparent" />

        {/* Content */}
        <div className="relative">
          <div className="max-w-4xl mx-auto px-4 sm:px-6 lg:px-8 py-16 lg:py-24">
            <div className="text-center">

              {/* Version badge - fade in from top */}
              <div
                className="inline-flex items-center gap-2 mb-8 px-4 py-2 rounded-full bg-[var(--surface)]/80 backdrop-blur-sm border border-[var(--border)] opacity-0 animate-fade-in-down"
                style={{ animationDelay: '0.1s' }}
              >
                <span className="w-2 h-2 rounded-full bg-[var(--accent)] animate-pulse" />
                <span className="text-sm font-medium text-[var(--text-secondary)]">v0.1.6</span>
                <span className="text-[var(--text-tertiary)]">·</span>
                <span className="text-sm text-[var(--text-tertiary)]">Sub-microsecond IPC</span>
              </div>

              {/* Title - fade in up with stagger */}
              <h1
                className="text-4xl md:text-5xl lg:text-6xl font-bold tracking-tight mb-6 opacity-0 animate-fade-in-up"
                style={{ animationDelay: '0.2s' }}
              >
                <span className="text-[var(--text-primary)]">HORUS </span>
                <span className="bg-gradient-to-r from-[var(--accent)] to-[var(--success)] bg-clip-text text-transparent">Documentation</span>
              </h1>

              {/* Subtitle */}
              <p
                className="text-lg md:text-xl text-[var(--text-secondary)] mb-10 leading-relaxed max-w-2xl mx-auto opacity-0 animate-fade-in-up"
                style={{ animationDelay: '0.3s' }}
              >
                High-performance robotics framework for real-time systems.
                <br className="hidden sm:block" />
                575x faster than ROS2. Start building in 5 minutes.
              </p>

              {/* CTA Buttons */}
              <div
                className="flex flex-wrap justify-center gap-3 mb-16 opacity-0 animate-fade-in-up"
                style={{ animationDelay: '0.4s' }}
              >
                <Link
                  href="/getting-started/installation"
                  className="group inline-flex items-center gap-2 px-6 py-3 rounded-lg bg-[var(--accent)] text-white font-medium transition-all duration-200 hover:shadow-lg hover:shadow-[var(--accent)]/25 hover:-translate-y-0.5 active:translate-y-0"
                >
                  <FiPlay className="w-4 h-4 transition-transform group-hover:scale-110" />
                  Get Started
                </Link>
                <Link
                  href="https://github.com/softmata/horus"
                  target="_blank"
                  rel="noopener noreferrer"
                  className="group inline-flex items-center gap-2 px-6 py-3 rounded-lg bg-[var(--surface)] border border-[var(--border)] text-[var(--text-primary)] font-medium transition-all duration-200 hover:border-[var(--accent)]/50 hover:bg-[var(--accent)]/5 hover:-translate-y-0.5 active:translate-y-0"
                >
                  <FiGithub className="w-4 h-4 transition-transform group-hover:scale-110" />
                  GitHub
                </Link>
              </div>

              {/* Install Command */}
              <div
                className="max-w-2xl mx-auto opacity-0 animate-fade-in-up"
                style={{ animationDelay: '0.5s' }}
              >
                <p className="text-xs text-[var(--text-tertiary)] mb-3 font-medium uppercase tracking-widest">
                  Quick Install
                </p>
                <InstallSnippet />
              </div>

              {/* Quick links grid */}
              <div
                className="grid grid-cols-2 sm:grid-cols-4 gap-4 mt-16 max-w-3xl mx-auto opacity-0 animate-fade-in-up"
                style={{ animationDelay: '0.6s' }}
              >
                <Link
                  href="/architecture"
                  className="group p-5 rounded-lg border border-[var(--border)] bg-[var(--surface)]/30 hover:border-[var(--accent)]/50 hover:bg-[var(--surface)]/50 transition-all"
                >
                  <div className="text-[var(--text-primary)] font-medium mb-1 group-hover:text-[var(--accent)] transition-colors">Architecture</div>
                  <div className="text-xs text-[var(--text-tertiary)]">System design →</div>
                </Link>
                <Link
                  href="/development/cli-reference"
                  className="group p-5 rounded-lg border border-[var(--border)] bg-[var(--surface)]/30 hover:border-[var(--accent)]/50 hover:bg-[var(--surface)]/50 transition-all"
                >
                  <div className="text-[var(--text-primary)] font-medium mb-1 group-hover:text-[var(--accent)] transition-colors">CLI Reference</div>
                  <div className="text-xs text-[var(--text-tertiary)]">Commands →</div>
                </Link>
                <Link
                  href="/performance/benchmarks"
                  className="group p-5 rounded-lg border border-[var(--border)] bg-[var(--surface)]/30 hover:border-[var(--accent)]/50 hover:bg-[var(--surface)]/50 transition-all"
                >
                  <div className="text-[var(--text-primary)] font-medium mb-1 group-hover:text-[var(--accent)] transition-colors">Benchmarks</div>
                  <div className="text-xs text-[var(--text-tertiary)]">Performance →</div>
                </Link>
                <Link
                  href="/basic-examples"
                  className="group p-5 rounded-lg border border-[var(--border)] bg-[var(--surface)]/30 hover:border-[var(--accent)]/50 hover:bg-[var(--surface)]/50 transition-all"
                >
                  <div className="text-[var(--text-primary)] font-medium mb-1 group-hover:text-[var(--accent)] transition-colors">Examples</div>
                  <div className="text-xs text-[var(--text-tertiary)]">Code samples →</div>
                </Link>
              </div>

            </div>

          </div>
        </div>
      </main>
    </div>
  );
}

