"use client";

import { useEffect, useState } from "react";

interface Heading {
  id: string;
  text: string;
  level: number;
}

export function TableOfContents() {
  const [headings, setHeadings] = useState<Heading[]>([]);
  const [activeId, setActiveId] = useState<string>("");

  useEffect(() => {
    // Extract all h2 and h3 elements from the main documentation article
    const article = document.querySelector("main article");
    if (!article) return;

    const headingElements = article.querySelectorAll("h2, h3");
    const extractedHeadings: Heading[] = [];

    headingElements.forEach((heading, index) => {
      const text = heading.textContent || "";
      let id = heading.id;

      // Create an ID if it doesn't exist
      if (!id) {
        id = text
          .toLowerCase()
          .replace(/[^a-z0-9]+/g, "-")
          .replace(/(^-|-$)/g, "");
        heading.id = id;
      }

      extractedHeadings.push({
        id,
        text,
        level: parseInt(heading.tagName[1]),
      });
    });

    setHeadings(extractedHeadings);

    // Set up intersection observer for active heading
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (entry.isIntersecting) {
            setActiveId(entry.target.id);
          }
        });
      },
      {
        rootMargin: "0% 0% -80% 0%",
        threshold: 0.1,
      }
    );

    headingElements.forEach((heading) => {
      observer.observe(heading);
    });

    return () => {
      headingElements.forEach((heading) => {
        observer.unobserve(heading);
      });
    };
  }, []);

  if (headings.length === 0) {
    return null;
  }

  const handleClick = (e: React.MouseEvent<HTMLAnchorElement>, id: string) => {
    e.preventDefault();
    const element = document.getElementById(id);
    if (element) {
      const yOffset = -80; // Account for header
      const y = element.getBoundingClientRect().top + window.pageYOffset + yOffset;
      window.scrollTo({ top: y, behavior: "smooth" });
      setActiveId(id);
    }
  };

  return (
    <nav className="hidden xl:block w-64 border-l border-[var(--border)] bg-[var(--bg)] h-[calc(100vh-4rem)] sticky top-16 overflow-y-auto">
      <div className="p-6 space-y-4">
        <div className="mb-6">
          <h3 className="text-sm font-semibold text-[var(--text)] uppercase tracking-wide mb-4">
            On This Page
          </h3>
          <ul className="space-y-2">
            {headings.map((heading) => (
              <li
                key={heading.id}
                className={`${heading.level === 3 ? "ml-4" : ""}`}
              >
                <a
                  href={`#${heading.id}`}
                  onClick={(e) => handleClick(e, heading.id)}
                  className={`block text-sm py-1 transition-all duration-200 border-l-2 pl-3 touch-manipulation ${
                    activeId === heading.id
                      ? "border-[var(--text)] text-[var(--text)] font-medium"
                      : "border-transparent text-[var(--text-secondary)] hover:text-[var(--text)] hover:border-[var(--border)]"
                  }`}
                >
                  {heading.text}
                </a>
              </li>
            ))}
          </ul>
        </div>

        {/* Quick Links */}
        <div className="pt-6 border-t border-[var(--border)]">
          <h4 className="text-xs font-semibold text-[var(--text-secondary)] uppercase tracking-wide mb-3">
            Quick Links
          </h4>
          <ul className="space-y-2">
            <li>
              <a
                href="/getting-started/installation"
                className="text-xs text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors block touch-manipulation py-1"
              >
                Getting Started
              </a>
            </li>
            <li>
              <a
                href="/examples"
                className="text-xs text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors block touch-manipulation py-1"
              >
                Examples
              </a>
            </li>
            <li>
              <a
                href="/api-node"
                className="text-xs text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors block touch-manipulation py-1"
              >
                API Reference
              </a>
            </li>
            <li>
              <a
                href="/benchmarks"
                className="text-xs text-[var(--text-secondary)] hover:text-[var(--text)] transition-colors block touch-manipulation py-1"
              >
                Benchmarks
              </a>
            </li>
          </ul>
        </div>
      </div>
    </nav>
  );
}
