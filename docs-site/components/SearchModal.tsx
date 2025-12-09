"use client";

import { useState, useEffect, useCallback, useRef } from "react";
import { FiSearch, FiX, FiFileText, FiArrowRight } from "react-icons/fi";
import Link from "next/link";
import { Document } from "flexsearch";

interface SearchDoc {
  id: number;
  title: string;
  description: string;
  slug: string;
  content: string;
  headings: string;
  category: string;
}

interface SearchResult extends SearchDoc {
  score: number;
  highlights: {
    title?: string;
    description?: string;
    content?: string;
  };
}

interface SearchModalProps {
  isOpen: boolean;
  onClose: () => void;
}

// Create FlexSearch index
// eslint-disable-next-line @typescript-eslint/no-explicit-any
let searchIndex: any = null;
let docsCache: SearchDoc[] = [];

function highlightText(text: string, query: string): string {
  if (!query.trim() || !text) return text;

  const terms = query.toLowerCase().split(/\s+/).filter(t => t.length > 1);
  if (terms.length === 0) return text;

  // Create regex pattern for all terms
  const pattern = terms.map(t => t.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')).join('|');
  const regex = new RegExp(`(${pattern})`, 'gi');

  return text.replace(regex, '<mark class="bg-[var(--accent)]/30 text-[var(--accent)] px-0.5 rounded">$1</mark>');
}

function getContentSnippet(content: string, query: string, maxLength = 150): string {
  if (!content) return '';

  const terms = query.toLowerCase().split(/\s+/).filter(t => t.length > 1);
  if (terms.length === 0) return content.substring(0, maxLength) + '...';

  // Find the first occurrence of any search term
  const lowerContent = content.toLowerCase();
  let bestIndex = -1;

  for (const term of terms) {
    const index = lowerContent.indexOf(term);
    if (index !== -1 && (bestIndex === -1 || index < bestIndex)) {
      bestIndex = index;
    }
  }

  if (bestIndex === -1) {
    return content.substring(0, maxLength) + (content.length > maxLength ? '...' : '');
  }

  // Get snippet around the match
  const start = Math.max(0, bestIndex - 40);
  const end = Math.min(content.length, bestIndex + maxLength - 40);
  let snippet = content.substring(start, end);

  if (start > 0) snippet = '...' + snippet;
  if (end < content.length) snippet = snippet + '...';

  return snippet;
}

export function SearchModal({ isOpen, onClose }: SearchModalProps) {
  const [query, setQuery] = useState("");
  const [results, setResults] = useState<SearchResult[]>([]);
  const [loading, setLoading] = useState(false);
  const [selectedIndex, setSelectedIndex] = useState(0);
  const [indexReady, setIndexReady] = useState(false);
  const inputRef = useRef<HTMLInputElement>(null);
  const resultsRef = useRef<HTMLDivElement>(null);

  // Initialize search index
  useEffect(() => {
    async function initSearchIndex() {
      if (searchIndex) {
        setIndexReady(true);
        return;
      }

      try {
        const response = await fetch('/search-index.json');
        const data = await response.json();
        docsCache = data.docs;

        // Create FlexSearch document index
        searchIndex = new Document({
          document: {
            id: 'id',
            index: ['title', 'description', 'content', 'headings'],
            store: true,
          },
          tokenize: 'forward',
          resolution: 9,
          cache: 100,
        });

        // Add documents to index
        for (const doc of docsCache) {
          searchIndex.add(doc);
        }

        setIndexReady(true);
      } catch (error) {
        console.error('Failed to load search index:', error);
      }
    }

    if (isOpen) {
      initSearchIndex();
    }
  }, [isOpen]);

  // Focus input when modal opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      setTimeout(() => inputRef.current?.focus(), 100);
    }
  }, [isOpen]);

  // Handle keyboard navigation
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!isOpen) return;

      if (e.key === "Escape") {
        onClose();
      } else if (e.key === "ArrowDown") {
        e.preventDefault();
        setSelectedIndex(prev => Math.min(prev + 1, results.length - 1));
      } else if (e.key === "ArrowUp") {
        e.preventDefault();
        setSelectedIndex(prev => Math.max(prev - 1, 0));
      } else if (e.key === "Enter" && results[selectedIndex]) {
        e.preventDefault();
        window.location.href = results[selectedIndex].slug;
        onClose();
      }
    };

    document.addEventListener("keydown", handleKeyDown);
    return () => document.removeEventListener("keydown", handleKeyDown);
  }, [isOpen, onClose, results, selectedIndex]);

  // Scroll selected result into view
  useEffect(() => {
    if (resultsRef.current && results.length > 0) {
      const selectedElement = resultsRef.current.children[selectedIndex] as HTMLElement;
      if (selectedElement) {
        selectedElement.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
      }
    }
  }, [selectedIndex, results.length]);

  // Lock body scroll when modal is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = "hidden";
    }
    return () => {
      document.body.style.overflow = "unset";
    };
  }, [isOpen]);

  // Search function with FlexSearch
  const performSearch = useCallback(async (searchQuery: string) => {
    if (!searchQuery.trim() || !searchIndex || !indexReady) {
      setResults([]);
      return;
    }

    setLoading(true);

    try {
      // Search across all indexed fields
      const searchResults = searchIndex.search(searchQuery, {
        limit: 15,
        enrich: true,
      });

      // Merge and score results
      const scoreMap = new Map<number, { doc: SearchDoc; score: number }>();

      // Weight different fields
      const fieldWeights: Record<string, number> = {
        title: 10,
        headings: 5,
        description: 3,
        content: 1,
      };

      for (const fieldResult of searchResults) {
        const field = fieldResult.field as string;
        const weight = fieldWeights[field] || 1;

        for (const result of fieldResult.result) {
          const doc = result.doc as SearchDoc;
          const existing = scoreMap.get(doc.id);
          const newScore = (existing?.score || 0) + weight;
          scoreMap.set(doc.id, { doc, score: newScore });
        }
      }

      // Sort by score and create results with highlights
      const sortedResults = Array.from(scoreMap.values())
        .sort((a, b) => b.score - a.score)
        .slice(0, 10)
        .map(({ doc, score }): SearchResult => ({
          ...doc,
          score,
          highlights: {
            title: highlightText(doc.title, searchQuery),
            description: highlightText(doc.description, searchQuery),
            content: highlightText(getContentSnippet(doc.content, searchQuery), searchQuery),
          },
        }));

      setResults(sortedResults);
      setSelectedIndex(0);
    } catch (error) {
      console.error("Search error:", error);
      setResults([]);
    } finally {
      setLoading(false);
    }
  }, [indexReady]);

  // Debounce search
  useEffect(() => {
    const timer = setTimeout(() => {
      performSearch(query);
    }, 150);

    return () => clearTimeout(timer);
  }, [query, performSearch]);

  const handleResultClick = () => {
    onClose();
    setQuery("");
    setResults([]);
  };

  const getCategoryColor = () => {
    return 'text-[var(--text-muted)]';
  };

  if (!isOpen) return null;

  return (
    <div
      className="fixed inset-0 z-[100] flex items-start justify-center bg-black/60 backdrop-blur-sm pt-[10vh]"
      onClick={onClose}
    >
      <div
        className="w-full max-w-2xl mx-4 bg-[var(--bg)] border border-[var(--border)] overflow-hidden"
        onClick={(e) => e.stopPropagation()}
      >
        {/* Search Input */}
        <div className="flex items-center gap-3 px-4 py-4 border-b border-[var(--border)]">
          <FiSearch className="w-5 h-5 text-[var(--text-muted)]" />
          <input
            ref={inputRef}
            type="text"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="Search documentation..."
            className="flex-1 bg-transparent outline-none text-[var(--text)] placeholder:text-[var(--text-tertiary)] text-lg"
            autoComplete="off"
            autoCorrect="off"
            spellCheck="false"
          />
          {query && (
            <button
              onClick={() => setQuery("")}
              className="p-1 hover:bg-[var(--surface)] rounded transition-colors"
              aria-label="Clear search"
            >
              <FiX className="w-4 h-4 text-[var(--text-secondary)]" />
            </button>
          )}
        </div>

        {/* Results */}
        <div ref={resultsRef} className="max-h-[60vh] overflow-y-auto">
          {!indexReady && (
            <div className="p-8 text-center text-[var(--text-secondary)]">
              <div className="animate-pulse">Loading search index...</div>
            </div>
          )}

          {indexReady && loading && (
            <div className="p-8 text-center text-[var(--text-secondary)]">
              <div className="animate-pulse">Searching...</div>
            </div>
          )}

          {indexReady && !loading && query && results.length === 0 && (
            <div className="p-8 text-center">
              <div className="text-[var(--text-secondary)] mb-2">No results found for "{query}"</div>
              <div className="text-sm text-[var(--text-tertiary)]">
                Try different keywords or check your spelling
              </div>
            </div>
          )}

          {indexReady && !loading && results.length > 0 && (
            <div className="py-2">
              {results.map((result, index) => (
                <Link
                  key={result.id}
                  href={result.slug}
                  onClick={handleResultClick}
                  className={`block px-4 py-3 transition-colors border-l-2 ${
                    index === selectedIndex
                      ? 'bg-[var(--accent)]/10 border-l-[var(--accent)]'
                      : 'border-l-transparent hover:bg-[var(--surface)]'
                  }`}
                  onMouseEnter={() => setSelectedIndex(index)}
                >
                  <div className="flex items-start gap-3">
                    <FiFileText className="w-4 h-4 mt-1 text-[var(--text-tertiary)] flex-shrink-0" />
                    <div className="flex-1 min-w-0">
                      <div className="flex items-center gap-2 mb-1">
                        <span
                          className="font-medium text-[var(--text)]"
                          dangerouslySetInnerHTML={{ __html: result.highlights.title || result.title }}
                        />
                        <span className={`text-xs uppercase tracking-wide ${getCategoryColor()}`}>
                          {result.category.replace('-', ' ')}
                        </span>
                      </div>
                      {result.highlights.description && (
                        <div
                          className="text-sm text-[var(--text-secondary)] line-clamp-1 mb-1"
                          dangerouslySetInnerHTML={{ __html: result.highlights.description }}
                        />
                      )}
                      {result.highlights.content && (
                        <div
                          className="text-xs text-[var(--text-tertiary)] line-clamp-2"
                          dangerouslySetInnerHTML={{ __html: result.highlights.content }}
                        />
                      )}
                    </div>
                    <FiArrowRight className={`w-4 h-4 mt-1 transition-opacity ${
                      index === selectedIndex ? 'opacity-100 text-[var(--accent)]' : 'opacity-0'
                    }`} />
                  </div>
                </Link>
              ))}
            </div>
          )}

          {indexReady && !loading && !query && (
            <div className="p-8 text-center">
              <div className="text-[var(--text-secondary)] mb-3">Start typing to search...</div>
              <div className="flex flex-wrap justify-center gap-2">
                {['node', 'scheduler', 'ipc', 'python', 'simulation'].map(term => (
                  <button
                    key={term}
                    onClick={() => setQuery(term)}
                    className="px-3 py-1 text-sm bg-[var(--surface)] border border-[var(--border)] hover:border-[var(--border-hover)] hover:text-[var(--text)] transition-colors"
                  >
                    {term}
                  </button>
                ))}
              </div>
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="px-4 py-2 border-t border-[var(--border)] flex items-center justify-between text-xs text-[var(--text-tertiary)] bg-[var(--surface)]/50">
          <div className="flex items-center gap-4">
            <span className="flex items-center gap-1">
              <kbd className="px-1.5 py-0.5 bg-[var(--surface)] border border-[var(--border)] rounded text-[10px]">↑</kbd>
              <kbd className="px-1.5 py-0.5 bg-[var(--surface)] border border-[var(--border)] rounded text-[10px]">↓</kbd>
              <span className="ml-1">Navigate</span>
            </span>
            <span className="flex items-center gap-1">
              <kbd className="px-1.5 py-0.5 bg-[var(--surface)] border border-[var(--border)] rounded text-[10px]">↵</kbd>
              <span className="ml-1">Select</span>
            </span>
            <span className="flex items-center gap-1">
              <kbd className="px-1.5 py-0.5 bg-[var(--surface)] border border-[var(--border)] rounded text-[10px]">ESC</kbd>
              <span className="ml-1">Close</span>
            </span>
          </div>
          <div>
            {results.length > 0 && (
              <span className="text-[var(--accent)]">{results.length} result{results.length === 1 ? '' : 's'}</span>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
