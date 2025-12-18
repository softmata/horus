'use client';

import { useEffect, useRef, useState, useCallback } from 'react';
import mermaid from 'mermaid';

// HORUS brand colors
const HORUS_COLORS = {
  primary: '#06b6d4',      // Cyan
  secondary: '#8b5cf6',    // Purple
  accent: '#10b981',       // Emerald
  warning: '#f59e0b',      // Amber
  error: '#ef4444',        // Red
};

// Theme configurations
const darkThemeConfig = {
  theme: 'base' as const,
  themeVariables: {
    // Background colors
    primaryColor: '#1f2937',
    primaryBorderColor: HORUS_COLORS.primary,
    primaryTextColor: '#f5f5f5',

    // Secondary
    secondaryColor: '#374151',
    secondaryBorderColor: '#4b5563',
    secondaryTextColor: '#e5e7eb',

    // Tertiary
    tertiaryColor: '#1e3a5f',
    tertiaryBorderColor: HORUS_COLORS.primary,
    tertiaryTextColor: '#f5f5f5',

    // Lines and text
    lineColor: '#6b7280',
    textColor: '#f5f5f5',

    // Notes
    noteBkgColor: '#374151',
    noteTextColor: '#f5f5f5',
    noteBorderColor: '#4b5563',

    // Flowchart specific
    nodeBorder: HORUS_COLORS.primary,
    clusterBkg: '#111827',
    clusterBorder: '#374151',
    defaultLinkColor: '#9ca3af',
    titleColor: '#f5f5f5',
    edgeLabelBackground: '#1f2937',

    // Actor (sequence diagrams)
    actorBkg: '#1f2937',
    actorBorder: HORUS_COLORS.primary,
    actorTextColor: '#f5f5f5',
    actorLineColor: '#6b7280',

    // Signals
    signalColor: '#9ca3af',
    signalTextColor: '#f5f5f5',

    // Labels
    labelBoxBkgColor: '#1f2937',
    labelBoxBorderColor: '#374151',
    labelTextColor: '#f5f5f5',

    // Loop
    loopTextColor: '#f5f5f5',

    // Fonts
    fontFamily: 'system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
  },
};

const lightThemeConfig = {
  theme: 'base' as const,
  themeVariables: {
    // Background colors
    primaryColor: '#f3f4f6',
    primaryBorderColor: '#0891b2',
    primaryTextColor: '#1a1a1a',

    // Secondary
    secondaryColor: '#e5e7eb',
    secondaryBorderColor: '#d1d5db',
    secondaryTextColor: '#374151',

    // Tertiary
    tertiaryColor: '#e0f2fe',
    tertiaryBorderColor: '#0891b2',
    tertiaryTextColor: '#1a1a1a',

    // Lines and text
    lineColor: '#6b7280',
    textColor: '#1a1a1a',

    // Notes
    noteBkgColor: '#fef3c7',
    noteTextColor: '#1a1a1a',
    noteBorderColor: '#d97706',

    // Flowchart specific
    nodeBorder: '#0891b2',
    clusterBkg: '#f9fafb',
    clusterBorder: '#d1d5db',
    defaultLinkColor: '#6b7280',
    titleColor: '#1a1a1a',
    edgeLabelBackground: '#f3f4f6',

    // Actor (sequence diagrams)
    actorBkg: '#f3f4f6',
    actorBorder: '#0891b2',
    actorTextColor: '#1a1a1a',
    actorLineColor: '#6b7280',

    // Signals
    signalColor: '#6b7280',
    signalTextColor: '#1a1a1a',

    // Labels
    labelBoxBkgColor: '#f3f4f6',
    labelBoxBorderColor: '#d1d5db',
    labelTextColor: '#1a1a1a',

    // Loop
    loopTextColor: '#1a1a1a',

    // Fonts
    fontFamily: 'system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
  },
};

interface MermaidDiagramProps {
  chart: string;
  caption?: string;
}

export default function MermaidDiagram({ chart, caption }: MermaidDiagramProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const [svg, setSvg] = useState<string>('');
  const [isDark, setIsDark] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const idRef = useRef(`mermaid-${Math.random().toString(36).substr(2, 9)}`);

  // Theme detection
  useEffect(() => {
    const checkTheme = () => {
      const theme = document.documentElement.getAttribute('data-theme');
      if (theme) {
        setIsDark(theme === 'dark');
      } else {
        setIsDark(window.matchMedia('(prefers-color-scheme: dark)').matches);
      }
    };

    checkTheme();

    // Watch for theme changes
    const observer = new MutationObserver(checkTheme);
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['data-theme'],
    });

    // Watch for system preference changes
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    mediaQuery.addEventListener('change', checkTheme);

    return () => {
      observer.disconnect();
      mediaQuery.removeEventListener('change', checkTheme);
    };
  }, []);

  // Render diagram
  const renderDiagram = useCallback(async () => {
    if (!chart) return;

    try {
      // Initialize mermaid with current theme
      mermaid.initialize({
        startOnLoad: false,
        securityLevel: 'loose',
        flowchart: {
          htmlLabels: true,
          padding: 20,
          nodeSpacing: 50,
          rankSpacing: 50,
          curve: 'basis',
          useMaxWidth: true,
        },
        ...(isDark ? darkThemeConfig : lightThemeConfig),
      });

      // Generate unique ID for this render
      const id = `${idRef.current}-${Date.now()}`;

      // Render the diagram
      const { svg: renderedSvg } = await mermaid.render(id, chart);
      setSvg(renderedSvg);
      setError(null);
    } catch (err) {
      console.error('Mermaid rendering error:', err);
      setError(err instanceof Error ? err.message : 'Failed to render diagram');
    }
  }, [chart, isDark]);

  useEffect(() => {
    renderDiagram();
  }, [renderDiagram]);

  if (error) {
    return (
      <div
        className="my-6 p-4 border rounded-lg"
        style={{
          backgroundColor: isDark ? '#1f2937' : '#fef2f2',
          borderColor: isDark ? '#991b1b' : '#fecaca',
          color: isDark ? '#fca5a5' : '#991b1b',
        }}
      >
        <p className="font-medium">Diagram Error</p>
        <pre className="mt-2 text-sm overflow-auto">{error}</pre>
        <details className="mt-2">
          <summary className="cursor-pointer text-sm opacity-70">Show source</summary>
          <pre className="mt-2 text-xs overflow-auto p-2 rounded" style={{
            backgroundColor: isDark ? '#111827' : '#f3f4f6',
          }}>
            {chart}
          </pre>
        </details>
      </div>
    );
  }

  return (
    <figure className="my-8">
      <div
        ref={containerRef}
        className="overflow-x-auto p-6 rounded-lg border flex justify-center"
        style={{
          backgroundColor: isDark ? '#111827' : '#f9fafb',
          borderColor: isDark ? '#374151' : '#e5e7eb',
        }}
      >
        <div
          className="mermaid-svg-container"
          style={{
            display: 'flex',
            justifyContent: 'center',
            width: '100%',
          }}
          dangerouslySetInnerHTML={{ __html: svg }}
        />
      </div>
      {caption && (
        <figcaption
          className="mt-2 text-center text-sm"
          style={{ color: isDark ? '#9ca3af' : '#6b7280' }}
        >
          {caption}
        </figcaption>
      )}
    </figure>
  );
}

// Pre-built diagram components for common patterns
export function ArchitectureDiagram({ chart, caption }: MermaidDiagramProps) {
  return <MermaidDiagram chart={chart} caption={caption} />;
}

export function FlowDiagram({ chart, caption }: MermaidDiagramProps) {
  return <MermaidDiagram chart={chart} caption={caption} />;
}

export function SequenceDiagram({ chart, caption }: MermaidDiagramProps) {
  return <MermaidDiagram chart={chart} caption={caption} />;
}
