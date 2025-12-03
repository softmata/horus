'use client';

import { useEffect, useRef, useState } from 'react';

interface CodeBlockProps {
  children: string;
  className?: string;
}

export default function CodeBlock({ children, className = '' }: CodeBlockProps) {
  const codeRef = useRef<HTMLElement>(null);
  const [highlighted, setHighlighted] = useState(false);
  const [theme, setTheme] = useState<'light' | 'dark'>('dark');
  const [copied, setCopied] = useState(false);
  const language = className.replace(/language-/, '') || 'text';

  const handleCopy = async () => {
    try {
      await navigator.clipboard.writeText(children);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch (err) {
      console.error('Failed to copy:', err);
    }
  };

  // Detect theme changes
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const detectTheme = () => {
      const currentTheme = document.documentElement.getAttribute('data-theme');
      setTheme(currentTheme === 'light' ? 'light' : 'dark');
    };

    // Initial detection
    detectTheme();

    // Watch for theme changes
    const observer = new MutationObserver(detectTheme);
    observer.observe(document.documentElement, {
      attributes: true,
      attributeFilter: ['data-theme']
    });

    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    // Only run on client side
    if (typeof window === 'undefined') return;

    // Dynamically import Prism and highlight
    const highlightCode = async () => {
      try {
        const Prism = (await import('prismjs')).default;

        // Load language-specific grammars
        if (language === 'rust') await import('prismjs/components/prism-rust' as any);
        else if (language === 'bash' || language === 'shell' || language === 'sh') await import('prismjs/components/prism-bash' as any);
        else if (language === 'python' || language === 'py') await import('prismjs/components/prism-python' as any);
        else if (language === 'cpp' || language === 'c++') await import('prismjs/components/prism-cpp' as any);
        else if (language === 'typescript' || language === 'ts') await import('prismjs/components/prism-typescript' as any);
        else if (language === 'javascript' || language === 'js') await import('prismjs/components/prism-javascript' as any);
        else if (language === 'json') await import('prismjs/components/prism-json' as any);
        else if (language === 'yaml' || language === 'yml') await import('prismjs/components/prism-yaml' as any);
        else if (language === 'toml') await import('prismjs/components/prism-toml' as any);
        else if (language === 'markdown' || language === 'md') await import('prismjs/components/prism-markdown' as any);

        if (codeRef.current) {
          // Use textContent to preserve angle brackets (Hub<f32>, Vec<T>, etc.)
          // This prevents <f32> from being parsed as an HTML tag
          codeRef.current.textContent = children;

          // Re-highlight with Prism
          Prism.highlightElement(codeRef.current);
          setHighlighted(true);
        }
      } catch (error) {
        console.warn('Failed to load syntax highlighting:', error);
      }
    };

    highlightCode();
  }, [children, language, theme]);

  const preStyle: React.CSSProperties = theme === 'light' ? {
    fontFamily: '"Courier New", Courier, monospace',
    fontVariantLigatures: 'none' as const,
    fontFeatureSettings: '"liga" 0, "calt" 0',
    letterSpacing: '0',
    textRendering: 'optimizeSpeed' as const,
    WebkitFontSmoothing: 'antialiased' as const,
    fontWeight: 'normal',
    backgroundColor: '#f8fafc',
    padding: '1rem',
    borderRadius: '0.375rem',
    overflow: 'auto',
    marginBottom: '1.5rem',
  } : {
    fontFamily: '"Courier New", Courier, monospace',
    fontVariantLigatures: 'none' as const,
    fontFeatureSettings: '"liga" 0, "calt" 0',
    letterSpacing: '0',
    textRendering: 'optimizeSpeed' as const,
    WebkitFontSmoothing: 'none' as const,
    fontWeight: 'normal',
    backgroundColor: '#1e1e1e',
    padding: '1rem',
    borderRadius: '0.375rem',
    overflow: 'auto',
    marginBottom: '1.5rem',
  };

  return (
    <div style={{ position: 'relative', marginBottom: '1.5rem' }}>
      <button
        onClick={handleCopy}
        style={{
          position: 'absolute',
          top: '0.5rem',
          right: '0.5rem',
          padding: '0.5rem 0.75rem',
          background: theme === 'light' ? '#E2E8F0' : '#1F2229',
          border: `1px solid ${theme === 'light' ? 'rgba(100, 116, 139, 0.3)' : 'rgba(0, 212, 255, 0.2)'}`,
          borderRadius: '4px',
          color: theme === 'light' ? '#0F172A' : '#E2E8F0',
          fontSize: '0.75rem',
          fontWeight: '500',
          cursor: 'pointer',
          transition: 'all 0.2s',
          fontFamily: 'JetBrains Mono, monospace',
          zIndex: 10,
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.background = theme === 'light' ? '#CBD5E1' : '#2D3748';
          e.currentTarget.style.borderColor = theme === 'light' ? '#0284C7' : '#00D4FF';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.background = theme === 'light' ? '#E2E8F0' : '#1F2229';
          e.currentTarget.style.borderColor = theme === 'light' ? 'rgba(100, 116, 139, 0.3)' : 'rgba(0, 212, 255, 0.2)';
        }}
      >
        {copied ? ' Copied!' : 'Copy'}
      </button>
      <pre className={`code-block ${className}`} style={preStyle}>
        <code
          ref={codeRef}
          className={className}
          style={{
            fontVariantLigatures: 'none',
            fontFeatureSettings: '"liga" 0, "calt" 0',
          }}
        >
          {children}
        </code>
      </pre>
    </div>
  );
}
