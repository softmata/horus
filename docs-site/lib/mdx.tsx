/**
 * MDX compilation and rendering for HORUS documentation
 *
 * IMPORTANT for AI assistants and contributors:
 * Before editing .mdx files, read MDX_GUIDELINES.md to avoid common rendering errors!
 *
 * Common pitfalls:
 * - Using `<` in text (e.g., "<1%" should be "&lt;1%")
 * - Generic types without backticks (e.g., "Hub<T>" should be "`Hub<T>`")
 * - Headings starting with numbers (auto-fixed below but still discouraged)
 */

import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';
import { compileMDX } from 'next-mdx-remote/rsc';
import remarkGfm from 'remark-gfm';
import CodeBlock from '@/components/CodeBlock';
import {
  LatencyComparisonChart,
  LatencyScalingChart,
  ScalabilityChart,
  SpeedupChart,
  ThroughputChart,
  RealTimeChart,
} from '@/components/BenchmarkCharts';
import MermaidDiagram from '@/components/MermaidDiagram';

const contentDirectory = path.join(process.cwd(), 'content');

export interface DocFrontmatter {
  title: string;
  description?: string;
  section?: string;
  order?: number;
}

export interface DocContent {
  slug: string;
  frontmatter: DocFrontmatter;
  content: React.ReactElement;
}

/**
 * Get all MDX files from a directory
 */
export async function getDocSlugs(dir: string): Promise<string[]> {
  const fullPath = path.join(contentDirectory, dir);

  if (!fs.existsSync(fullPath)) {
    return [];
  }

  const files = fs.readdirSync(fullPath);
  return files
    .filter(file => file.endsWith('.mdx'))
    .map(file => file.replace(/\.mdx$/, ''));
}

/**
 * Get a single MDX document by slug
 */
export async function getDoc(slug: string[]): Promise<DocContent | null> {
  try {
    // Try the direct file path first
    let filePath = path.join(contentDirectory, ...slug) + '.mdx';

    // If that doesn't exist, try looking for an index.mdx file in a directory
    if (!fs.existsSync(filePath)) {
      const indexPath = path.join(contentDirectory, ...slug, 'index.mdx');
      if (fs.existsSync(indexPath)) {
        filePath = indexPath;
      } else {
        return null;
      }
    }

    const source = fs.readFileSync(filePath, 'utf-8');
    const { data, content: mdxContent } = matter(source);

    const { content } = await compileMDX<DocFrontmatter>({
      source: mdxContent,
      options: {
        parseFrontmatter: true,
        mdxOptions: {
          remarkPlugins: [remarkGfm],
          rehypePlugins: [],
        },
      },
      components: {
        // Benchmark charts
        LatencyComparisonChart,
        LatencyScalingChart,
        ScalabilityChart,
        SpeedupChart,
        ThroughputChart,
        RealTimeChart,
        // Diagrams
        MermaidDiagram,
        h2: ({ children, ...props }: any) => {
          let id = typeof children === 'string'
            ? children.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/(^-|-$)/g, '')
            : '';
          // Prefix IDs that start with a number (invalid HTML IDs)
          if (id && /^[0-9]/.test(id)) {
            id = 'section-' + id;
          }
          return (
            <h2 id={id} {...props}>
              {children}
            </h2>
          );
        },
        h3: ({ children, ...props }: any) => {
          let id = typeof children === 'string'
            ? children.toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/(^-|-$)/g, '')
            : '';
          // Prefix IDs that start with a number (invalid HTML IDs)
          if (id && /^[0-9]/.test(id)) {
            id = 'section-' + id;
          }
          return (
            <h3 id={id} {...props}>
              {children}
            </h3>
          );
        },
        pre: ({ children, ...props }: any) => {
          const codeElement = children?.props;
          const className = codeElement?.className || '';
          const code = codeElement?.children?.toString() || '';

          // If we have a code block with language, use our CodeBlock component
          if (className) {
            return <CodeBlock className={className}>{code}</CodeBlock>;
          }

          // Otherwise render plain pre
          return (
            <pre
              className="code-block"
              style={{
                fontFamily: '"Courier New", Courier, monospace',
                fontVariantLigatures: 'none',
                fontFeatureSettings: '"liga" 0, "calt" 0',
                letterSpacing: '0',
                textRendering: 'optimizeSpeed',
                WebkitFontSmoothing: 'none',
                fontWeight: 'normal',
                backgroundColor: '#1e1e1e',
                padding: '1rem',
                borderRadius: '0.375rem',
                overflow: 'auto',
                marginBottom: '1.5rem',
                color: '#d4d4d4',
              }}
              {...props}
            >
              {children}
            </pre>
          );
        },
        code: ({ children, className, ...props }: any) => {
          // Inline code styling
          if (!className) {
            return (
              <code
                className="px-1.5 py-0.5 rounded bg-gray-800 text-cyan-400 text-sm"
                style={{
                  fontVariantLigatures: 'none',
                  fontFeatureSettings: '"liga" 0, "calt" 0',
                }}
                {...props}
              >
                {children}
              </code>
            );
          }

          // Code block (language specified) - syntax highlighted
          return (
            <code
              className={className}
              style={{
                fontVariantLigatures: 'none',
                fontFeatureSettings: '"liga" 0, "calt" 0',
                color: '#d4d4d4', // Light gray text
                display: 'block',
              }}
              {...props}
            >
              {children}
            </code>
          );
        },
        table: ({ children, ...props }: any) => (
          <div className="overflow-x-auto my-6">
            <table className="min-w-full border-collapse border border-gray-700" {...props}>
              {children}
            </table>
          </div>
        ),
        thead: ({ children, ...props }: any) => (
          <thead className="bg-[var(--surface)]" {...props}>
            {children}
          </thead>
        ),
        tbody: ({ children, ...props }: any) => (
          <tbody {...props}>
            {children}
          </tbody>
        ),
        tr: ({ children, ...props }: any) => (
          <tr className="border-b border-gray-700" {...props}>
            {children}
          </tr>
        ),
        th: ({ children, ...props }: any) => (
          <th className="px-4 py-2 text-left font-semibold text-[var(--accent)]" {...props}>
            {children}
          </th>
        ),
        td: ({ children, ...props }: any) => (
          <td className="px-4 py-2 text-gray-300" {...props}>
            {children}
          </td>
        ),
      },
    });

    return {
      slug: slug.join('/'),
      frontmatter: data as DocFrontmatter,
      content,
    };
  } catch (error) {
    console.error('Error loading doc:', error);
    return null;
  }
}

/**
 * Get all documents in a section with their metadata
 */
export async function getDocsList(section: string): Promise<Array<{ slug: string; frontmatter: DocFrontmatter }>> {
  const slugs = await getDocSlugs(section);
  const docs = await Promise.all(
    slugs.map(async (slug) => {
      const doc = await getDoc([section, slug]);
      return doc ? { slug, frontmatter: doc.frontmatter } : null;
    })
  );

  return docs
    .filter((doc): doc is { slug: string; frontmatter: DocFrontmatter } => doc !== null)
    .sort((a, b) => (a.frontmatter.order || 999) - (b.frontmatter.order || 999));
}
