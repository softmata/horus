import { getDoc } from '@/lib/mdx';
import { DocsLayout } from '@/components/DocsLayout';
import { TableOfContents } from '@/components/TableOfContents';
import { Breadcrumb } from '@/components/Breadcrumb';
import { notFound } from 'next/navigation';
import type { Metadata } from 'next';

interface PageProps {
  params: {
    slug: string[];
  };
}

export async function generateMetadata({ params }: PageProps): Promise<Metadata> {
  const { slug } = params;
  const docPath = ['docs', ...slug];
  const doc = await getDoc(docPath);

  if (!doc) {
    return {
      title: 'Page Not Found | HORUS - World\'s Fastest Robotics Framework',
      description: 'The requested page could not be found. Explore HORUS documentation to build revolutionary robots 575x faster than ROS2.',
    };
  }

  const baseTitle = doc.frontmatter.title || 'HORUS Documentation';
  const title = `${baseTitle} | HORUS - 575x Faster Than ROS2`;
  const description = doc.frontmatter.description || 'Learn to build production robots with HORUS - the world\'s fastest robotics framework. 87ns latency, 575x faster than ROS2. Rust & Python. FREE & open source.';
  const url = `https://docs.horus-registry.dev/${slug.join('/')}`;

  return {
    title,
    description,
    keywords: [
      // Primary keywords
      'HORUS',
      'fastest robotics framework',
      '575x faster than ROS2',
      '87ns latency',
      'revolutionary robotics',

      // Technical keywords
      'real-time robotics',
      'zero-copy IPC',
      'shared memory robotics',
      'Rust robotics framework',
      'Python robotics',

      // Use case keywords
      'autonomous robot',
      'humanoid robot',
      'drone control',
      'industrial automation',

      // Comparison keywords
      'ROS alternative',
      'ROS2 alternative',
      'best robotics framework',
      'modern robotics',

      // Intent keywords
      'learn robotics',
      'robot programming tutorial',
      'build robots fast',
    ],
    authors: [{ name: 'HORUS Robotics Team' }],
    creator: 'HORUS Robotics',
    publisher: 'HORUS Robotics',
    openGraph: {
      title: `${baseTitle} | HORUS - Revolutionary Robotics Framework`,
      description: `${description} Build your first robot in 5 minutes.`,
      url,
      siteName: 'HORUS - World\'s Fastest Robotics Framework',
      type: 'article',
      locale: 'en_US',
      images: [
        {
          url: 'https://docs.horus-registry.dev/og-image.png',
          width: 1200,
          height: 630,
          alt: `${baseTitle} - HORUS Documentation | 575x Faster Than ROS2`,
        },
      ],
    },
    twitter: {
      card: 'summary_large_image',
      title: `${baseTitle} | HORUS - 575x Faster`,
      description: `${description.substring(0, 200)}...`,
      images: ['https://docs.horus-registry.dev/og-image.png'],
      creator: '@horus_robotics',
      site: '@horus_robotics',
    },
    alternates: {
      canonical: url,
    },
    robots: {
      index: true,
      follow: true,
      googleBot: {
        index: true,
        follow: true,
        'max-video-preview': -1,
        'max-image-preview': 'large',
        'max-snippet': -1,
      },
    },
  };
}

export default async function DocPage({ params }: PageProps) {
  const { slug } = params;

  // Always prepend 'docs' to the path
  const docPath = ['docs', ...slug];

  const doc = await getDoc(docPath);

  if (!doc) {
    notFound();
  }

  return (
    <DocsLayout>
      <main className="flex-1 w-full max-w-4xl mx-auto px-4 sm:px-6 lg:px-8 py-8 sm:py-12">
        <Breadcrumb />
        <article className="prose prose-invert max-w-none prose-headings:scroll-mt-20 prose-p:text-[var(--text-secondary)] prose-p:leading-relaxed prose-li:text-[var(--text-secondary)]">
          {doc.content}
        </article>
      </main>
      <TableOfContents />
    </DocsLayout>
  );
}

export async function generateStaticParams() {
  const fs = require('fs');
  const path = require('path');

  const contentDir = path.join(process.cwd(), 'content/docs');
  const routes: { slug: string[] }[] = [];

  // Recursively find all .mdx files
  function findMdxFiles(dir: string, basePath: string[] = []): void {
    const files = fs.readdirSync(dir);

    for (const file of files) {
      const filePath = path.join(dir, file);
      const stat = fs.statSync(filePath);

      if (stat.isDirectory()) {
        // Recurse into subdirectory
        findMdxFiles(filePath, [...basePath, file]);
      } else if (file.endsWith('.mdx')) {
        // Add route for this MDX file
        const fileName = file.replace(/\.mdx$/, '');

        // For index.mdx files, use the directory path without 'index'
        if (fileName === 'index') {
          // Only add if basePath is not empty (we don't want a route for root index.mdx)
          if (basePath.length > 0) {
            routes.push({ slug: basePath });
          }
        } else {
          routes.push({ slug: [...basePath, fileName] });
        }
      }
    }
  }

  findMdxFiles(contentDir);

  return routes;
}
