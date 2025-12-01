import type { Metadata, Viewport } from "next";
import "./globals.css";
import { DocsFooter } from "@/components/DocsFooter";
import { Analytics } from "@vercel/analytics/react";

export const metadata: Metadata = {
  metadataBase: new URL('https://docs.horus-registry.dev'),
  title: "HORUS: Learn Robot Programming - Fastest Framework for Beginners | Rust, Python",
  description: "Learn to build real robots with HORUS - the easiest and fastest robotics framework. 5-minute setup, supports Python and Rust. Build autonomous robots, drones, humanoids. 500x faster than ROS2. Free tutorials for beginners. Start your first robot today.",
  keywords: [
    // BEGINNER/DISCOVERY KEYWORDS (capture people early!)
    'how to program a robot',
    'learn robot programming',
    'robot programming tutorial',
    'robotics programming for beginners',
    'how to build a robot',
    'learn robotics',
    'robot software tutorial',
    'robotics for beginners',
    'how to code a robot',
    'robot programming language',
    'best way to learn robotics',
    'robotics software for beginners',
    'start robotics programming',

    // EXPLORING OPTIONS (people looking for tools)
    'best robotics software',
    'what software to use for robotics',
    'robot operating system',
    'robotics programming software',
    'best robotics framework',
    'what is the best robotics framework',
    'robot development software',
    'software for building robots',

    // Brand + Unique Features
    'horus robotics',
    'horus framework',
    'horus tutorial',
    'fastest robotics framework',
    'easiest robotics framework',
    '500x faster than ros2',
    '87ns robotics',

    // Problem-focused (experienced users)
    'ROS alternative',
    'ROS2 alternative',
    'faster than ROS2',
    'easier than ROS',
    'ROS2 too complicated',
    'ROS2 too slow',
    'simple ROS alternative',

    // Language + framework combinations (high intent)
    'rust robotics framework',
    'python robotics framework',
    'fast robotics framework',
    'rust robot library',
    'rust real-time robotics',
    'rust embedded robotics',
    'python robot control',
    'python realtime robotics',

    // Specific use cases (long-tail, easier to rank)
    'build autonomous robot',
    'build humanoid robot',
    'robot control software',
    'drone control system',
    'robot arm control',
    'mobile robot framework',
    'quadruped robot software',
    'industrial robot control',
    'warehouse robot software',

    // Technical searches (developers)
    'zero copy messaging',
    'shared memory IPC',
    'real-time pub sub',
    'low latency IPC',
    'fast message passing',
    'real-time control system',
    'deterministic robotics',
    'hard real-time robotics',

    // Comparison searches (high commercial intent)
    'ros vs horus',
    'ros2 vs horus',
    'best robotics framework 2024',
    'modern robotics framework',
    'robotics framework comparison',
    'fastest robotics middleware',

    // Developer pain points
    'ros latency problems',
    'fast robot communication',
    'production robotics framework',
    'reliable robotics software',
    'easy robotics framework',
    'simple robot programming',

    // Academic/startup keywords
    'robotics startup framework',
    'research robotics platform',
    'robotics PhD tools',
    'robot prototyping framework',
    'rapid robot development',

    // Emerging/trending
    'ai robotics framework',
    'embodied ai platform',
    'physical ai system',
    'humanoid robot framework',
    'foundation model robotics',
    'llm robotics integration',

    // Integration keywords
    'ros to horus migration',
    'robotics framework rust',
    'multi language robotics',
    'cross platform robotics',
  ],
  icons: {
    icon: [
      { url: '/favicon.ico', sizes: '32x32' },
      { url: '/favicon-16x16.png', sizes: '16x16', type: 'image/png' },
      { url: '/favicon-32x32.png', sizes: '32x32', type: 'image/png' },
      { url: '/horus_logo.png', sizes: '192x192', type: 'image/png' },
    ],
    apple: [
      { url: '/apple-touch-icon.png', sizes: '180x180', type: 'image/png' },
    ],
  },
  openGraph: {
    title: "HORUS: 500x Faster Than ROS2 - Real-Time Robotics Framework",
    description: "Build production robots with 87ns latency (wait-free IPC). Zero-copy messaging, multi-language (Rust/Python), open source. Modern ROS alternative for autonomous robots, humanoids, drones. Start building in 5 minutes.",
    url: "https://docs.horus-registry.dev",
    siteName: "HORUS Robotics",
    images: [
      {
        url: 'https://docs.horus-registry.dev/og-image.png',
        width: 1200,
        height: 630,
        alt: 'HORUS: Fastest Open Source Robotics Framework - 500x Faster Than ROS2',
      },
    ],
    locale: 'en_US',
    type: 'website',
  },
  twitter: {
    card: 'summary_large_image',
    title: "HORUS: 500x Faster Robotics Framework",
    description: "Build real-time robots with 87ns latency. ROS alternative with Rust and Python. Zero-copy, production-ready, open source. Start in 5 min.",
    images: ['https://docs.horus-registry.dev/og-image.png'],
    creator: '@horus_robotics',
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
  alternates: {
    canonical: 'https://docs.horus-registry.dev',
  },
};

export const viewport: Viewport = {
  width: "device-width",
  initialScale: 1,
  maximumScale: 5,
  userScalable: true,
  themeColor: "#16181c",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  const jsonLd = {
    '@context': 'https://schema.org',
    '@type': 'SoftwareApplication',
    name: 'HORUS Robotics Framework',
    alternateName: ['HORUS', 'Hybrid Optimized Robotics Unified System'],
    headline: 'Fastest Open Source Robotics Framework - 500x Faster Than ROS2',
    applicationCategory: ['DeveloperApplication', 'SoftwareLibrary'],
    applicationSubCategory: 'Real-Time Robotics Framework',
    operatingSystem: ['Linux', 'macOS', 'Windows'],
    description: 'HORUS is the fastest open-source robotics framework with 87ns IPC latency (wait-free) - 575x faster than ROS2. Build production autonomous robots, humanoids, and drones with zero-copy messaging, multi-language support (Rust, Python), and deterministic real-time control. Modern alternative to ROS/ROS2 for AI robotics startups and research labs.',
    softwareVersion: '0.1.3',
    url: 'https://docs.horus-registry.dev',
    downloadUrl: 'https://github.com/softmata/HORUS',
    installUrl: 'https://docs.horus-registry.dev/docs/installation',
    softwareHelp: 'https://docs.horus-registry.dev/docs',
    keywords: 'fastest robotics framework, 575x faster than ROS2, sub-microsecond robotics, 87ns latency, wait-free IPC, zero-copy messaging, rust robotics, python robotics, autonomous robot, humanoid robot, drone control, ROS alternative, real-time control, production robotics, AI robotics, embodied AI, robot startup, robotics research',
    programmingLanguage: ['Rust', 'Python'],
    license: 'https://opensource.org/licenses/Apache-2.0',
    creator: {
      '@type': 'Organization',
      name: 'HORUS Robotics',
      url: 'https://docs.horus-registry.dev',
      logo: 'https://docs.horus-registry.dev/horus_logo.png',
    },
    offers: {
      '@type': 'Offer',
      price: '0',
      priceCurrency: 'USD',
      availability: 'https://schema.org/InStock',
      description: 'Free and open source under Apache License 2.0',
    },
    featureList: [
      '87-313ns IPC latency (575x faster than ROS2)',
      'Zero-copy shared memory architecture',
      'Deterministic real-time control',
      'Multi-language: Rust and Python support',
      'Production-ready reliability and stability',
      'Native hardware integration',
      'High-speed sensor data processing',
      'Distributed multi-robot systems',
      'Easy migration from ROS/ROS2',
      'Works with AI/ML frameworks',
      '5-minute quick start',
      'Active community support',
    ],
    aggregateRating: {
      '@type': 'AggregateRating',
      ratingValue: '5',
      reviewCount: '1',
      bestRating: '5',
      worstRating: '1',
    },
    potentialAction: {
      '@type': 'InstallAction',
      target: {
        '@type': 'EntryPoint',
        urlTemplate: 'https://docs.horus-registry.dev/docs/installation',
        actionPlatform: ['http://schema.org/DesktopWebPlatform', 'http://schema.org/MobileWebPlatform'],
      },
    },
  };

  return (
    <html lang="en">
      <head>
        <script
          type="application/ld+json"
          dangerouslySetInnerHTML={{ __html: JSON.stringify(jsonLd) }}
        />
      </head>
      <body className="font-mono antialiased">
        <main className="min-h-screen">
          {children}
        </main>
        <DocsFooter />
        <Analytics />
      </body>
    </html>
  );
}
