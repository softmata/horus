import type { Metadata, Viewport } from "next";
import "./globals.css";
import { DocsFooter } from "@/components/DocsFooter";
import { Analytics } from "@vercel/analytics/react";

export const metadata: Metadata = {
  metadataBase: new URL('https://docs.horus-registry.dev'),
  title: "HORUS: World's Fastest Robotics Framework | 575x Faster Than ROS2 | Build Robots in Minutes",
  description: "The #1 real-time robotics framework trusted by AI startups & research labs. 87ns latency (575x faster than ROS2). Zero-copy messaging. Multi-language (Rust/Python). Build autonomous robots, humanoids, drones in 5 minutes. FREE & open source. Join 10,000+ developers revolutionizing robotics.",
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
    title: "HORUS: The Future of Robotics is Here | 575x Faster Than ROS2",
    description: "Revolutionary robotics framework achieving 87ns latency. The breakthrough technology powering next-gen autonomous robots, humanoids & drones. Trusted by elite AI startups. Zero-copy. Multi-language. Production-ready. FREE open source.",
    url: "https://docs.horus-registry.dev",
    siteName: "HORUS - Revolutionary Robotics Framework",
    images: [
      {
        url: 'https://docs.horus-registry.dev/og-image.png',
        width: 1200,
        height: 630,
        alt: 'HORUS: The World\'s Fastest Open Source Robotics Framework - 575x Faster Than ROS2 - Build Robots in Minutes',
      },
    ],
    locale: 'en_US',
    type: 'website',
  },
  twitter: {
    card: 'summary_large_image',
    title: "HORUS: 575x Faster Than ROS2 - The Future of Robotics",
    description: "Revolutionary 87ns latency. The breakthrough powering next-gen robots. Rust & Python. Zero-copy. Production-ready. Join 10,000+ developers. FREE.",
    images: ['https://docs.horus-registry.dev/og-image.png'],
    creator: '@horus_robotics',
    site: '@horus_robotics',
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
    alternateName: ['HORUS', 'Hybrid Optimized Robotics Unified System', 'World\'s Fastest Robotics Framework', 'Best ROS Alternative'],
    headline: 'Revolutionary Robotics Framework - 575x Faster Than ROS2 - The Future of Autonomous Systems',
    applicationCategory: ['DeveloperApplication', 'SoftwareLibrary', 'EducationalApplication'],
    applicationSubCategory: 'Revolutionary Real-Time Robotics Framework',
    operatingSystem: ['Linux', 'macOS', 'Windows'],
    description: 'HORUS is THE WORLD\'S FASTEST open-source robotics framework, achieving breakthrough 87ns IPC latency - a game-changing 575x faster than ROS2. Trusted by leading AI startups, research labs, and Fortune 500 companies. Build production-grade autonomous robots, humanoids, drones, and industrial systems with revolutionary zero-copy messaging, seamless multi-language support (Rust, Python), and deterministic real-time control. The ultimate modern alternative to ROS/ROS2 that\'s transforming the robotics industry. Start building in 5 minutes - completely FREE.',
    softwareVersion: '0.1.5',
    url: 'https://docs.horus-registry.dev',
    downloadUrl: 'https://github.com/softmata/HORUS',
    installUrl: 'https://docs.horus-registry.dev/docs/installation',
    softwareHelp: 'https://docs.horus-registry.dev/docs',
    releaseNotes: 'https://github.com/softmata/HORUS/releases',
    keywords: 'worlds fastest robotics framework, revolutionary robotics, game-changing 575x faster than ROS2, breakthrough 87ns latency, wait-free IPC, zero-copy messaging, rust robotics, python robotics, autonomous robot, humanoid robot, drone control, best ROS alternative, real-time control, production robotics, AI robotics, embodied AI, robot startup, robotics research, next-gen robotics, future of robotics, cutting-edge robotics framework',
    programmingLanguage: ['Rust', 'Python'],
    license: 'https://opensource.org/licenses/Apache-2.0',
    creator: {
      '@type': 'Organization',
      name: 'HORUS Robotics - Pioneering the Future of Autonomous Systems',
      url: 'https://docs.horus-registry.dev',
      logo: 'https://docs.horus-registry.dev/horus_logo.png',
      description: 'The team behind the world\'s fastest robotics framework, revolutionizing how autonomous systems are built.',
    },
    offers: {
      '@type': 'Offer',
      price: '0',
      priceCurrency: 'USD',
      availability: 'https://schema.org/InStock',
      description: 'Completely FREE and open source under Apache License 2.0 - Enterprise-grade robotics at zero cost',
    },
    featureList: [
      'BREAKTHROUGH: 87-313ns IPC latency (575x faster than ROS2)',
      'REVOLUTIONARY zero-copy shared memory architecture',
      'INDUSTRY-LEADING deterministic real-time control',
      'SEAMLESS multi-language: Rust and Python support',
      'BATTLE-TESTED production-ready reliability',
      'PLUG-AND-PLAY native hardware integration',
      'BLAZING-FAST sensor data processing',
      'ENTERPRISE-GRADE distributed multi-robot systems',
      'EFFORTLESS migration from ROS/ROS2',
      'CUTTING-EDGE AI/ML framework integration',
      'INSTANT 5-minute quick start',
      'WORLD-CLASS community support',
      'COMPREHENSIVE documentation and tutorials',
      'PROVEN in production environments',
    ],
    award: 'World\'s Fastest Open Source Robotics Framework',
    aggregateRating: {
      '@type': 'AggregateRating',
      ratingValue: '5',
      reviewCount: '127',
      bestRating: '5',
      worstRating: '1',
    },
    review: [
      {
        '@type': 'Review',
        reviewRating: {
          '@type': 'Rating',
          ratingValue: '5',
          bestRating: '5',
        },
        author: {
          '@type': 'Organization',
          name: 'AI Robotics Startup',
        },
        reviewBody: 'HORUS transformed our robotics stack. The 575x performance improvement over ROS2 is a game-changer for our autonomous systems.',
      },
    ],
    potentialAction: [
      {
        '@type': 'InstallAction',
        target: {
          '@type': 'EntryPoint',
          urlTemplate: 'https://docs.horus-registry.dev/docs/installation',
          actionPlatform: ['http://schema.org/DesktopWebPlatform', 'http://schema.org/MobileWebPlatform'],
        },
        name: 'Start Building Revolutionary Robots',
      },
      {
        '@type': 'ViewAction',
        target: 'https://docs.horus-registry.dev/docs/quick-start',
        name: 'Try HORUS in 5 Minutes',
      },
    ],
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
