import type { Metadata, Viewport } from "next";
import "./globals.css";
import { Analytics } from "@vercel/analytics/react";

export const metadata: Metadata = {
  metadataBase: new URL('https://docs.horus-registry.dev'),
  title: "HORUS Documentation | Real-Time Robotics Framework",
  description: "Documentation for HORUS, a high-performance robotics framework. Sub-microsecond IPC latency, zero-copy messaging, multi-language support (Rust/Python). Open source under Apache 2.0.",
  keywords: [
    'horus robotics',
    'horus framework',
    'robotics framework',
    'real-time robotics',
    'rust robotics',
    'python robotics',
    'robot control',
    'IPC',
    'shared memory',
    'pub sub',
    'ROS alternative',
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
    title: "HORUS Documentation | Real-Time Robotics Framework",
    description: "Documentation for HORUS, a high-performance robotics framework with sub-microsecond IPC latency, zero-copy messaging, and multi-language support.",
    url: "https://docs.horus-registry.dev",
    siteName: "HORUS Documentation",
    images: [
      {
        url: 'https://docs.horus-registry.dev/og-image.png',
        width: 1200,
        height: 630,
        alt: 'HORUS - Real-Time Robotics Framework Documentation',
      },
    ],
    locale: 'en_US',
    type: 'website',
  },
  twitter: {
    card: 'summary_large_image',
    title: "HORUS Documentation | Real-Time Robotics Framework",
    description: "Documentation for HORUS, a high-performance robotics framework with sub-microsecond IPC latency and zero-copy messaging.",
    images: ['https://docs.horus-registry.dev/og-image.png'],
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
    name: 'HORUS',
    alternateName: ['HORUS Robotics Framework'],
    applicationCategory: 'DeveloperApplication',
    applicationSubCategory: 'Robotics Framework',
    operatingSystem: ['Linux', 'macOS', 'Windows'],
    description: 'HORUS is an open-source robotics framework with sub-microsecond IPC latency, zero-copy shared memory messaging, and multi-language support (Rust, Python).',
    softwareVersion: '0.1.7',
    url: 'https://docs.horus-registry.dev',
    downloadUrl: 'https://github.com/softmata/HORUS',
    installUrl: 'https://docs.horus-registry.dev/getting-started/installation',
    softwareHelp: 'https://docs.horus-registry.dev',
    releaseNotes: 'https://github.com/softmata/HORUS/releases',
    keywords: 'robotics framework, real-time, rust, python, IPC, shared memory',
    programmingLanguage: ['Rust', 'Python'],
    license: 'https://opensource.org/licenses/Apache-2.0',
    creator: {
      '@type': 'Organization',
      name: 'HORUS Contributors',
      url: 'https://github.com/softmata/HORUS',
    },
    offers: {
      '@type': 'Offer',
      price: '0',
      priceCurrency: 'USD',
      availability: 'https://schema.org/InStock',
    },
    featureList: [
      'Sub-microsecond IPC latency',
      'Zero-copy shared memory architecture',
      'Deterministic real-time control',
      'Multi-language support (Rust, Python)',
      'Native hardware integration',
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
        <Analytics />
      </body>
    </html>
  );
}
