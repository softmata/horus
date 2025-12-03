import { MetadataRoute } from 'next';

export default function manifest(): MetadataRoute.Manifest {
  return {
    name: 'HORUS - World\'s Fastest Robotics Framework | 575x Faster Than ROS2',
    short_name: 'HORUS Robotics',
    description: 'Revolutionary real-time robotics framework with breakthrough 87ns latency. Build autonomous robots, humanoids, and drones 575x faster than ROS2. Trusted by elite AI startups. Multi-language (Rust/Python). FREE & open source.',
    start_url: '/',
    display: 'standalone',
    background_color: '#0a0e14',
    theme_color: '#00d4ff',
    orientation: 'portrait-primary',
    scope: '/',
    lang: 'en',
    dir: 'ltr',
    categories: ['developer tools', 'robotics', 'software', 'education', 'productivity'],
    icons: [
      {
        src: '/favicon-16x16.png',
        sizes: '16x16',
        type: 'image/png',
      },
      {
        src: '/favicon-32x32.png',
        sizes: '32x32',
        type: 'image/png',
      },
      {
        src: '/apple-touch-icon.png',
        sizes: '180x180',
        type: 'image/png',
      },
      {
        src: '/horus_logo.png',
        sizes: '192x192',
        type: 'image/png',
        purpose: 'any',
      },
      {
        src: '/horus_logo.png',
        sizes: '512x512',
        type: 'image/png',
        purpose: 'maskable',
      },
    ],
    screenshots: [
      {
        src: '/screenshots/dashboard.png',
        sizes: '1920x1080',
        type: 'image/png',
      },
      {
        src: '/screenshots/code-example.png',
        sizes: '1920x1080',
        type: 'image/png',
      },
    ],
    shortcuts: [
      {
        name: 'Quick Start Guide',
        short_name: 'Quick Start',
        description: 'Build your first robot in 5 minutes',
        url: '/getting-started/quick-start',
        icons: [{ src: '/icons/rocket.png', sizes: '96x96' }],
      },
      {
        name: 'Installation',
        short_name: 'Install',
        description: 'Get HORUS running instantly',
        url: '/getting-started/installation',
        icons: [{ src: '/icons/download.png', sizes: '96x96' }],
      },
      {
        name: 'Examples',
        short_name: 'Examples',
        description: 'Production-ready code samples',
        url: '/basic-examples',
        icons: [{ src: '/icons/code.png', sizes: '96x96' }],
      },
      {
        name: 'Benchmarks',
        short_name: 'Performance',
        description: 'See why HORUS is 575x faster',
        url: '/performance/benchmarks',
        icons: [{ src: '/icons/speed.png', sizes: '96x96' }],
      },
    ],
    related_applications: [
      {
        platform: 'web',
        url: 'https://github.com/softmata/HORUS',
        id: 'horus-github',
      },
    ],
    prefer_related_applications: false,
  };
}
