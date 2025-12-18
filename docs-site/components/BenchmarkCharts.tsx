'use client';

import React, { useEffect, useState } from 'react';
import {
  BarChart,
  Bar,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  Legend,
  ResponsiveContainer,
  LineChart,
  Line,
  AreaChart,
  Area,
  Cell,
} from 'recharts';

// Theme-aware colors - these are the actual color values
// HORUS branding colors (consistent across themes)
const BRAND_COLORS = {
  horus: '#22c55e',      // Green - HORUS
  horusLink: '#10b981',  // Emerald - HORUS Link
  horusHub: '#22c55e',   // Green - HORUS Hub
  ros2: '#ef4444',       // Red - ROS2
  accent: '#06b6d4',     // Cyan accent
};

// Hook to get current theme
function useTheme() {
  const [isDark, setIsDark] = useState(true);

  useEffect(() => {
    const checkTheme = () => {
      const theme = document.documentElement.getAttribute('data-theme');
      if (theme) {
        setIsDark(theme === 'dark');
      } else {
        // Check system preference
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

    // Also watch for system preference changes
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    mediaQuery.addEventListener('change', checkTheme);

    return () => {
      observer.disconnect();
      mediaQuery.removeEventListener('change', checkTheme);
    };
  }, []);

  return isDark;
}

// Get theme-aware colors
function useColors() {
  const isDark = useTheme();

  return {
    ...BRAND_COLORS,
    // Theme-adaptive colors
    grid: isDark ? '#374151' : '#d1d5db',
    text: isDark ? '#9ca3af' : '#6b7280',
    textBold: isDark ? '#f5f5f5' : '#1a1a1a',
    background: isDark ? '#1f2937' : '#f3f4f6',
    surface: isDark ? 'rgba(17, 24, 39, 0.5)' : 'rgba(243, 244, 246, 0.8)',
    cardBg: isDark ? 'rgba(31, 41, 55, 0.5)' : 'rgba(229, 231, 235, 0.5)',
    border: isDark ? '#374151' : '#d1d5db',
    tooltipBg: isDark ? '#1f2937' : '#ffffff',
    tooltipBorder: isDark ? '#374151' : '#e5e7eb',
  };
}

// Custom tooltip component
const CustomTooltip = ({ active, payload, label, colors }: any) => {
  if (active && payload && payload.length) {
    return (
      <div
        className="rounded-lg p-3 shadow-lg"
        style={{
          backgroundColor: colors.tooltipBg,
          border: `1px solid ${colors.border}`
        }}
      >
        <p className="font-medium mb-2" style={{ color: colors.textBold }}>{label}</p>
        {payload.map((entry: any, index: number) => (
          <p key={index} style={{ color: entry.color }} className="text-sm">
            {entry.name}: <span className="font-mono font-bold">{entry.value}</span>
            {entry.payload.unit && ` ${entry.payload.unit}`}
          </p>
        ))}
      </div>
    );
  }
  return null;
};

/**
 * Bar chart comparing HORUS vs ROS2 latency
 */
export function LatencyComparisonChart() {
  const colors = useColors();

  const data = [
    {
      name: 'CmdVel\n(16B)',
      'HORUS Link': 0.087,
      'HORUS Hub': 0.313,
      'ROS2 DDS': 75,
      unit: 'μs',
    },
    {
      name: 'IMU\n(304B)',
      'HORUS Link': 0.16,
      'HORUS Hub': 0.5,
      'ROS2 DDS': 115,
      unit: 'μs',
    },
    {
      name: 'LaserScan\n(1.5KB)',
      'HORUS Link': 0.4,
      'HORUS Hub': 2.2,
      'ROS2 DDS': 225,
      unit: 'μs',
    },
    {
      name: 'PointCloud\n(120KB)',
      'HORUS Link': 120,
      'HORUS Hub': 360,
      'ROS2 DDS': 750,
      unit: 'μs',
    },
  ];

  return (
    <div
      className="w-full rounded-xl p-6 my-6"
      style={{ backgroundColor: colors.surface, border: `1px solid ${colors.border}` }}
    >
      <h3 className="text-lg font-semibold mb-2" style={{ color: colors.textBold }}>
        Latency Comparison: HORUS vs ROS2
      </h3>
      <p className="text-sm mb-4" style={{ color: colors.text }}>
        Lower is better. Logarithmic scale (send-only latency in μs)
      </p>
      <ResponsiveContainer width="100%" height={350}>
        <BarChart data={data} margin={{ top: 20, right: 30, left: 20, bottom: 60 }}>
          <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
          <XAxis
            dataKey="name"
            stroke={colors.text}
            tick={{ fill: colors.text, fontSize: 11 }}
            interval={0}
            angle={0}
            textAnchor="middle"
            height={60}
          />
          <YAxis
            stroke={colors.text}
            tick={{ fill: colors.text }}
            scale="log"
            domain={[0.05, 1000]}
            tickFormatter={(value) => value < 1 ? `${(value * 1000).toFixed(0)}ns` : `${value}μs`}
            label={{ value: 'Latency (log scale)', angle: -90, position: 'insideLeft', fill: colors.text }}
          />
          <Tooltip content={<CustomTooltip colors={colors} />} />
          <Legend
            wrapperStyle={{ paddingTop: '20px' }}
            formatter={(value: any) => <span style={{ color: colors.text }}>{value}</span>}
          />
          <Bar dataKey="HORUS Link" fill={colors.horusLink} radius={[4, 4, 0, 0]} />
          <Bar dataKey="HORUS Hub" fill={colors.horusHub} radius={[4, 4, 0, 0]} />
          <Bar dataKey="ROS2 DDS" fill={colors.ros2} radius={[4, 4, 0, 0]} />
        </BarChart>
      </ResponsiveContainer>
      <div className="mt-4 flex flex-wrap justify-center gap-4 md:gap-6 text-sm">
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded" style={{ backgroundColor: colors.horusLink }}></div>
          <span style={{ color: colors.text }}>HORUS Link (SPSC, wait-free)</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded" style={{ backgroundColor: colors.horusHub }}></div>
          <span style={{ color: colors.text }}>HORUS Hub (MPMC, lock-free)</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded" style={{ backgroundColor: colors.ros2 }}></div>
          <span style={{ color: colors.text }}>ROS2 DDS (typical)</span>
        </div>
      </div>
    </div>
  );
}

/**
 * Line chart showing message size vs latency (linear scaling)
 */
export function LatencyScalingChart() {
  const colors = useColors();

  const data = [
    { size: 16, sizeLabel: '16B', horusLink: 87, horusHub: 313, ros2: 75000 },
    { size: 104, sizeLabel: '104B', horusLink: 350, horusHub: 600, ros2: 80000 },
    { size: 304, sizeLabel: '304B', horusLink: 400, horusHub: 940, ros2: 115000 },
    { size: 736, sizeLabel: '736B', horusLink: 600, horusHub: 1100, ros2: 150000 },
    { size: 1480, sizeLabel: '1.5KB', horusLink: 900, horusHub: 2200, ros2: 225000 },
    { size: 12000, sizeLabel: '12KB', horusLink: 7550, horusHub: 12000, ros2: 400000 },
    { size: 120000, sizeLabel: '120KB', horusLink: 120000, horusHub: 360000, ros2: 750000 },
  ];

  return (
    <div
      className="w-full rounded-xl p-6 my-6"
      style={{ backgroundColor: colors.surface, border: `1px solid ${colors.border}` }}
    >
      <h3 className="text-lg font-semibold mb-2" style={{ color: colors.textBold }}>
        Latency vs Message Size
      </h3>
      <p className="text-sm mb-4" style={{ color: colors.text }}>
        HORUS shows linear scaling. Values in nanoseconds.
      </p>
      <ResponsiveContainer width="100%" height={350}>
        <LineChart data={data} margin={{ top: 20, right: 30, left: 20, bottom: 20 }}>
          <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
          <XAxis
            dataKey="sizeLabel"
            stroke={colors.text}
            tick={{ fill: colors.text }}
            label={{ value: 'Message Size', position: 'bottom', fill: colors.text, offset: 0 }}
          />
          <YAxis
            stroke={colors.text}
            tick={{ fill: colors.text }}
            scale="log"
            domain={[50, 1000000]}
            tickFormatter={(value) => {
              if (value >= 1000000) return `${(value/1000000).toFixed(0)}ms`;
              if (value >= 1000) return `${(value/1000).toFixed(0)}μs`;
              return `${value}ns`;
            }}
            label={{ value: 'Latency (log scale)', angle: -90, position: 'insideLeft', fill: colors.text }}
          />
          <Tooltip
            formatter={(value: any) => {
              if (value === undefined || value === null) return ['', ''];
              if (value >= 1000000) return [`${(value/1000000).toFixed(2)}ms`, ''];
              if (value >= 1000) return [`${(value/1000).toFixed(2)}μs`, ''];
              return [`${value}ns`, ''];
            }}
            contentStyle={{ backgroundColor: colors.tooltipBg, border: `1px solid ${colors.tooltipBorder}`, borderRadius: '8px' }}
            labelStyle={{ color: colors.text }}
          />
          <Legend
            formatter={(value: any) => <span style={{ color: colors.text }}>{value}</span>}
          />
          <Line
            type="monotone"
            dataKey="horusLink"
            stroke={colors.horusLink}
            strokeWidth={3}
            dot={{ fill: colors.horusLink, strokeWidth: 2, r: 5 }}
            name="HORUS Link"
          />
          <Line
            type="monotone"
            dataKey="horusHub"
            stroke={colors.horusHub}
            strokeWidth={3}
            dot={{ fill: colors.horusHub, strokeWidth: 2, r: 5 }}
            name="HORUS Hub"
          />
          <Line
            type="monotone"
            dataKey="ros2"
            stroke={colors.ros2}
            strokeWidth={3}
            dot={{ fill: colors.ros2, strokeWidth: 2, r: 5 }}
            name="ROS2 DDS"
            strokeDasharray="5 5"
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}

/**
 * Line chart showing node count vs execution time (scalability)
 */
export function ScalabilityChart() {
  const colors = useColors();

  const data = [
    { nodes: 10, time: 106.93, label: '10 nodes' },
    { nodes: 50, time: 113.93, label: '50 nodes' },
    { nodes: 100, time: 116.49, label: '100 nodes' },
    { nodes: 200, time: 119.55, label: '200 nodes' },
  ];

  // Calculate theoretical linear scaling for reference
  const linearData = data.map(d => ({
    ...d,
    linear: 106.93 * (d.nodes / 10), // What it would be with linear (bad) scaling
  }));

  return (
    <div
      className="w-full rounded-xl p-6 my-6"
      style={{ backgroundColor: colors.surface, border: `1px solid ${colors.border}` }}
    >
      <h3 className="text-lg font-semibold mb-2" style={{ color: colors.textBold }}>
        Scheduler Scalability
      </h3>
      <p className="text-sm mb-4" style={{ color: colors.text }}>
        Near-constant execution time regardless of node count
      </p>
      <ResponsiveContainer width="100%" height={350}>
        <AreaChart data={linearData} margin={{ top: 20, right: 30, left: 20, bottom: 20 }}>
          <defs>
            <linearGradient id="colorHorus" x1="0" y1="0" x2="0" y2="1">
              <stop offset="5%" stopColor={colors.horus} stopOpacity={0.3}/>
              <stop offset="95%" stopColor={colors.horus} stopOpacity={0}/>
            </linearGradient>
            <linearGradient id="colorLinear" x1="0" y1="0" x2="0" y2="1">
              <stop offset="5%" stopColor={colors.ros2} stopOpacity={0.1}/>
              <stop offset="95%" stopColor={colors.ros2} stopOpacity={0}/>
            </linearGradient>
          </defs>
          <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
          <XAxis
            dataKey="nodes"
            stroke={colors.text}
            tick={{ fill: colors.text }}
            label={{ value: 'Number of Nodes', position: 'bottom', fill: colors.text, offset: 0 }}
          />
          <YAxis
            stroke={colors.text}
            tick={{ fill: colors.text }}
            domain={[0, 2500]}
            tickFormatter={(value) => `${value}ms`}
            label={{ value: 'Execution Time', angle: -90, position: 'insideLeft', fill: colors.text }}
          />
          <Tooltip
            formatter={(value: any, name: any) => [
              value !== undefined && value !== null ? `${value.toFixed(2)}ms` : '',
              name === 'linear' ? 'Linear Scaling (bad)' : 'HORUS Actual'
            ]}
            contentStyle={{ backgroundColor: colors.tooltipBg, border: `1px solid ${colors.tooltipBorder}`, borderRadius: '8px' }}
            labelStyle={{ color: colors.text }}
            labelFormatter={(label: any) => `${label} nodes`}
          />
          <Legend
            formatter={(value: any) => (
              <span style={{ color: colors.text }}>
                {value === 'linear' ? 'Linear Scaling (what to avoid)' : 'HORUS Actual'}
              </span>
            )}
          />
          <Area
            type="monotone"
            dataKey="linear"
            stroke={colors.ros2}
            strokeWidth={2}
            strokeDasharray="5 5"
            fillOpacity={1}
            fill="url(#colorLinear)"
            name="linear"
          />
          <Area
            type="monotone"
            dataKey="time"
            stroke={colors.horus}
            strokeWidth={3}
            fillOpacity={1}
            fill="url(#colorHorus)"
            name="HORUS"
            dot={{ fill: colors.horus, strokeWidth: 2, r: 6 }}
          />
        </AreaChart>
      </ResponsiveContainer>
      <div className="mt-4 grid grid-cols-2 md:grid-cols-4 gap-4 text-center">
        {data.map((d) => (
          <div
            key={d.nodes}
            className="rounded-lg p-3"
            style={{ backgroundColor: colors.cardBg }}
          >
            <div className="text-2xl font-bold" style={{ color: colors.horus }}>{d.time.toFixed(1)}ms</div>
            <div className="text-sm" style={{ color: colors.text }}>{d.nodes} nodes</div>
            <div className="text-xs mt-1" style={{ color: colors.text }}>
              {d.nodes > 10 ? `${((d.time / 106.93 - 1) * 100).toFixed(1)}% overhead` : 'baseline'}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

/**
 * Speedup comparison chart - shows how much faster HORUS is
 */
export function SpeedupChart() {
  const colors = useColors();

  const data = [
    { name: 'CmdVel', speedup: 575, category: 'Control' },
    { name: 'IMU', speedup: 940, category: 'Sensor' },
    { name: 'LaserScan', speedup: 750, category: 'Perception' },
    { name: 'Odometry', speedup: 167, category: 'Localization' },
    { name: 'PointCloud', speedup: 8, category: 'Vision' },
  ];

  return (
    <div
      className="w-full rounded-xl p-6 my-6"
      style={{ backgroundColor: colors.surface, border: `1px solid ${colors.border}` }}
    >
      <h3 className="text-lg font-semibold mb-2" style={{ color: colors.textBold }}>
        HORUS Speedup vs ROS2
      </h3>
      <p className="text-sm mb-4" style={{ color: colors.text }}>
        How many times faster HORUS Link is compared to ROS2 DDS
      </p>
      <ResponsiveContainer width="100%" height={300}>
        <BarChart data={data} layout="vertical" margin={{ top: 20, right: 80, left: 80, bottom: 20 }}>
          <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} horizontal={true} vertical={false} />
          <XAxis
            type="number"
            stroke={colors.text}
            tick={{ fill: colors.text }}
            tickFormatter={(value) => `${value}x`}
            domain={[0, 1000]}
          />
          <YAxis
            type="category"
            dataKey="name"
            stroke={colors.text}
            tick={{ fill: colors.text }}
            width={80}
          />
          <Tooltip
            formatter={(value: any) => [`${value}x faster`, 'Speedup']}
            contentStyle={{ backgroundColor: colors.tooltipBg, border: `1px solid ${colors.tooltipBorder}`, borderRadius: '8px' }}
            labelStyle={{ color: colors.text }}
          />
          <Bar dataKey="speedup" radius={[0, 4, 4, 0]}>
            {data.map((entry, index) => (
              <Cell
                key={`cell-${index}`}
                fill={entry.speedup > 500 ? colors.horus : entry.speedup > 100 ? colors.horusLink : colors.accent}
              />
            ))}
          </Bar>
        </BarChart>
      </ResponsiveContainer>
      <div className="mt-4 flex flex-wrap justify-center gap-4 md:gap-6 text-sm">
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded" style={{ backgroundColor: colors.horus }}></div>
          <span style={{ color: colors.text }}>&gt;500x faster</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded" style={{ backgroundColor: colors.horusLink }}></div>
          <span style={{ color: colors.text }}>100-500x faster</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded" style={{ backgroundColor: colors.accent }}></div>
          <span style={{ color: colors.text }}>&lt;100x faster</span>
        </div>
      </div>
    </div>
  );
}

/**
 * Throughput comparison chart
 */
export function ThroughputChart() {
  const colors = useColors();

  const data = [
    { name: 'Small\n(16B)', horusLink: 12, horusHub: 3, ros2: 0.02 },
    { name: 'Medium\n(1KB)', horusLink: 3, horusHub: 1.5, ros2: 0.01 },
    { name: 'Large\n(100KB)', horusLink: 0.5, horusHub: 0.1, ros2: 0.002 },
  ];

  return (
    <div
      className="w-full rounded-xl p-6 my-6"
      style={{ backgroundColor: colors.surface, border: `1px solid ${colors.border}` }}
    >
      <h3 className="text-lg font-semibold mb-2" style={{ color: colors.textBold }}>
        Throughput Comparison
      </h3>
      <p className="text-sm mb-4" style={{ color: colors.text }}>
        Messages per second (millions). Higher is better.
      </p>
      <ResponsiveContainer width="100%" height={300}>
        <BarChart data={data} margin={{ top: 20, right: 30, left: 20, bottom: 40 }}>
          <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
          <XAxis
            dataKey="name"
            stroke={colors.text}
            tick={{ fill: colors.text, fontSize: 11 }}
            interval={0}
          />
          <YAxis
            stroke={colors.text}
            tick={{ fill: colors.text }}
            scale="log"
            domain={[0.001, 20]}
            tickFormatter={(value) => `${value}M`}
            label={{ value: 'Messages/sec (millions, log)', angle: -90, position: 'insideLeft', fill: colors.text }}
          />
          <Tooltip
            formatter={(value: any) => [`${value}M msg/s`, '']}
            contentStyle={{ backgroundColor: colors.tooltipBg, border: `1px solid ${colors.tooltipBorder}`, borderRadius: '8px' }}
            labelStyle={{ color: colors.text }}
          />
          <Legend
            formatter={(value: any) => <span style={{ color: colors.text }}>{value}</span>}
          />
          <Bar dataKey="horusLink" fill={colors.horusLink} radius={[4, 4, 0, 0]} name="HORUS Link" />
          <Bar dataKey="horusHub" fill={colors.horusHub} radius={[4, 4, 0, 0]} name="HORUS Hub" />
          <Bar dataKey="ros2" fill={colors.ros2} radius={[4, 4, 0, 0]} name="ROS2 DDS" />
        </BarChart>
      </ResponsiveContainer>
    </div>
  );
}

/**
 * Real-time performance chart showing jitter and deadline accuracy
 */
export function RealTimeChart() {
  const colors = useColors();

  const data = [
    { name: 'RT Critical\n1000 Hz', target: 1000, achieved: 999.8, jitter: 10, misses: 0 },
    { name: 'RT High\n500 Hz', target: 500, achieved: 499.9, jitter: 15, misses: 0 },
    { name: 'Normal\n100 Hz', target: 100, achieved: 99.9, jitter: 50, misses: 0.1 },
    { name: 'Background\n10 Hz', target: 10, achieved: 10, jitter: 200, misses: 0.5 },
  ];

  return (
    <div
      className="w-full rounded-xl p-6 my-6"
      style={{ backgroundColor: colors.surface, border: `1px solid ${colors.border}` }}
    >
      <h3 className="text-lg font-semibold mb-2" style={{ color: colors.textBold }}>
        Real-Time Node Performance
      </h3>
      <p className="text-sm mb-4" style={{ color: colors.text }}>
        Target rate achievement and jitter measurements
      </p>
      <div className="grid grid-cols-2 md:grid-cols-4 gap-4 mb-6">
        {data.map((d) => (
          <div
            key={d.name}
            className="rounded-lg p-4 text-center"
            style={{ backgroundColor: colors.cardBg }}
          >
            <div className="text-xs mb-1 whitespace-pre-line" style={{ color: colors.text }}>{d.name}</div>
            <div className="text-2xl font-bold" style={{ color: colors.horus }}>
              {((d.achieved / d.target) * 100).toFixed(1)}%
            </div>
            <div className="text-xs" style={{ color: colors.text }}>rate achieved</div>
            <div className="mt-2 text-sm">
              <span style={{ color: colors.accent }}>±{d.jitter}μs</span>
              <span style={{ color: colors.text }}> jitter</span>
            </div>
            <div className="text-xs" style={{ color: colors.text }}>
              {d.misses}% deadline misses
            </div>
          </div>
        ))}
      </div>
      <ResponsiveContainer width="100%" height={200}>
        <BarChart data={data} margin={{ top: 10, right: 30, left: 20, bottom: 40 }}>
          <CartesianGrid strokeDasharray="3 3" stroke={colors.grid} />
          <XAxis
            dataKey="name"
            stroke={colors.text}
            tick={{ fill: colors.text, fontSize: 10 }}
            interval={0}
          />
          <YAxis
            stroke={colors.text}
            tick={{ fill: colors.text }}
            domain={[0, 250]}
            tickFormatter={(value) => `${value}μs`}
            label={{ value: 'Jitter', angle: -90, position: 'insideLeft', fill: colors.text }}
          />
          <Tooltip
            formatter={(value: any) => [`±${value}μs`, 'Jitter']}
            contentStyle={{ backgroundColor: colors.tooltipBg, border: `1px solid ${colors.tooltipBorder}`, borderRadius: '8px' }}
            labelStyle={{ color: colors.text }}
          />
          <Bar dataKey="jitter" fill={colors.accent} radius={[4, 4, 0, 0]} name="Jitter" />
        </BarChart>
      </ResponsiveContainer>
    </div>
  );
}

// Export all charts as a single default for easy MDX import
export default {
  LatencyComparisonChart,
  LatencyScalingChart,
  ScalabilityChart,
  SpeedupChart,
  ThroughputChart,
  RealTimeChart,
};
