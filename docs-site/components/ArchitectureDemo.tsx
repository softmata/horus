'use client';

import { useEffect, useState } from 'react';

export function ArchitectureDemo() {
  const [dataPackets, setDataPackets] = useState<Array<{
    id: number;
    progress: number;
    sourceNode: number;
    targetTopic: number;
    direction: 'publish' | 'subscribe';
  }>>([]);

  useEffect(() => {
    // Spawn data packets periodically
    const interval = setInterval(() => {
      setDataPackets(prev => {
        // Remove completed packets and update progress
        const updated = prev
          .map(p => ({ ...p, progress: p.progress + 2 }))
          .filter(p => p.progress < 100);

        // Add new packet randomly (both publish and subscribe)
        if (Math.random() > 0.7) {
          const sourceNode = Math.floor(Math.random() * 3);
          const targetTopic = Math.floor(Math.random() * 4);
          const direction = Math.random() > 0.5 ? 'publish' : 'subscribe';

          updated.push({
            id: Date.now() + Math.random(),
            progress: 0,
            sourceNode,
            targetTopic,
            direction
          });
        }

        return updated;
      });
    }, 50);

    return () => clearInterval(interval);
  }, []);

  // Node positions (left side)
  const nodes = [
    { x: 250, y: 180, label: 'SensorNode', color: 'var(--accent)' },
    { x: 250, y: 280, label: 'ControlNode', color: 'var(--accent)' },
    { x: 250, y: 380, label: 'ActuatorNode', color: 'var(--accent)' }
  ];

  // Topics inside shared memory (left side of panel)
  const topics = [
    { y: 180, label: 'sensors.imu' },
    { y: 250, label: 'cmd_vel' },
    { y: 320, label: 'odom' },
    { y: 390, label: 'laser_scan' }
  ];

  // Log entries (right side of panel) - simulating real-time logs
  const [logOffset, setLogOffset] = useState(0);

  useEffect(() => {
    const interval = setInterval(() => {
      setLogOffset(prev => (prev + 1) % 100);
    }, 150);
    return () => clearInterval(interval);
  }, []);

  // Extended log entries for continuous scrolling (updated with wait-free IPC times)
  const allLogEntries = [
    { time: '12:39:28.039', ipc: '87ns', tick: '218μs', node: 'SensorNode', type: 'PUB', topic: 'sensors.imu' },
    { time: '12:39:28.039', ipc: '312ns', tick: '10μs', node: 'ControlNode', type: 'SUB', topic: 'sensors.imu' },
    { time: '12:39:28.156', ipc: '91ns', tick: '31μs', node: 'SensorNode', type: 'PUB', topic: 'cmd_vel' },
    { time: '12:39:28.207', ipc: '298ns', tick: '41μs', node: 'ActuatorNode', type: 'SUB', topic: 'cmd_vel' },
    { time: '12:39:28.208', ipc: '84ns', tick: '241μs', node: 'ControlNode', type: 'PUB', topic: 'odom' },
    { time: '12:39:28.219', ipc: '321ns', tick: '25μs', node: 'ControlNode', type: 'SUB', topic: 'odom' },
    { time: '12:39:28.358', ipc: '89ns', tick: '29μs', node: 'SensorNode', type: 'PUB', topic: 'laser_scan' },
    { time: '12:39:28.408', ipc: '287ns', tick: '30μs', node: 'ActuatorNode', type: 'SUB', topic: 'laser_scan' },
    { time: '12:39:28.558', ipc: '82ns', tick: '25μs', node: 'SensorNode', type: 'PUB', topic: 'sensors.imu' },
    { time: '12:39:28.608', ipc: '305ns', tick: '28μs', node: 'ControlNode', type: 'SUB', topic: 'sensors.imu' },
    { time: '12:39:28.708', ipc: '93ns', tick: '35μs', node: 'ActuatorNode', type: 'PUB', topic: 'cmd_vel' },
    { time: '12:39:28.808', ipc: '276ns', tick: '42μs', node: 'SensorNode', type: 'SUB', topic: 'laser_scan' },
    { time: '12:39:28.908', ipc: '86ns', tick: '19μs', node: 'ControlNode', type: 'PUB', topic: 'sensors.imu' },
    { time: '12:39:29.008', ipc: '318ns', tick: '51μs', node: 'ActuatorNode', type: 'SUB', topic: 'odom' },
    { time: '12:39:29.108', ipc: '88ns', tick: '27μs', node: 'SensorNode', type: 'PUB', topic: 'cmd_vel' }
  ];

  return (
    <div className="w-full relative bg-[var(--surface)] border border-[var(--border)] overflow-hidden">
      {/* Mobile scroll hint */}
      <div className="md:hidden absolute top-2 right-4 z-10 text-xs text-[var(--text-tertiary)] bg-[var(--surface)] px-2 py-1 rounded border border-[var(--border)] opacity-75">
         Scroll 
      </div>
      <div className="w-full h-[300px] md:h-[400px] overflow-x-auto overflow-y-hidden scrollbar-thin scrollbar-thumb-[var(--accent)] scrollbar-track-transparent">
        <svg
          width="100%"
          height="100%"
          viewBox="100 0 1600 520"
          className="overflow-visible min-w-[1200px] md:min-w-0"
          preserveAspectRatio="xMidYMid meet"
        >

        {/* Title */}
        <text x="900" y="30" fill="var(--text)" fontSize="18" textAnchor="middle" fontWeight="700">
          HORUS IPC Architecture
        </text>
        <text x="900" y="50" fill="var(--text-tertiary)" fontSize="13" textAnchor="middle">
          87ns latency (wait-free) • Zero-copy shared memory communication
        </text>

        {/* Shared Memory Region (Center-Right) */}
        <rect
          x="650"
          y="135"
          width="1100"
          height="330"
          fill="var(--surface)"
          stroke="var(--accent)"
          strokeWidth="3"
          rx="12"
        />
        <text x="1200" y="118" fill="var(--accent)" fontSize="15" textAnchor="middle" fontWeight="700" letterSpacing="1">
          SHARED MEMORY
        </text>

        {/* Divider line between topics and logs */}
        <line
          x1="1150"
          y1="145"
          x2="1150"
          y2="455"
          stroke="var(--accent)"
          strokeWidth="2"
          strokeOpacity="0.3"
          strokeDasharray="4,4"
        />

        {/* Connection Lines - from each node to each topic */}
        {nodes.map((node, i) =>
          topics.map((topic, j) => (
            <line
              key={`line-${i}-${j}`}
              x1={node.x + 35}
              y1={node.y}
              x2={695}
              y2={topic.y}
              stroke="var(--border)"
              strokeWidth="1.5"
            />
          ))
        )}

        {/* Data Packets (animated) */}
        {dataPackets.map(packet => {
          const node = nodes[packet.sourceNode];
          const topic = topics[packet.targetTopic];

          // Determine start and end based on direction
          const isPublish = packet.direction === 'publish';
          const startX = isPublish ? node.x + 35 : 695;
          const endX = isPublish ? 695 : node.x + 35;
          const startY = isPublish ? node.y : topic.y;
          const endY = isPublish ? topic.y : node.y;

          const x = startX + (endX - startX) * (packet.progress / 100);
          const y = startY + (endY - startY) * (packet.progress / 100);

          // Different colors for publish vs subscribe
          const color = isPublish ? 'var(--text)' : 'var(--text-muted)';

          return (
            <g key={packet.id}>
              <circle
                cx={x}
                cy={y}
                r="7"
                fill={color}
                opacity={0.95}
              >
                <animate
                  attributeName="r"
                  values="7;9;7"
                  dur="0.6s"
                  repeatCount="indefinite"
                />
              </circle>
              <circle
                cx={x}
                cy={y}
                r="14"
                fill="none"
                stroke={color}
                strokeWidth="2"
                opacity={0.5}
              />
            </g>
          );
        })}

        {/* Nodes (left side) - floating animation */}
        {nodes.map((node, i) => (
          <g key={`node-${i}`}>
            {/* Node outer circle */}
            <circle
              cx={node.x}
              cy={node.y}
              r="35"
              fill="var(--surface)"
              stroke={node.color}
              strokeWidth="3"
            >
              <animate
                attributeName="cy"
                values={`${node.y};${node.y - 8};${node.y}`}
                dur={`${4 + i * 0.5}s`}
                repeatCount="indefinite"
              />
              <animate
                attributeName="opacity"
                values="0.8;1;0.8"
                dur="3s"
                repeatCount="indefinite"
              />
            </circle>
            <text
              x={node.x}
              y={node.y - 5}
              fill="var(--text)"
              fontSize="12"
              textAnchor="middle"
              fontWeight="700"
            >
              <animate
                attributeName="y"
                values={`${node.y - 5};${node.y - 13};${node.y - 5}`}
                dur={`${4 + i * 0.5}s`}
                repeatCount="indefinite"
              />
              {node.label.replace('Node', '')}
            </text>
            <text
              x={node.x}
              y={node.y + 8}
              fill="var(--text-secondary)"
              fontSize="10"
              textAnchor="middle"
              fontWeight="500"
            >
              <animate
                attributeName="y"
                values={`${node.y + 8};${node.y};${node.y + 8}`}
                dur={`${4 + i * 0.5}s`}
                repeatCount="indefinite"
              />
              Node
            </text>
          </g>
        ))}

        {/* Topics (left side of shared memory) */}
        {topics.map((topic, i) => (
          <g key={`topic-${i}`}>
            <rect
              x={700}
              y={topic.y - 24}
              width="420"
              height="48"
              fill="var(--surface)"
              stroke="var(--success)"
              strokeWidth="2.5"
              rx="10"
            />
            <circle
              cx={730}
              cy={topic.y}
              r="7"
              fill="var(--success)"
              opacity="0.6"
            >
              <animate
                attributeName="opacity"
                values="0.4;0.9;0.4"
                dur="2s"
                repeatCount="indefinite"
              />
            </circle>
            <text
              x={910}
              y={topic.y + 6}
              fill="var(--success)"
              fontSize="16"
              textAnchor="middle"
              fontFamily="monospace"
              fontWeight="700"
            >
              {topic.label}
            </text>
          </g>
        ))}

        {/* Log entries panel header (right side of shared memory) */}
        <text x="1450" y="165" fill="var(--text-muted)" fontSize="13" textAnchor="middle" fontWeight="700" letterSpacing="0.5">
          Live Logs
        </text>

        {/* Clipping region for scrolling logs */}
        <defs>
          <clipPath id="log-clip">
            <rect x="1160" y="185" width="580" height="265" />
          </clipPath>
        </defs>

        {/* Log entries (right side of shared memory) with scrolling animation */}
        <g clipPath="url(#log-clip)">
          {allLogEntries.concat(allLogEntries).map((log, i) => {
            // Double the entries for seamless looping
            const baseY = 210 + (i * 20);
            const scrollY = baseY - (logOffset * 2); // Scroll upward

            // Loop when reaching the end
            const loopedY = scrollY > 460 ? scrollY - (allLogEntries.length * 20 * 2) : scrollY;

            // Only render if visible in the clipping region
            if (loopedY < 180 || loopedY > 460) return null;

            // Different arrows: PUB --> vs SUB <--
            const arrow = log.type === 'PUB' ? '-->' : '<--';

            return (
              <text
                key={`log-${i}`}
                x={1170}
                y={loopedY}
                fill="var(--text-tertiary)"
                fontSize="9"
                fontFamily="monospace"
                opacity={0.85}
              >
                <tspan fill="var(--text-tertiary)">[{log.time}]</tspan>
                <tspan fill="var(--text-tertiary)"> [IPC: </tspan>
                <tspan fill="var(--text-muted)">{log.ipc}</tspan>
                <tspan fill="var(--text-tertiary)"> | Tick: </tspan>
                <tspan fill="var(--text-muted)">{log.tick}</tspan>
                <tspan fill="var(--text-tertiary)">] </tspan>
                <tspan fill="var(--text-secondary)">{log.node}</tspan>
                <tspan fill={log.type === 'PUB' ? 'var(--accent)' : 'var(--success)'}> {arrow} </tspan>
                <tspan fill="var(--success)">'{log.topic}'</tspan>
              </text>
            );
          })}
        </g>

        {/* Legend */}
        <g transform="translate(250, 485)">
          <circle cx="0" cy="0" r="7" fill="var(--accent)" opacity="0.95" />
          <text x="15" y="5" fill="var(--text-secondary)" fontSize="13" fontWeight="600">
            Publish (Node  Topic)
          </text>

          <circle cx="0" cy="25" r="7" fill="var(--success)" opacity="0.95" />
          <text x="15" y="30" fill="var(--text-secondary)" fontSize="13" fontWeight="600">
            Subscribe (Topic  Node)
          </text>
        </g>
      </svg>
    </div>
  </div>
  );
}
