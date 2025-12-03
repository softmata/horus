"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { FiChevronDown, FiChevronRight, FiX } from "react-icons/fi";
import { useState, useEffect } from "react";

interface DocLink {
  title: string;
  href: string;
  order?: number;
  children?: DocLink[];
}

interface SidebarSection {
  title: string;
  links: DocLink[];
}

const sections: SidebarSection[] = [
  {
    title: "Getting Started",
    links: [
      { title: "What is HORUS?", href: "/what-is-horus", order: 0 },
      { title: "Goals & Vision", href: "/goals", order: 1 },
      { title: "Complete Beginner's Guide", href: "/complete-beginners-guide", order: 2 },
      { title: "Installation", href: "/getting-started/installation", order: 4 },
      { title: "Quick Start", href: "/getting-started/quick-start", order: 5 },
      { title: "Second Application", href: "/second-application", order: 6 },
      { title: "Architecture", href: "/architecture", order: 7 },
            { title: "Troubleshooting", href: "/troubleshooting", order: 8 },
      { title: "Runtime Errors", href: "/troubleshooting-runtime", order: 9 },
      { title: "Robot Architectures", href: "/robot-architectures", order: 10 },
      { title: "Basic Examples", href: "/basic-examples", order: 11 },
      { title: "Advanced Examples", href: "/advanced-examples", order: 12 },
    ],
  },
  {
    title: "Core Concepts",
    links: [
      { title: "Overview", href: "/core-concepts/core", order: 1 },
      { title: "Nodes", href: "/core-concepts/core-concepts-nodes", order: 2 },
      {
        title: "Communication Patterns",
        href: "/core-concepts/communication-overview",
        order: 3,
        children: [
          { title: "Hub (MPMC)", href: "/core-concepts/core-concepts-hub", order: 1 },
          { title: "Link (SPSC)", href: "/core-concepts/core-concepts-link", order: 2 },
        ]
      },
      {
        title: "Communication Transport",
        href: "/core-concepts/communication-transport",
        order: 4,
        children: [
          { title: "Local (Shared Memory)", href: "/core-concepts/core-concepts-shared-memory", order: 1 },
          { title: "Network", href: "/core-concepts/network-communication", order: 2 },
          { title: "Configuration", href: "/core-concepts/communication-configuration", order: 3 },
        ]
      },
      { title: "Scheduler", href: "/core-concepts/core-concepts-scheduler", order: 5 },
      { title: "node! Macro", href: "/core-concepts/node-macro", order: 6 },
      { title: "message! Macro", href: "/core-concepts/message-macro", order: 7 },
      { title: "Message Types", href: "/core-concepts/message-types", order: 8 },
      { title: "Real-Time Nodes", href: "/core-concepts/realtime-nodes", order: 9 },
      { title: "Hybrid Nodes & Processors", href: "/core-concepts/hybrid-nodes", order: 10 },
    ],
  },
  {
    title: "Development",
    links: [
      { title: "CLI Reference", href: "/development/cli-reference", order: 1 },
      { title: "Dashboard", href: "/development/dashboard", order: 2 },
      { title: "Simulation", href: "/development/simulation", order: 3 },
      { title: "Testing", href: "/development/testing", order: 4 },
      { title: "Parameters", href: "/development/parameters", order: 5 },
      { title: "Library Reference", href: "/development/library-reference", order: 6 },
    ],
  },
  {
    title: "Sim3D Simulator",
    links: [
      { title: "Overview", href: "/sim3d", order: 0 },
      { title: "Installation", href: "/sim3d/getting-started/installation", order: 1 },
      { title: "Quick Start", href: "/sim3d/getting-started/quick-start", order: 2 },
      { title: "Sensors", href: "/sim3d/sensors/overview", order: 3 },
      { title: "Physics", href: "/sim3d/physics/overview", order: 4 },
      { title: "Reinforcement Learning", href: "/sim3d/rl/overview", order: 5 },
    ],
  },
  {
    title: "Built-in Nodes/Drivers",
    links: [
      { title: "Overview", href: "/built-in-nodes", order: 0 },
      {
        title: "Communication",
        href: "/built-in-nodes/i2c-bus",
        order: 1,
        children: [
          { title: "I2C Bus", href: "/built-in-nodes/i2c-bus", order: 1 },
          { title: "SPI Bus", href: "/built-in-nodes/spi-bus", order: 2 },
          { title: "CAN Bus", href: "/built-in-nodes/can-bus", order: 3 },
          { title: "Serial", href: "/built-in-nodes/serial", order: 4 },
          { title: "Modbus", href: "/built-in-nodes/modbus", order: 5 },
        ]
      },
      {
        title: "Motors",
        href: "/built-in-nodes/dc-motor",
        order: 2,
        children: [
          { title: "DC Motor", href: "/built-in-nodes/dc-motor", order: 1 },
          { title: "BLDC Motor", href: "/built-in-nodes/bldc-motor", order: 2 },
          { title: "Stepper Motor", href: "/built-in-nodes/stepper-motor", order: 3 },
          { title: "Servo Controller", href: "/built-in-nodes/servo-controller", order: 4 },
          { title: "Dynamixel", href: "/built-in-nodes/dynamixel", order: 5 },
          { title: "RoboClaw Motor", href: "/built-in-nodes/roboclaw-motor", order: 6 },
        ]
      },
      {
        title: "Sensors",
        href: "/built-in-nodes/camera",
        order: 3,
        children: [
          { title: "Camera", href: "/built-in-nodes/camera", order: 1 },
          { title: "Depth Camera", href: "/built-in-nodes/depth-camera", order: 2 },
          { title: "LiDAR", href: "/built-in-nodes/lidar", order: 3 },
          { title: "IMU", href: "/built-in-nodes/imu", order: 4 },
          { title: "GPS", href: "/built-in-nodes/gps", order: 5 },
          { title: "Encoder", href: "/built-in-nodes/encoder", order: 6 },
          { title: "Ultrasonic", href: "/built-in-nodes/ultrasonic", order: 7 },
          { title: "Battery Monitor", href: "/built-in-nodes/battery-monitor", order: 8 },
          { title: "Force/Torque", href: "/built-in-nodes/force-torque", order: 9 },
        ]
      },
      {
        title: "Control & Navigation",
        href: "/built-in-nodes/pid-controller",
        order: 4,
        children: [
          { title: "PID Controller", href: "/built-in-nodes/pid-controller", order: 1 },
          { title: "Differential Drive", href: "/built-in-nodes/differential-drive", order: 2 },
          { title: "Odometry", href: "/built-in-nodes/odometry", order: 3 },
          { title: "Path Planner", href: "/built-in-nodes/path-planner", order: 4 },
          { title: "Localization", href: "/built-in-nodes/localization", order: 5 },
        ]
      },
      {
        title: "Safety & I/O",
        href: "/built-in-nodes/emergency-stop",
        order: 5,
        children: [
          { title: "Emergency Stop", href: "/built-in-nodes/emergency-stop", order: 1 },
          { title: "Safety Monitor", href: "/built-in-nodes/safety-monitor", order: 2 },
          { title: "Collision Detector", href: "/built-in-nodes/collision-detector", order: 3 },
          { title: "Digital I/O", href: "/built-in-nodes/digital-io", order: 4 },
        ]
      },
      {
        title: "User Interfaces",
        href: "/built-in-nodes/joystick",
        order: 6,
        children: [
          { title: "Joystick", href: "/built-in-nodes/joystick", order: 1 },
          { title: "Keyboard Input", href: "/built-in-nodes/keyboard-input", order: 2 },
          { title: "Image Processor", href: "/built-in-nodes/image-processor", order: 3 },
        ]
      },
      {
        title: "Computer Vision",
        href: "/built-in-nodes/yolo-detector",
        order: 7,
        children: [
          { title: "YOLO Detector", href: "/built-in-nodes/yolo-detector", order: 1 },
          { title: "Pose Estimation", href: "/built-in-nodes/pose-estimation", order: 2 },
          { title: "Segmentation", href: "/built-in-nodes/semantic-segmentation", order: 3 },
          { title: "Visual Odometry", href: "/built-in-nodes/visual-odometry", order: 4 },
          { title: "Depth Estimation", href: "/built-in-nodes/depth-estimation", order: 5 },
          { title: "Embedding", href: "/built-in-nodes/embedding", order: 6 },
        ]
      },
      {
        title: "ML & AI",
        href: "/built-in-nodes/onnx-inference",
        order: 8,
        children: [
          { title: "ONNX Inference", href: "/built-in-nodes/onnx-inference", order: 1 },
          { title: "TensorRT Inference", href: "/built-in-nodes/tensorrt-inference", order: 2 },
          { title: "TFLite Inference", href: "/built-in-nodes/tflite-inference", order: 3 },
          { title: "Cloud LLM", href: "/built-in-nodes/cloud-llm", order: 4 },
        ]
      },
    ],
  },
  {
    title: "Package Management",
    links: [
      { title: "Package Management", href: "/package-management/package-management", order: 1 },
      { title: "Using Prebuilt Nodes", href: "/package-management/using-prebuilt-nodes", order: 2 },
      { title: "Environment Management", href: "/package-management/environment-management", order: 3 },
      { title: "Configuration Reference", href: "/package-management/configuration", order: 4 },
    ],
  },
  {
    title: "Multi-Language",
    links: [
      { title: "Overview", href: "/multi-language/multi-language", order: 1 },
      { title: "Python Bindings", href: "/multi-language/python-bindings", order: 2 },
      { title: "Python Message Library", href: "/multi-language/python-message-library", order: 3 },
      { title: "Python Hardware Nodes", href: "/multi-language/python-hardware-nodes", order: 4 },
      { title: "AI API Integration", href: "/multi-language/ai-integration", order: 5 },
    ],
  },
  {
    title: "Performance",
    links: [
      { title: "Optimization Guide", href: "/performance/performance", order: 1 },
      { title: "Benchmarks", href: "/performance/benchmarks", order: 2 },
    ],
  },
  {
    title: "Advanced Topics",
    links: [
      { title: "Scheduler Configuration", href: "/advanced/scheduler-configuration", order: 1 },
      { title: "Execution Modes", href: "/advanced/execution-modes", order: 2 },
      { title: "Deterministic Execution", href: "/advanced/deterministic-execution", order: 3 },
      { title: "Robot Presets", href: "/advanced/robot-presets", order: 4 },
      { title: "RTOS Integration", href: "/advanced/rtos-integration", order: 5 },
      { title: "GPU Tensor Sharing", href: "/advanced/gpu-tensor-sharing", order: 6 },
    ],
  },
  {
    title: "API Reference",
    links: [
      { title: "Overview", href: "/api", order: 0 },
      { title: "horus_core", href: "/api/core", order: 1 },
      { title: "horus_messages", href: "/api/messages", order: 2 },
      { title: "horus_macros", href: "/api/macros", order: 3 },
      { title: "TensorPool", href: "/api/tensor-pool", order: 4 },
    ],
  },
];

interface DocsSidebarProps {
  isOpen?: boolean;
  onClose?: () => void;
}

export function DocsSidebar({ isOpen = true, onClose }: DocsSidebarProps) {
  const pathname = usePathname();
  const [expandedSections, setExpandedSections] = useState<Record<string, boolean>>({
    "Getting Started": true,
    "Core Concepts": true,
    "Development": true,
    "Sim3D Simulator": true,
    "Built-in Nodes/Drivers": true,
    "Package Management": true,
    "Multi-Language": true,
    "Performance": true,
    "Advanced Topics": true,
    "API Reference": true,
  });

  // Track expanded nested items (by href)
  const [expandedItems, setExpandedItems] = useState<Record<string, boolean>>({});

  const toggleSection = (title: string) => {
    setExpandedSections((prev) => ({ ...prev, [title]: !prev[title] }));
  };

  const toggleItem = (href: string) => {
    setExpandedItems((prev) => ({ ...prev, [href]: !prev[href] }));
  };

  // Close sidebar on mobile when clicking a link
  const handleLinkClick = () => {
    if (onClose) {
      onClose();
    }
  };

  // Prevent body scroll when mobile menu is open
  useEffect(() => {
    if (isOpen && onClose) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [isOpen, onClose]);

  // Recursive component to render link with potential children
  const renderLink = (link: DocLink, depth: number = 0) => {
    const isActive = pathname === link.href;
    const hasChildren = link.children && link.children.length > 0;
    const isExpanded = expandedItems[link.href];

    return (
      <li key={link.href}>
        <div className="flex items-center">
          {hasChildren && (
            <button
              onClick={() => toggleItem(link.href)}
              className="p-1 hover:bg-[var(--surface)] rounded transition-colors touch-manipulation"
              aria-label={isExpanded ? "Collapse" : "Expand"}
            >
              {isExpanded ? (
                <FiChevronDown className="w-3 h-3 text-[var(--text-secondary)]" />
              ) : (
                <FiChevronRight className="w-3 h-3 text-[var(--text-secondary)]" />
              )}
            </button>
          )}
          <Link
            href={link.href}
            onClick={handleLinkClick}
            className={`flex-1 block px-3 py-2 rounded text-sm transition-colors touch-manipulation ${
              hasChildren ? "" : depth > 0 ? "ml-4" : ""
            } ${
              isActive
                ? "bg-[var(--accent)]/10 text-[var(--accent)] font-medium border-l-2 border-[var(--accent)]"
                : "text-[var(--text-secondary)] hover:text-[var(--accent)] hover:bg-[var(--border)]"
            }`}
          >
            {link.title}
          </Link>
        </div>

        {hasChildren && isExpanded && (
          <ul className="space-y-1 ml-6 mt-1">
            {link.children!
              .sort((a, b) => (a.order ?? 999) - (b.order ?? 999))
              .map((child) => renderLink(child, depth + 1))}
          </ul>
        )}
      </li>
    );
  };

  const sidebarContent = (
    <div className="p-6 space-y-6 pb-12">
      {sections.map((section) => {
        const isExpanded = expandedSections[section.title];

        return (
          <div key={section.title}>
            <button
              onClick={() => toggleSection(section.title)}
              className="flex items-center gap-2 w-full text-left font-semibold text-[var(--text-primary)] hover:text-[var(--accent)] transition-colors mb-2 touch-manipulation"
            >
              {isExpanded ? (
                <FiChevronDown className="w-4 h-4" />
              ) : (
                <FiChevronRight className="w-4 h-4" />
              )}
              {section.title}
            </button>

            {isExpanded && (
              <ul className="space-y-1 ml-6">
                {section.links
                  .sort((a, b) => (a.order ?? 999) - (b.order ?? 999))
                  .map((link) => renderLink(link, 0))}
              </ul>
            )}
          </div>
        );
      })}
    </div>
  );

  // Desktop sidebar
  if (!onClose) {
    return (
      <aside className="hidden lg:block w-64 border-r border-[var(--border)] bg-[var(--surface)] h-[calc(100vh-4rem)] sticky top-16 overflow-y-auto">
        {sidebarContent}
      </aside>
    );
  }

  // Mobile sidebar (drawer)
  return (
    <>
      {/* Backdrop */}
      {isOpen && (
        <div
          className="fixed inset-0 bg-black/50 z-40 lg:hidden backdrop-blur-sm"
          onClick={onClose}
        />
      )}

      {/* Drawer */}
      <aside
        className={`fixed top-0 left-0 bottom-0 w-80 max-w-[85vw] bg-[var(--background)] border-r border-[var(--border)] z-50 lg:hidden transform transition-transform duration-300 ease-in-out overflow-y-auto ${
          isOpen ? 'translate-x-0' : '-translate-x-full'
        }`}
      >
        {/* Close button */}
        <div className="sticky top-0 bg-[var(--background)] border-b border-[var(--border)] p-4 flex items-center justify-between">
          <span className="font-semibold text-[var(--text-primary)]">Documentation</span>
          <button
            onClick={onClose}
            className="p-2 hover:bg-[var(--surface)] rounded-md transition-colors touch-manipulation"
            aria-label="Close menu"
          >
            <FiX className="w-5 h-5" />
          </button>
        </div>
        {sidebarContent}
      </aside>
    </>
  );
}
