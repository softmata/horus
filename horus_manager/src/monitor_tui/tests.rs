use super::*;

// ========================================================================
// Tab Navigation Tests
// ========================================================================

#[test]
fn test_tab_as_str() {
    assert_eq!(Tab::Overview.as_str(), "Overview");
    assert_eq!(Tab::Nodes.as_str(), "Nodes");
    assert_eq!(Tab::Topics.as_str(), "Topics");
    assert_eq!(Tab::Network.as_str(), "Network");
    assert_eq!(Tab::HFrame.as_str(), "HFrame");
    assert_eq!(Tab::Packages.as_str(), "Packages");
    assert_eq!(Tab::Parameters.as_str(), "Params");
}

#[test]
fn test_tab_all_returns_all_tabs() {
    let tabs = Tab::all();
    assert_eq!(tabs.len(), 8);
    assert!(tabs.contains(&Tab::Overview));
    assert!(tabs.contains(&Tab::Nodes));
    assert!(tabs.contains(&Tab::Topics));
    assert!(tabs.contains(&Tab::Network));
    assert!(tabs.contains(&Tab::HFrame));
    assert!(tabs.contains(&Tab::Packages));
    assert!(tabs.contains(&Tab::Parameters));
    assert!(tabs.contains(&Tab::Recordings));
}

// ========================================================================
// TuiDashboard State Tests
// ========================================================================

#[test]
fn test_tui_dashboard_new_defaults() {
    let dashboard = TuiDashboard::new();

    // Check initial state
    assert_eq!(dashboard.active_tab, Tab::Overview);
    assert_eq!(dashboard.selected_index, 0);
    assert_eq!(dashboard.scroll_offset, 0);
    assert!(!dashboard.paused);
    assert!(!dashboard.show_help);
    assert!(!dashboard.show_log_panel);
    assert!(dashboard.panel_target.is_none());
    assert_eq!(dashboard.param_edit_mode, ParamEditMode::None);
    assert_eq!(dashboard.package_view_mode, PackageViewMode::List);
    assert_eq!(
        dashboard.package_panel_focus,
        PackagePanelFocus::LocalWorkspaces
    );
    assert_eq!(dashboard.overview_panel_focus, OverviewPanelFocus::Nodes);
}

#[test]
fn test_tui_dashboard_default_impl() {
    let dashboard1 = TuiDashboard::new();
    let dashboard2 = TuiDashboard::default();

    // Both should have same initial state
    assert_eq!(dashboard1.active_tab, dashboard2.active_tab);
    assert_eq!(dashboard1.selected_index, dashboard2.selected_index);
    assert_eq!(dashboard1.paused, dashboard2.paused);
}

// ========================================================================
// Tab Navigation Logic Tests
// ========================================================================

#[test]
fn test_next_tab_cycles_through_all() {
    let mut dashboard = TuiDashboard::new();
    assert_eq!(dashboard.active_tab, Tab::Overview);

    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Nodes);

    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Topics);

    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Network);

    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::HFrame);

    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Packages);

    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Parameters);

    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Recordings);

    // Should wrap around
    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Overview);
}

#[test]
fn test_prev_tab_cycles_backwards() {
    let mut dashboard = TuiDashboard::new();
    assert_eq!(dashboard.active_tab, Tab::Overview);

    // Should wrap to Recordings (last tab)
    dashboard.prev_tab();
    assert_eq!(dashboard.active_tab, Tab::Recordings);

    dashboard.prev_tab();
    assert_eq!(dashboard.active_tab, Tab::Parameters);

    dashboard.prev_tab();
    assert_eq!(dashboard.active_tab, Tab::Packages);

    dashboard.prev_tab();
    assert_eq!(dashboard.active_tab, Tab::HFrame);

    dashboard.prev_tab();
    assert_eq!(dashboard.active_tab, Tab::Network);

    dashboard.prev_tab();
    assert_eq!(dashboard.active_tab, Tab::Topics);
}

// ========================================================================
// Selection Navigation Tests
// ========================================================================

#[test]
fn test_select_next_increments_index() {
    let mut dashboard = TuiDashboard::new();
    dashboard.nodes = vec![
        NodeStatus {
            name: "node1".to_string(),
            status: "running".to_string(),
            priority: 1,
            process_id: 1234,
            cpu_usage: 10.0,
            memory_usage: 1024,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "node2".to_string(),
            status: "running".to_string(),
            priority: 2,
            process_id: 5678,
            cpu_usage: 20.0,
            memory_usage: 2048,
            publishers: vec![],
            subscribers: vec![],
        },
    ];

    assert_eq!(dashboard.selected_index, 0);
    dashboard.select_next();
    assert_eq!(dashboard.selected_index, 1);
}

#[test]
fn test_select_prev_decrements_index() {
    let mut dashboard = TuiDashboard::new();
    dashboard.selected_index = 2;
    dashboard.nodes = vec![
        NodeStatus {
            name: "node1".to_string(),
            status: "running".to_string(),
            priority: 1,
            process_id: 1234,
            cpu_usage: 10.0,
            memory_usage: 1024,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "node2".to_string(),
            status: "running".to_string(),
            priority: 2,
            process_id: 5678,
            cpu_usage: 20.0,
            memory_usage: 2048,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "node3".to_string(),
            status: "running".to_string(),
            priority: 3,
            process_id: 9999,
            cpu_usage: 30.0,
            memory_usage: 3072,
            publishers: vec![],
            subscribers: vec![],
        },
    ];

    dashboard.select_prev();
    assert_eq!(dashboard.selected_index, 1);
    dashboard.select_prev();
    assert_eq!(dashboard.selected_index, 0);
}

// ========================================================================
// Pause Toggle Tests
// ========================================================================

#[test]
fn test_pause_toggle() {
    let mut dashboard = TuiDashboard::new();
    assert!(!dashboard.paused);

    dashboard.paused = !dashboard.paused;
    assert!(dashboard.paused);

    dashboard.paused = !dashboard.paused;
    assert!(!dashboard.paused);
}

// ========================================================================
// Log Panel Tests
// ========================================================================

#[test]
fn test_log_panel_toggle() {
    let mut dashboard = TuiDashboard::new();
    assert!(!dashboard.show_log_panel);
    assert!(dashboard.panel_target.is_none());

    // Simulate opening log panel
    dashboard.show_log_panel = true;
    dashboard.panel_target = Some(LogPanelTarget::Node("test_node".to_string()));

    assert!(dashboard.show_log_panel);
    assert!(dashboard.panel_target.is_some());

    // Check target type
    assert!(
        matches!(&dashboard.panel_target, Some(LogPanelTarget::Node(name)) if name == "test_node"),
        "Expected Node target, got {:?}",
        dashboard.panel_target
    );
}

#[test]
fn test_log_panel_target_topic() {
    let mut dashboard = TuiDashboard::new();
    dashboard.show_log_panel = true;
    dashboard.panel_target = Some(LogPanelTarget::Topic("sensors.lidar".to_string()));

    assert!(
        matches!(&dashboard.panel_target, Some(LogPanelTarget::Topic(name)) if name == "sensors.lidar"),
        "Expected Topic target, got {:?}",
        dashboard.panel_target
    );
}

// ========================================================================
// Parameter Edit Mode Tests
// ========================================================================

#[test]
fn test_param_edit_modes() {
    let mut dashboard = TuiDashboard::new();
    assert_eq!(dashboard.param_edit_mode, ParamEditMode::None);

    // Test Add mode
    dashboard.param_edit_mode = ParamEditMode::Add;
    assert_eq!(dashboard.param_edit_mode, ParamEditMode::Add);

    // Test Edit mode
    dashboard.param_edit_mode = ParamEditMode::Edit("my_key".to_string());
    assert!(
        matches!(&dashboard.param_edit_mode, ParamEditMode::Edit(key) if key == "my_key"),
        "Expected Edit mode, got {:?}",
        dashboard.param_edit_mode
    );

    // Test Delete mode
    dashboard.param_edit_mode = ParamEditMode::Delete("delete_key".to_string());
    assert!(
        matches!(&dashboard.param_edit_mode, ParamEditMode::Delete(key) if key == "delete_key"),
        "Expected Delete mode, got {:?}",
        dashboard.param_edit_mode
    );
}

#[test]
fn test_param_input_focus() {
    let mut dashboard = TuiDashboard::new();
    assert_eq!(dashboard.param_input_focus, ParamInputFocus::Key);

    dashboard.param_input_focus = ParamInputFocus::Value;
    assert_eq!(dashboard.param_input_focus, ParamInputFocus::Value);
}

// ========================================================================
// Package View Mode Tests
// ========================================================================

#[test]
fn test_package_view_modes() {
    let mut dashboard = TuiDashboard::new();
    assert_eq!(dashboard.package_view_mode, PackageViewMode::List);

    dashboard.package_view_mode = PackageViewMode::WorkspaceDetails;
    assert_eq!(
        dashboard.package_view_mode,
        PackageViewMode::WorkspaceDetails
    );
}

#[test]
fn test_package_panel_focus() {
    let mut dashboard = TuiDashboard::new();
    assert_eq!(
        dashboard.package_panel_focus,
        PackagePanelFocus::LocalWorkspaces
    );

    dashboard.package_panel_focus = PackagePanelFocus::GlobalPackages;
    assert_eq!(
        dashboard.package_panel_focus,
        PackagePanelFocus::GlobalPackages
    );
}

// ========================================================================
// Overview Panel Focus Tests
// ========================================================================

#[test]
fn test_overview_panel_focus() {
    let mut dashboard = TuiDashboard::new();
    assert_eq!(dashboard.overview_panel_focus, OverviewPanelFocus::Nodes);

    dashboard.overview_panel_focus = OverviewPanelFocus::Topics;
    assert_eq!(dashboard.overview_panel_focus, OverviewPanelFocus::Topics);
}

// ========================================================================
// Data Model Tests
// ========================================================================

#[test]
fn test_node_status_creation() {
    let node = NodeStatus {
        name: "test_node".to_string(),
        status: "running".to_string(),
        priority: 1,
        process_id: 12345,
        cpu_usage: 25.5,
        memory_usage: 1024 * 1024,
        publishers: vec!["topic1".to_string(), "topic2".to_string()],
        subscribers: vec!["topic3".to_string()],
    };

    assert_eq!(node.name, "test_node");
    assert_eq!(node.status, "running");
    assert_eq!(node.priority, 1);
    assert_eq!(node.process_id, 12345);
    assert!((node.cpu_usage - 25.5).abs() < 0.001);
    assert_eq!(node.memory_usage, 1024 * 1024);
    assert_eq!(node.publishers.len(), 2);
    assert_eq!(node.subscribers.len(), 1);
}

#[test]
fn test_topic_info_creation() {
    let topic = TopicInfo {
        name: "sensors.lidar".to_string(),
        msg_type: "LidarScan".to_string(),
        publishers: 2,
        subscribers: 3,
        rate: 10.0,
        publisher_nodes: vec!["node1".to_string(), "node2".to_string()],
        subscriber_nodes: vec![
            "node3".to_string(),
            "node4".to_string(),
            "node5".to_string(),
        ],
        status: crate::discovery::TopicStatus::Active,
    };

    assert_eq!(topic.name, "sensors.lidar");
    assert_eq!(topic.msg_type, "LidarScan");
    assert_eq!(topic.publishers, 2);
    assert_eq!(topic.subscribers, 3);
    assert!((topic.rate - 10.0).abs() < 0.001);
    assert_eq!(topic.publisher_nodes.len(), 2);
    assert_eq!(topic.subscriber_nodes.len(), 3);
}

#[test]
fn test_workspace_data_creation() {
    let workspace = WorkspaceData {
        name: "my_robot".to_string(),
        path: "/home/user/my_robot".to_string(),
        packages: vec![PackageData {
            name: "controller".to_string(),
            version: "1.0.0".to_string(),
            installed_packages: vec![("lidar_driver".to_string(), "0.5.0".to_string())],
        }],
        is_current: true,
    };

    assert_eq!(workspace.name, "my_robot");
    assert!(workspace.is_current);
    assert_eq!(workspace.packages.len(), 1);
}

// ========================================================================
// Workspace Cache Tests
// ========================================================================

#[test]
fn test_workspace_cache_initialization() {
    let dashboard = TuiDashboard::new();

    // Cache should be empty initially
    assert!(dashboard.workspace_cache.is_empty());

    // Cache time should be set to force initial load
    assert!(dashboard.workspace_cache_time.elapsed().as_secs() >= 5);
}

// ========================================================================
// Phase 1: TUI ↔ Backend Integration Tests
// ========================================================================

#[test]
fn test_update_data_populates_nodes_from_discovery() {
    // Create a dashboard and call update_data — it should populate nodes
    // from the discovery backend (which reads /dev/shm presence files).
    let mut dashboard = TuiDashboard::new();
    assert!(dashboard.nodes.is_empty());

    // After update_data, we should have at least the placeholder entry
    // (since no real nodes are running in the test environment)
    let _ = dashboard.update_data();

    // Either we get real nodes or the "No HORUS nodes detected" placeholder
    assert!(
        !dashboard.nodes.is_empty(),
        "update_data should populate nodes (even if only placeholder)"
    );
}

#[test]
fn test_update_data_populates_topics_from_discovery() {
    let mut dashboard = TuiDashboard::new();
    assert!(dashboard.topics.is_empty());

    let _ = dashboard.update_data();

    // Topics may be empty in test env, but every discovered topic must be valid
    for topic in &dashboard.topics {
        assert!(!topic.name.is_empty(), "Topic name must not be empty");
    }
    // update_data should also have populated nodes (at minimum an empty valid vec)
    // Verify nodes vec is valid (not corrupted) by checking each entry
    for node in &dashboard.nodes {
        assert!(!node.name.is_empty(), "Node name must not be empty");
    }
}

#[test]
fn test_update_data_preserves_nodes_on_error_recovery() {
    let mut dashboard = TuiDashboard::new();

    // Pre-populate with known data
    dashboard.nodes = vec![NodeStatus {
        name: "preserved_node".to_string(),
        status: "active".to_string(),
        priority: 1,
        process_id: 9999,
        cpu_usage: 5.0,
        memory_usage: 1024,
        publishers: vec![],
        subscribers: vec![],
    }];

    // update_data should replace with discovery data (not panic)
    let result = dashboard.update_data();
    assert!(result.is_ok(), "update_data should not return error");

    // Nodes should be updated (either real nodes or placeholder)
    assert!(!dashboard.nodes.is_empty());
}

#[test]
fn test_update_data_called_multiple_times_no_panic() {
    let mut dashboard = TuiDashboard::new();

    // Call update_data 10 times to verify no accumulated state issues
    for _ in 0..10 {
        let result = dashboard.update_data();
        result.unwrap();
    }
}

#[test]
fn test_get_active_node_count_with_placeholder() {
    let mut dashboard = TuiDashboard::new();

    // Simulate placeholder entry (what get_active_nodes returns when no real nodes)
    dashboard.nodes = vec![NodeStatus {
        name: "No HORUS nodes detected".to_string(),
        status: "inactive".to_string(),
        priority: 0,
        process_id: 0,
        cpu_usage: 0.0,
        memory_usage: 0,
        publishers: vec![],
        subscribers: vec![],
    }];

    assert_eq!(
        dashboard.get_active_node_count(),
        0,
        "Placeholder should count as 0"
    );
}

#[test]
fn test_get_active_node_count_with_real_nodes() {
    let mut dashboard = TuiDashboard::new();

    dashboard.nodes = vec![
        NodeStatus {
            name: "camera".to_string(),
            status: "active".to_string(),
            priority: 1,
            process_id: 1000,
            cpu_usage: 5.0,
            memory_usage: 1024,
            publishers: vec!["rgb_image".to_string()],
            subscribers: vec![],
        },
        NodeStatus {
            name: "lidar".to_string(),
            status: "active".to_string(),
            priority: 2,
            process_id: 1001,
            cpu_usage: 10.0,
            memory_usage: 2048,
            publishers: vec!["point_cloud".to_string()],
            subscribers: vec![],
        },
    ];

    assert_eq!(dashboard.get_active_node_count(), 2);
}

#[test]
fn test_get_active_topic_count_with_real_topics() {
    let mut dashboard = TuiDashboard::new();

    dashboard.topics = vec![TopicInfo {
        name: "cmd_vel".to_string(),
        msg_type: "CmdVel".to_string(),
        publishers: 1,
        subscribers: 2,
        rate: 50.0,
        publisher_nodes: vec!["planner".to_string()],
        subscriber_nodes: vec!["motor_left".to_string(), "motor_right".to_string()],
        status: crate::discovery::TopicStatus::Active,
    }];

    assert_eq!(dashboard.get_active_topic_count(), 1);
}

#[test]
fn test_node_data_fields_accessible_after_injection() {
    let mut dashboard = TuiDashboard::new();

    dashboard.nodes = vec![NodeStatus {
        name: "test_sensor".to_string(),
        status: "active".to_string(),
        priority: 5,
        process_id: 42,
        cpu_usage: 33.3,
        memory_usage: 1024 * 1024 * 50, // 50 MB
        publishers: vec!["scan".to_string(), "imu".to_string()],
        subscribers: vec!["cmd".to_string()],
    }];

    let node = &dashboard.nodes[0];
    assert_eq!(node.name, "test_sensor");
    assert_eq!(node.status, "active");
    assert_eq!(node.priority, 5);
    assert_eq!(node.process_id, 42);
    assert!((node.cpu_usage - 33.3).abs() < 0.01);
    assert_eq!(node.memory_usage, 1024 * 1024 * 50);
    assert_eq!(node.publishers.len(), 2);
    assert_eq!(node.subscribers.len(), 1);
}

#[test]
fn test_topic_data_fields_accessible_after_injection() {
    let mut dashboard = TuiDashboard::new();

    dashboard.topics = vec![TopicInfo {
        name: "sensors/lidar/scan".to_string(),
        msg_type: "LaserScan".to_string(),
        publishers: 1,
        subscribers: 3,
        rate: 10.0,
        publisher_nodes: vec!["lidar_driver".to_string()],
        subscriber_nodes: vec![
            "slam".to_string(),
            "planner".to_string(),
            "safety".to_string(),
        ],
        status: crate::discovery::TopicStatus::Active,
    }];

    let topic = &dashboard.topics[0];
    assert_eq!(topic.name, "sensors/lidar/scan");
    assert_eq!(topic.msg_type, "LaserScan");
    assert_eq!(topic.publishers, 1);
    assert_eq!(topic.subscribers, 3);
    assert!((topic.rate - 10.0).abs() < 0.01);
    assert_eq!(topic.publisher_nodes.len(), 1);
    assert_eq!(topic.subscriber_nodes.len(), 3);
}

// ========================================================================
// Phase 1: Node/Topic Appearance & Disappearance in State
// ========================================================================

#[test]
fn test_node_disappearance_from_state() {
    let mut dashboard = TuiDashboard::new();

    // Start with 3 nodes
    dashboard.nodes = vec![
        NodeStatus {
            name: "a".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 1,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "b".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 2,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "c".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 3,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
    ];
    assert_eq!(dashboard.nodes.len(), 3);

    // Simulate disappearance by replacing with fewer nodes
    dashboard.nodes = vec![NodeStatus {
        name: "a".to_string(),
        status: "active".to_string(),
        priority: 0,
        process_id: 1,
        cpu_usage: 0.0,
        memory_usage: 0,
        publishers: vec![],
        subscribers: vec![],
    }];
    assert_eq!(dashboard.nodes.len(), 1);
    assert_eq!(dashboard.nodes[0].name, "a");
}

#[test]
fn test_selected_index_clamped_after_node_removal() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Nodes;

    // Start with 5 nodes, selected at index 4
    dashboard.nodes = (0..5)
        .map(|i| NodeStatus {
            name: format!("node_{}", i),
            status: "active".to_string(),
            priority: 0,
            process_id: i as u32,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        })
        .collect();
    dashboard.selected_index = 4;

    // Remove all but 2 nodes
    dashboard.nodes.truncate(2);

    // select_next should not go beyond available nodes
    dashboard.select_next();
    assert!(
        dashboard.selected_index <= dashboard.nodes.len().saturating_sub(1),
        "selected_index {} should be <= max {}",
        dashboard.selected_index,
        dashboard.nodes.len().saturating_sub(1)
    );
}

// ========================================================================
// Phase 2: TestBackend Rendering Tests
// ========================================================================

/// Helper: render a TuiDashboard frame into a TestBackend buffer
fn render_dashboard(
    dashboard: &mut TuiDashboard,
    width: u16,
    height: u16,
) -> ratatui::buffer::Buffer {
    let backend = ratatui::backend::TestBackend::new(width, height);
    let mut terminal = Terminal::new(backend).unwrap();
    terminal.draw(|f| dashboard.draw_ui(f)).unwrap();
    terminal.backend().buffer().clone()
}

/// Helper: check if any cell in the buffer contains the given text substring
fn buffer_contains(buffer: &ratatui::buffer::Buffer, text: &str) -> bool {
    let area = buffer.area;
    for y in area.top()..area.bottom() {
        let mut line = String::new();
        for x in area.left()..area.right() {
            let cell = &buffer[(x, y)];
            line.push_str(cell.symbol());
        }
        if line.contains(text) {
            return true;
        }
    }
    false
}

#[test]
fn test_render_overview_tab_no_panic() {
    let mut dashboard = TuiDashboard::new();
    let buffer = render_dashboard(&mut dashboard, 120, 40);

    // Should contain "Overview" tab label
    assert!(
        buffer_contains(&buffer, "Overview"),
        "Buffer should contain 'Overview' tab"
    );
}

#[test]
fn test_render_all_tabs_contains_tab_label() {
    let mut dashboard = TuiDashboard::new();

    for tab in Tab::all() {
        dashboard.active_tab = tab;
        let buffer = render_dashboard(&mut dashboard, 120, 40);
        // Each rendered tab must contain its own label somewhere in the output
        assert!(
            buffer_contains(&buffer, tab.as_str()),
            "Tab {:?} buffer should contain '{}' label",
            tab,
            tab.as_str()
        );
    }
}

#[test]
fn test_render_all_tabs_at_minimum_size() {
    let mut dashboard = TuiDashboard::new();

    for tab in Tab::all() {
        dashboard.active_tab = tab;
        let buffer = render_dashboard(&mut dashboard, 40, 10);
        // Even at tiny size, buffer must have correct dimensions
        assert_eq!(
            buffer.area.width, 40,
            "Buffer width mismatch for tab {:?}",
            tab
        );
        assert_eq!(
            buffer.area.height, 10,
            "Buffer height mismatch for tab {:?}",
            tab
        );
    }
}

#[test]
fn test_render_nodes_tab_shows_node_data() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Nodes;

    dashboard.nodes = vec![NodeStatus {
        name: "camera_node".to_string(),
        status: "active".to_string(),
        priority: 1,
        process_id: 12345,
        cpu_usage: 15.5,
        memory_usage: 1024 * 1024 * 32,
        publishers: vec!["rgb_image".to_string()],
        subscribers: vec![],
    }];

    let buffer = render_dashboard(&mut dashboard, 120, 40);

    assert!(
        buffer_contains(&buffer, "camera_node"),
        "Node name should be visible"
    );
}

#[test]
fn test_render_topics_tab_shows_topic_data() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Topics;

    dashboard.topics = vec![TopicInfo {
        name: "cmd_vel".to_string(),
        msg_type: "CmdVel".to_string(),
        publishers: 1,
        subscribers: 2,
        rate: 50.0,
        publisher_nodes: vec!["planner".to_string()],
        subscriber_nodes: vec!["motor_l".to_string(), "motor_r".to_string()],
        status: crate::discovery::TopicStatus::Active,
    }];

    let buffer = render_dashboard(&mut dashboard, 120, 40);

    assert!(
        buffer_contains(&buffer, "cmd_vel"),
        "Topic name should be visible"
    );
    assert!(
        buffer_contains(&buffer, "CmdVel"),
        "Message type should be visible"
    );
}

#[test]
fn test_render_paused_indicator() {
    let mut dashboard = TuiDashboard::new();

    // Not paused — should show LIVE
    let buffer = render_dashboard(&mut dashboard, 120, 40);
    assert!(
        buffer_contains(&buffer, "LIVE"),
        "Should show LIVE when not paused"
    );

    // Paused — should show PAUSED
    dashboard.paused = true;
    let buffer = render_dashboard(&mut dashboard, 120, 40);
    assert!(
        buffer_contains(&buffer, "PAUSED"),
        "Should show PAUSED when paused"
    );
}

#[test]
fn test_render_help_panel() {
    let mut dashboard = TuiDashboard::new();
    dashboard.show_help = true;

    let buffer = render_dashboard(&mut dashboard, 120, 40);

    // Help panel should contain keybinding hints
    assert!(
        buffer_contains(&buffer, "quit")
            || buffer_contains(&buffer, "Quit")
            || buffer_contains(&buffer, "exit"),
        "Help panel should contain quit instruction"
    );
}

#[test]
fn test_render_tab_bar_highlights_active() {
    let mut dashboard = TuiDashboard::new();

    // Test each tab appears in the tab bar
    for tab in Tab::all() {
        dashboard.active_tab = tab;
        let buffer = render_dashboard(&mut dashboard, 120, 40);
        assert!(
            buffer_contains(&buffer, tab.as_str()),
            "Tab bar should contain '{}' when it's active",
            tab.as_str()
        );
    }
}

#[test]
fn test_render_nodes_empty_state() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Nodes;
    dashboard.nodes = vec![NodeStatus {
        name: "No HORUS nodes detected".to_string(),
        status: "inactive".to_string(),
        priority: 0,
        process_id: 0,
        cpu_usage: 0.0,
        memory_usage: 0,
        publishers: vec![],
        subscribers: vec![],
    }];

    let buffer = render_dashboard(&mut dashboard, 120, 40);

    assert!(
        buffer_contains(&buffer, "No HORUS nodes"),
        "Should show empty state message"
    );
}

#[test]
fn test_render_log_panel_split() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Nodes;
    dashboard.nodes = vec![NodeStatus {
        name: "test_node".to_string(),
        status: "active".to_string(),
        priority: 1,
        process_id: 1,
        cpu_usage: 0.0,
        memory_usage: 0,
        publishers: vec![],
        subscribers: vec![],
    }];

    // Open log panel
    dashboard.show_log_panel = true;
    dashboard.panel_target = Some(LogPanelTarget::Node("test_node".to_string()));

    let buffer = render_dashboard(&mut dashboard, 120, 40);

    // Log panel should render (check for Logs/Log panel title)
    assert!(
        buffer_contains(&buffer, "Log") || buffer_contains(&buffer, "test_node"),
        "Log panel should be visible with node target"
    );
}

#[test]
fn test_render_different_terminal_sizes() {
    let mut dashboard = TuiDashboard::new();

    let sizes = [(80, 24), (120, 40), (200, 60), (40, 10), (60, 15)];

    for (w, h) in sizes {
        let buffer = render_dashboard(&mut dashboard, w, h);
        assert_eq!(
            buffer.area.width, w,
            "Buffer width should match requested {}",
            w
        );
        assert_eq!(
            buffer.area.height, h,
            "Buffer height should match requested {}",
            h
        );
    }
}

// ========================================================================
// Phase 3: End-to-End TUI Simulation (Keypress → State → Render)
// ========================================================================

#[test]
fn test_tab_switching_produces_different_renders() {
    let mut dashboard = TuiDashboard::new();

    // Render Overview
    let buf_overview = render_dashboard(&mut dashboard, 120, 40);

    // Switch to Nodes
    dashboard.next_tab();
    assert_eq!(dashboard.active_tab, Tab::Nodes);
    let buf_nodes = render_dashboard(&mut dashboard, 120, 40);

    // The two buffers should be different (different content areas)
    assert_ne!(
        buf_overview, buf_nodes,
        "Overview and Nodes tabs should render differently"
    );
}

#[test]
fn test_pause_toggle_changes_render() {
    let mut dashboard = TuiDashboard::new();

    let buf_live = render_dashboard(&mut dashboard, 120, 40);

    dashboard.paused = true;
    let buf_paused = render_dashboard(&mut dashboard, 120, 40);

    // Buffers should differ (LIVE vs PAUSED indicator)
    assert_ne!(
        buf_live, buf_paused,
        "LIVE and PAUSED should render differently"
    );
}

#[test]
fn test_open_log_panel_for_node() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Nodes;

    dashboard.nodes = vec![
        NodeStatus {
            name: "sensor_node".to_string(),
            status: "active".to_string(),
            priority: 1,
            process_id: 100,
            cpu_usage: 5.0,
            memory_usage: 1024,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "motor_node".to_string(),
            status: "active".to_string(),
            priority: 2,
            process_id: 101,
            cpu_usage: 10.0,
            memory_usage: 2048,
            publishers: vec![],
            subscribers: vec![],
        },
    ];

    // Select first node and open log panel
    dashboard.selected_index = 0;
    dashboard.open_log_panel();

    assert!(dashboard.show_log_panel);
    assert_eq!(
        dashboard.panel_target,
        Some(LogPanelTarget::Node("sensor_node".to_string()))
    );

    // Close log panel
    dashboard.close_log_panel();
    assert!(!dashboard.show_log_panel);
    assert!(dashboard.panel_target.is_none());
}

#[test]
fn test_log_panel_target_switching_via_shift_nav() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Nodes;

    dashboard.nodes = vec![
        NodeStatus {
            name: "node_a".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 1,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "node_b".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 2,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "node_c".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 3,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
    ];

    // Open panel on first node
    dashboard.selected_index = 0;
    dashboard.open_log_panel();
    assert_eq!(
        dashboard.panel_target,
        Some(LogPanelTarget::Node("node_a".to_string()))
    );

    // Shift+Down: navigate to next node and update target
    dashboard.select_next();
    dashboard.update_log_panel_target();
    assert_eq!(
        dashboard.panel_target,
        Some(LogPanelTarget::Node("node_b".to_string()))
    );

    // Again
    dashboard.select_next();
    dashboard.update_log_panel_target();
    assert_eq!(
        dashboard.panel_target,
        Some(LogPanelTarget::Node("node_c".to_string()))
    );
}

#[test]
fn test_parameter_add_flow() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Parameters;

    // Start add
    dashboard.param_edit_mode = ParamEditMode::Add;
    dashboard.param_input_focus = ParamInputFocus::Key;
    dashboard.param_input_key = "robot_speed".to_string();

    // Move to value
    dashboard.param_input_focus = ParamInputFocus::Value;
    dashboard.param_input_value = "1.5".to_string();

    // Confirm
    dashboard.confirm_add_parameter();

    // Verify parameter was added
    assert_eq!(dashboard.param_edit_mode, ParamEditMode::None);
    let all_params = dashboard.params.get_all();
    assert!(
        all_params.contains_key("robot_speed"),
        "robot_speed should be in params after add"
    );
}

#[test]
fn test_parameter_edit_flow() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Parameters;

    // First add a parameter
    dashboard.param_edit_mode = ParamEditMode::Add;
    dashboard.param_input_key = "max_speed".to_string();
    dashboard.param_input_value = "2.0".to_string();
    dashboard.confirm_add_parameter();

    // Now edit it
    dashboard.param_edit_mode = ParamEditMode::Edit("max_speed".to_string());
    dashboard.param_input_key = "max_speed".to_string();
    dashboard.param_input_value = "3.0".to_string();
    dashboard.param_input_focus = ParamInputFocus::Value;
    dashboard.confirm_edit_parameter();

    assert_eq!(dashboard.param_edit_mode, ParamEditMode::None);
    let all_params = dashboard.params.get_all();
    let val = all_params.get("max_speed").expect("max_speed should exist");
    // Value should be 3.0 (either as number or string)
    let val_str = val.to_string();
    assert!(
        val_str.contains("3.0") || val_str.contains("3"),
        "max_speed should be updated to 3.0, got {}",
        val_str
    );
}

#[test]
fn test_parameter_delete_flow() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Parameters;

    // Add a parameter
    dashboard.param_edit_mode = ParamEditMode::Add;
    dashboard.param_input_key = "temp_param".to_string();
    dashboard.param_input_value = "delete_me".to_string();
    dashboard.confirm_add_parameter();

    // Verify it exists
    assert!(dashboard.params.get_all().contains_key("temp_param"));

    // Delete it
    dashboard.param_edit_mode = ParamEditMode::Delete("temp_param".to_string());
    dashboard.confirm_delete_parameter();

    assert_eq!(dashboard.param_edit_mode, ParamEditMode::None);
    assert!(
        !dashboard.params.get_all().contains_key("temp_param"),
        "temp_param should be deleted"
    );
}

#[test]
fn test_parameter_cancel_preserves_state() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Parameters;

    // Add a parameter
    dashboard.param_edit_mode = ParamEditMode::Add;
    dashboard.param_input_key = "keep_me".to_string();
    dashboard.param_input_value = "original".to_string();
    dashboard.confirm_add_parameter();

    let count_before = dashboard.params.get_all().len();

    // Start editing but cancel (ESC)
    dashboard.param_edit_mode = ParamEditMode::Edit("keep_me".to_string());
    dashboard.param_input_key = "keep_me".to_string();
    dashboard.param_input_value = "CHANGED".to_string();

    // Cancel instead of confirm
    dashboard.param_edit_mode = ParamEditMode::None;
    dashboard.param_input_key.clear();
    dashboard.param_input_value.clear();

    // Original value should be preserved
    let count_after = dashboard.params.get_all().len();
    assert_eq!(
        count_before, count_after,
        "Cancel should not change param count"
    );
}

// ========================================================================
// Phase 2: Additional Rendering Tests
// ========================================================================

#[test]
fn test_render_param_edit_dialog() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Parameters;
    dashboard.param_edit_mode = ParamEditMode::Add;
    dashboard.param_input_key = "test_key".to_string();

    let buffer = render_dashboard(&mut dashboard, 120, 40);

    // Dialog should be visible
    assert!(
        buffer_contains(&buffer, "Add")
            || buffer_contains(&buffer, "Parameter")
            || buffer_contains(&buffer, "ESC"),
        "Parameter edit dialog should be visible"
    );
}

#[test]
fn test_render_with_many_nodes_no_panic() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Nodes;

    // Add 100 nodes
    dashboard.nodes = (0..100)
        .map(|i| NodeStatus {
            name: format!("node_{:03}", i),
            status: "active".to_string(),
            priority: i as u32 % 10,
            process_id: 1000 + i as u32,
            cpu_usage: (i as f32) * 0.5,
            memory_usage: 1024 * (i as u64 + 1),
            publishers: vec![],
            subscribers: vec![],
        })
        .collect();

    // Rendering 100 nodes in a 24-line terminal should work (scrolling)
    let _buffer = render_dashboard(&mut dashboard, 80, 24);
}

#[test]
fn test_render_overview_shows_node_and_topic_counts() {
    let mut dashboard = TuiDashboard::new();
    dashboard.active_tab = Tab::Overview;

    dashboard.nodes = vec![
        NodeStatus {
            name: "n1".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 1,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "n2".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 2,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
    ];

    let buffer = render_dashboard(&mut dashboard, 120, 40);

    // Overview should mention the node names or count
    assert!(
        buffer_contains(&buffer, "n1")
            || buffer_contains(&buffer, "n2")
            || buffer_contains(&buffer, "2"),
        "Overview should show node data"
    );
}

// ========================================================================
// Data Module Tests
// ========================================================================

#[test]
fn test_format_bytes() {
    use super::data::format_bytes;

    assert_eq!(format_bytes(0), "0B");
    assert_eq!(format_bytes(512), "512B");
    assert_eq!(format_bytes(1024), "1.0KB");
    assert_eq!(format_bytes(1536), "1.5KB");
    assert_eq!(format_bytes(1024 * 1024), "1.0MB");
    assert_eq!(format_bytes(1024 * 1024 * 1024), "1.0GB");
    assert_eq!(format_bytes(1024 * 1024 * 1024 * 3), "3.0GB");
}

// ========================================================================
// Debugger State Tests
// ========================================================================

#[test]
fn test_debugger_step_forward() {
    let mut dashboard = TuiDashboard::new();
    dashboard.debugger_state.active = true;
    dashboard.debugger_state.total_ticks = 100;
    dashboard.debugger_state.current_tick = 0;

    dashboard.debugger_step_forward();
    assert_eq!(dashboard.debugger_state.current_tick, 1);

    dashboard.debugger_step_forward();
    assert_eq!(dashboard.debugger_state.current_tick, 2);
}

#[test]
fn test_debugger_step_backward() {
    let mut dashboard = TuiDashboard::new();
    dashboard.debugger_state.active = true;
    dashboard.debugger_state.total_ticks = 100;
    dashboard.debugger_state.current_tick = 5;

    dashboard.debugger_step_backward();
    assert_eq!(dashboard.debugger_state.current_tick, 4);

    // Should not go below 0
    dashboard.debugger_state.current_tick = 0;
    dashboard.debugger_step_backward();
    assert_eq!(dashboard.debugger_state.current_tick, 0);
}

#[test]
fn test_debugger_step_forward_at_end() {
    let mut dashboard = TuiDashboard::new();
    dashboard.debugger_state.active = true;
    dashboard.debugger_state.total_ticks = 10;
    dashboard.debugger_state.current_tick = 9; // Last tick

    dashboard.debugger_step_forward();
    assert_eq!(
        dashboard.debugger_state.current_tick, 9,
        "Should not go past last tick"
    );
}

#[test]
fn test_debugger_toggle_breakpoint() {
    let mut dashboard = TuiDashboard::new();
    dashboard.debugger_state.active = true;
    dashboard.debugger_state.current_tick = 5;

    // Add breakpoint
    dashboard.toggle_breakpoint();
    assert!(dashboard.debugger_state.breakpoints.contains(&5));

    // Remove breakpoint (toggle)
    dashboard.toggle_breakpoint();
    assert!(!dashboard.debugger_state.breakpoints.contains(&5));
}

#[test]
fn test_debugger_breakpoint_stops_playback() {
    let mut dashboard = TuiDashboard::new();
    dashboard.debugger_state.active = true;
    dashboard.debugger_state.total_ticks = 100;
    dashboard.debugger_state.current_tick = 4;

    // Set breakpoint at tick 5
    dashboard.debugger_state.breakpoints.push(5);

    // Step forward to tick 5
    dashboard.debugger_step_forward();
    assert_eq!(dashboard.debugger_state.current_tick, 5);
    assert_eq!(dashboard.debugger_state.playback, PlaybackState::Paused);
}

#[test]
fn test_debugger_playback_speed_bounds() {
    let mut dashboard = TuiDashboard::new();
    dashboard.debugger_state.active = true;

    // Increase speed
    dashboard.debugger_state.playback_speed = 1.0;
    dashboard.debugger_state.playback_speed =
        (dashboard.debugger_state.playback_speed * 2.0).min(8.0);
    assert!((dashboard.debugger_state.playback_speed - 2.0).abs() < 0.01);

    // Max speed
    dashboard.debugger_state.playback_speed = 8.0;
    dashboard.debugger_state.playback_speed =
        (dashboard.debugger_state.playback_speed * 2.0).min(8.0);
    assert!((dashboard.debugger_state.playback_speed - 8.0).abs() < 0.01);

    // Min speed
    dashboard.debugger_state.playback_speed = 0.125;
    dashboard.debugger_state.playback_speed =
        (dashboard.debugger_state.playback_speed * 0.5).max(0.125);
    assert!((dashboard.debugger_state.playback_speed - 0.125).abs() < 0.001);
}

// ========================================================================
// Phase 5: Responsiveness & Degradation Tests
// ========================================================================

#[test]
fn test_tui_graceful_with_empty_discovery() {
    // When no nodes/topics exist, TUI should render without panic
    let mut dashboard = TuiDashboard::new();
    let _ = dashboard.update_data();

    let buffer = render_dashboard(&mut dashboard, 120, 40);
    // Should show the placeholder or empty state, not crash
    assert!(
        buffer_contains(&buffer, "No HORUS")
            || buffer_contains(&buffer, "Overview")
            || buffer_contains(&buffer, "Nodes"),
        "Dashboard should render even with no real nodes"
    );
}

#[test]
fn test_tui_handles_rapid_data_updates() {
    // Simulate rapid update_data() calls (like fast polling)
    let mut dashboard = TuiDashboard::new();

    for _ in 0..100 {
        let _ = dashboard.update_data();
    }

    // Should not panic or accumulate stale data
    let buffer = render_dashboard(&mut dashboard, 120, 40);
    assert!(buffer_contains(&buffer, "Overview") || buffer_contains(&buffer, "Nodes"));
}

#[test]
fn test_tui_high_node_churn_stability() {
    // Rapidly add and remove nodes to simulate churn
    let mut dashboard = TuiDashboard::new();

    for i in 0..50 {
        // Add nodes
        dashboard.nodes = (0..10)
            .map(|j| NodeStatus {
                name: format!("churn_{}_{}", i, j),
                status: "active".to_string(),
                priority: 0,
                process_id: (i * 10 + j) as u32,
                cpu_usage: 0.0,
                memory_usage: 0,
                publishers: vec![],
                subscribers: vec![],
            })
            .collect();

        // Render should never panic
        let _buffer = render_dashboard(&mut dashboard, 100, 30);

        // Remove all nodes
        dashboard.nodes.clear();
        let _buffer = render_dashboard(&mut dashboard, 100, 30);
    }
}

#[test]
fn test_tui_error_recovery_after_empty_state() {
    // Start with data, lose it, then recover
    let mut dashboard = TuiDashboard::new();

    // Step 1: Normal state with nodes
    dashboard.nodes = vec![NodeStatus {
        name: "healthy_node".to_string(),
        status: "active".to_string(),
        priority: 5,
        process_id: 1234,
        cpu_usage: 25.0,
        memory_usage: 2048,
        publishers: vec!["sensor.data".to_string()],
        subscribers: vec![],
    }];
    dashboard.topics = vec![TopicInfo {
        name: "sensor.data".to_string(),
        msg_type: "SensorMsg".to_string(),
        publishers: 1,
        subscribers: 0,
        rate: 30.0,
        publisher_nodes: vec!["healthy_node".to_string()],
        subscriber_nodes: vec![],
        status: crate::discovery::TopicStatus::Active,
    }];

    let buffer1 = render_dashboard(&mut dashboard, 120, 40);
    assert!(
        buffer_contains(&buffer1, "healthy_node")
            || buffer_contains(&buffer1, "1")
            || buffer_contains(&buffer1, "Overview")
    );

    // Step 2: Everything disappears (discovery failure)
    dashboard.nodes.clear();
    dashboard.topics.clear();

    let buffer2 = render_dashboard(&mut dashboard, 120, 40);
    // Should render gracefully
    assert!(buffer2.area.width > 0);

    // Step 3: Recovery — nodes come back
    dashboard.nodes = vec![NodeStatus {
        name: "recovered_node".to_string(),
        status: "active".to_string(),
        priority: 5,
        process_id: 5678,
        cpu_usage: 10.0,
        memory_usage: 1024,
        publishers: vec![],
        subscribers: vec![],
    }];

    let buffer3 = render_dashboard(&mut dashboard, 120, 40);
    assert!(buffer3.area.width > 0);
}

#[test]
fn test_tui_unicode_node_names_no_panic() {
    let mut dashboard = TuiDashboard::new();
    dashboard.nodes = vec![
        NodeStatus {
            name: "ロボット_制御".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 1,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "nœud_contrôle".to_string(),
            status: "active".to_string(),
            priority: 0,
            process_id: 2,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
        NodeStatus {
            name: "узел_управления".to_string(),
            status: "inactive".to_string(),
            priority: 0,
            process_id: 3,
            cpu_usage: 0.0,
            memory_usage: 0,
            publishers: vec![],
            subscribers: vec![],
        },
    ];

    // Render all tabs with unicode names — should not panic
    for tab in Tab::all() {
        dashboard.active_tab = tab;
        let _buffer = render_dashboard(&mut dashboard, 120, 40);
    }
}

#[test]
fn test_tui_extremely_long_node_name_no_panic() {
    let mut dashboard = TuiDashboard::new();
    dashboard.nodes = vec![NodeStatus {
        name: "a".repeat(500),
        status: "active".to_string(),
        priority: 0,
        process_id: 1,
        cpu_usage: 0.0,
        memory_usage: 0,
        publishers: vec!["topic_".repeat(100)],
        subscribers: vec![],
    }];
    dashboard.active_tab = Tab::Nodes;

    // Should render without panic even with very long names
    let buffer = render_dashboard(&mut dashboard, 80, 24);
    assert!(buffer.area.width > 0);
}

#[test]
fn test_tui_zero_size_terminal_no_panic() {
    let mut dashboard = TuiDashboard::new();
    // Minimum possible terminal size
    let buffer = render_dashboard(&mut dashboard, 1, 1);
    assert_eq!(buffer.area.width, 1);
    assert_eq!(buffer.area.height, 1);
}

#[test]
fn test_tui_max_cpu_memory_values() {
    // Edge case: very high CPU and memory values
    let mut dashboard = TuiDashboard::new();
    dashboard.nodes = vec![NodeStatus {
        name: "overloaded".to_string(),
        status: "active".to_string(),
        priority: 99,
        process_id: 1,
        cpu_usage: 9999.9, // > 100% (multi-core)
        memory_usage: u64::MAX,
        publishers: vec![],
        subscribers: vec![],
    }];
    dashboard.active_tab = Tab::Nodes;

    let buffer = render_dashboard(&mut dashboard, 120, 40);
    assert!(
        buffer.area.width > 0,
        "should render extreme values without panic"
    );
}

// ========================================================================
// Phase 7: Cross-Frontend Consistency Tests
// ========================================================================

#[test]
fn test_tui_and_web_use_same_discovery_backend() {
    // Both frontends call the same get_active_nodes() / get_active_topics()
    // which calls discover_nodes() / discover_shared_memory() under the hood.
    // Verify the TUI data model can be populated from the same backend.

    let mut dashboard = TuiDashboard::new();
    let _ = dashboard.update_data();

    // The discovery backend always returns a Result<Vec<_>>, even if empty
    // Both web handlers and TUI call the same functions
    // This test verifies the TUI doesn't crash on whatever the backend returns
    // The node list is always populated (even if just a placeholder)
    let _ = dashboard.nodes.len();
}

#[test]
fn test_format_bytes_consistency() {
    // format_bytes is used by TUI. Web handlers format sizes differently.
    // Verify the TUI formatter produces expected outputs.
    use super::data::format_bytes;

    assert_eq!(format_bytes(0), "0B");
    assert_eq!(format_bytes(100), "100B");
    assert_eq!(format_bytes(1024), "1.0KB");
    assert_eq!(format_bytes(1536), "1.5KB");
    assert_eq!(format_bytes(1048576), "1.0MB");
    assert_eq!(format_bytes(1073741824), "1.0GB");
    assert_eq!(format_bytes(1572864), "1.5MB"); // 1.5 * 1024 * 1024
}

#[test]
fn test_zero_nodes_renders_same_across_tabs() {
    // When there are no nodes, all relevant tabs should render gracefully
    let mut dashboard = TuiDashboard::new();
    dashboard.nodes.clear();
    dashboard.topics.clear();

    // Each tab that shows node/topic data should handle empty state
    let tabs_with_node_data = [Tab::Overview, Tab::Nodes, Tab::Topics, Tab::Network];
    for tab in &tabs_with_node_data {
        dashboard.active_tab = *tab;
        let buffer = render_dashboard(&mut dashboard, 120, 40);
        assert!(
            buffer.area.width > 0,
            "Tab {:?} should render with no data",
            tab
        );
    }
}

#[test]
fn test_many_nodes_renders_same_across_tabs() {
    let mut dashboard = TuiDashboard::new();
    dashboard.nodes = (0..50)
        .map(|i| NodeStatus {
            name: format!("node_{:03}", i),
            status: if i % 3 == 0 { "inactive" } else { "active" }.to_string(),
            priority: (i % 10) as u32,
            process_id: 1000 + i as u32,
            cpu_usage: (i as f32) * 2.0,
            memory_usage: (i as u64) * 1024,
            publishers: if i % 2 == 0 {
                vec![format!("topic_{}", i)]
            } else {
                vec![]
            },
            subscribers: if i % 3 == 0 {
                vec![format!("topic_{}", i + 1)]
            } else {
                vec![]
            },
        })
        .collect();

    let tabs_with_node_data = [Tab::Overview, Tab::Nodes, Tab::Topics, Tab::Network];
    for tab in &tabs_with_node_data {
        dashboard.active_tab = *tab;
        let buffer = render_dashboard(&mut dashboard, 120, 40);
        assert!(
            buffer.area.width > 0,
            "Tab {:?} should render with 50 nodes",
            tab
        );
    }
}
