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
    match &dashboard.panel_target {
        Some(LogPanelTarget::Node(name)) => assert_eq!(name, "test_node"),
        _ => panic!("Expected Node target"),
    }
}

#[test]
fn test_log_panel_target_topic() {
    let mut dashboard = TuiDashboard::new();
    dashboard.show_log_panel = true;
    dashboard.panel_target = Some(LogPanelTarget::Topic("sensors.lidar".to_string()));

    match &dashboard.panel_target {
        Some(LogPanelTarget::Topic(name)) => assert_eq!(name, "sensors.lidar"),
        _ => panic!("Expected Topic target"),
    }
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
    match &dashboard.param_edit_mode {
        ParamEditMode::Edit(key) => assert_eq!(key, "my_key"),
        _ => panic!("Expected Edit mode"),
    }

    // Test Delete mode
    dashboard.param_edit_mode = ParamEditMode::Delete("delete_key".to_string());
    match &dashboard.param_edit_mode {
        ParamEditMode::Delete(key) => assert_eq!(key, "delete_key"),
        _ => panic!("Expected Delete mode"),
    }
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
        dependencies: vec![DependencyData {
            name: "slam".to_string(),
            declared_version: "2.0.0".to_string(),
        }],
        is_current: true,
    };

    assert_eq!(workspace.name, "my_robot");
    assert!(workspace.is_current);
    assert_eq!(workspace.packages.len(), 1);
    assert_eq!(workspace.dependencies.len(), 1);
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
