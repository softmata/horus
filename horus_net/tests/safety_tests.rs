#![allow(dead_code)]
//! Safety layer integration tests — import rejection, heartbeat, safe-state.

use std::sync::Arc;
use std::time::Duration;

use horus_net::config::{ImportConfig, NetConfig};
use horus_net::guard::{ExportMode, ImportExportGuard, ImportMode};
use horus_net::heartbeat::{LinkLostAction, SafetyHeartbeat};
use horus_net::registry::{TopicRegistry, TopicRole};
use horus_net::wire::topic_hash;

// ─── Import Guard Tests ─────────────────────────────────────────────────────

#[test]
fn import_deny_rejects_remote_cmd_vel() {
    let guard = ImportExportGuard::new(ImportMode::Deny, ExportMode::All, None);
    assert!(!guard.allow_import("cmd_vel"));
    assert!(!guard.allow_import("estop"));
    assert!(!guard.allow_import("anything"));
}

#[test]
fn import_auto_allows_sub_only_topics() {
    let reg = Arc::new(TopicRegistry::new());
    reg.register(
        "cmd_vel",
        topic_hash("cmd_vel"),
        16,
        TopicRole::Subscriber,
        true,
    );
    reg.register("imu", topic_hash("imu"), 64, TopicRole::Publisher, true);

    let guard = ImportExportGuard::new_default(reg);

    // cmd_vel: we subscribe only → allow import
    assert!(guard.allow_import("cmd_vel"));
    // imu: we publish → deny import (prevents conflict)
    assert!(!guard.allow_import("imu"));
}

#[test]
fn import_auto_denies_when_we_also_publish() {
    let reg = Arc::new(TopicRegistry::new());
    // Both publish and subscribe to cmd_vel
    reg.register("cmd_vel", topic_hash("cmd_vel"), 16, TopicRole::Both, true);

    let guard = ImportExportGuard::new_default(reg);
    // We publish cmd_vel → deny import (prevents remote command overwriting ours)
    assert!(!guard.allow_import("cmd_vel"));
}

#[test]
fn import_explicit_list_only_accepts_listed() {
    let guard = ImportExportGuard::new(
        ImportMode::AllowList(vec!["cmd_vel".into(), "estop".into()]),
        ExportMode::All,
        None,
    );
    assert!(guard.allow_import("cmd_vel"));
    assert!(guard.allow_import("estop"));
    assert!(!guard.allow_import("imu"));
    assert!(!guard.allow_import("camera.rgb"));
}

// ─── Export Guard Tests ─────────────────────────────────────────────────────

#[test]
fn export_denylist_blocks_camera_and_debug() {
    let guard = ImportExportGuard::new(
        ImportMode::Deny,
        ExportMode::DenyList(vec![
            "camera.*".into(),
            "debug.*".into(),
            "internal.*".into(),
        ]),
        None,
    );
    assert!(guard.allow_export("imu"));
    assert!(guard.allow_export("odom"));
    assert!(!guard.allow_export("camera.rgb"));
    assert!(!guard.allow_export("camera.depth"));
    assert!(!guard.allow_export("debug.profiling"));
    assert!(!guard.allow_export("internal.state"));
}

// ─── Heartbeat Timeout Tests ────────────────────────────────────────────────

#[test]
fn heartbeat_detects_link_loss_within_200ms() {
    use horus_net::heartbeat::tests_support::MockTransport;

    let mut hb = SafetyHeartbeat::with_config(
        [0xAA; 16],
        Duration::from_millis(10), // 10ms interval for fast test
        3,                         // 3 missed
        LinkLostAction::SafeState,
    );
    let peer_id = [1u8; 16];
    hb.add_peer(peer_id, "192.168.1.10:9100".parse().unwrap());

    // Wait > 3 * 10ms = 30ms
    std::thread::sleep(Duration::from_millis(60));

    let transport = MockTransport::new();
    let mut seq = 0u32;
    let lost = hb.tick(&transport, 0, &mut seq);

    assert_eq!(lost.len(), 1);
    assert_eq!(lost[0].0, peer_id);
    assert_eq!(lost[0].1, LinkLostAction::SafeState);
    assert!(!hb.is_link_alive(&peer_id));
}

#[test]
fn heartbeat_link_restored_on_receive() {
    use horus_net::heartbeat::tests_support::MockTransport;

    let mut hb = SafetyHeartbeat::with_config(
        [0xAA; 16],
        Duration::from_millis(10),
        3,
        LinkLostAction::Warn,
    );
    let peer_id = [1u8; 16];
    hb.add_peer(peer_id, "192.168.1.10:9100".parse().unwrap());

    // Timeout
    std::thread::sleep(Duration::from_millis(60));
    let transport = MockTransport::new();
    let mut seq = 0u32;
    hb.tick(&transport, 0, &mut seq);
    assert!(!hb.is_link_alive(&peer_id));

    // Receive → restore
    hb.on_received(&peer_id);
    assert!(hb.is_link_alive(&peer_id));
}

// ─── Config Roundtrip Tests ─────────────────────────────────────────────────

#[test]
fn config_defaults_match_blueprint() {
    let config = NetConfig::test_config(9100);
    assert!(config.enabled);
    assert_eq!(config.port, 9100);
    assert!(matches!(config.import, ImportConfig::Auto));
    assert!(config.deny_export.is_empty());
    assert_eq!(config.safety.heartbeat_ms, 50);
    assert_eq!(config.safety.missed_threshold, 3);
    assert_eq!(config.safety.on_link_lost, "warn");
    assert!(config.optimizers.is_empty());
}

#[test]
fn config_disabled_via_flag() {
    let mut config = NetConfig::test_config(0);
    config.enabled = false;
    assert!(horus_net::start_replicator(config).is_none());
}
