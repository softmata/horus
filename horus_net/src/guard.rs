//! Import/export guard — controls what enters/leaves this machine.
//!
//! **Import default: deny all.** Remote data does NOT write to local SHM unless
//! explicitly allowed. A robot must opt-in to receiving remote commands.
//!
//! **Export default: all topics.** Sensors should be visible by default.
//!
//! See blueprint section 9.

use std::sync::Arc;

use crate::registry::TopicRegistry;

/// Import control mode.
#[derive(Debug, Clone)]
pub enum ImportMode {
    /// Deny all imports. No remote data enters this machine.
    Deny,
    /// Auto: import topics we subscribe to but DON'T publish.
    /// If we publish the same topic, deny (prevents command conflicts).
    Auto,
    /// Explicit allowlist: only these topic patterns are accepted.
    AllowList(Vec<String>),
}

/// Export control mode.
#[derive(Debug, Clone)]
pub enum ExportMode {
    /// Export all topics that have remote subscribers (default).
    All,
    /// Export only topics matching these patterns.
    AllowList(Vec<String>),
    /// Export all EXCEPT topics matching these patterns.
    DenyList(Vec<String>),
}

/// Import/export guard for network traffic.
pub struct ImportExportGuard {
    import_mode: ImportMode,
    export_mode: ExportMode,
    /// Reference to local registry (for "auto" mode — need to check if we publish).
    registry: Option<Arc<TopicRegistry>>,
}

impl ImportExportGuard {
    /// Create a guard with default settings (import=auto, export=all).
    pub fn new_default(registry: Arc<TopicRegistry>) -> Self {
        Self {
            import_mode: ImportMode::Auto,
            export_mode: ExportMode::All,
            registry: Some(registry),
        }
    }

    /// Create with explicit modes.
    pub fn new(
        import_mode: ImportMode,
        export_mode: ExportMode,
        registry: Option<Arc<TopicRegistry>>,
    ) -> Self {
        Self {
            import_mode,
            export_mode,
            registry,
        }
    }

    /// Check if a remote write to this topic should be allowed.
    pub fn allow_import(&self, topic_name: &str) -> bool {
        // System topics always bypass guards
        if crate::registry::is_system_topic(topic_name) {
            return true;
        }
        match &self.import_mode {
            ImportMode::Deny => false,
            ImportMode::Auto => {
                // Import a topic this process subscribes to and does not itself
                // publish.
                //
                // This distinction was unobservable for a long time: the
                // lifecycle hook only learns that a topic was *created*, and a
                // `Topic<T>` handle can both send and recv, so every topic
                // registered as `Both` — first denying 100% of imports (a robot
                // configured exactly as documented received nothing, silently),
                // then, when that was patched by treating `Both` as importable,
                // allowing a remote peer to overwrite commands this robot
                // produces itself.
                //
                // horus_core now reports the direction on a handle's first send
                // or first recv (`TopicLifecycleEvent::RoleObserved`), so the
                // roles here are real and the original intent holds again.
                match self.registry {
                    Some(ref reg) => match reg.get(topic_name).map(|e| e.role) {
                        Some(crate::registry::TopicRole::Subscriber) => true,
                        // We publish it: a remote write would fight our own.
                        Some(crate::registry::TopicRole::Publisher) => false,
                        Some(crate::registry::TopicRole::Both) => false,
                        // Not open here at all — this is how another robot's
                        // traffic stays out.
                        None => false,
                    },
                    None => false,
                }
            }
            ImportMode::AllowList(patterns) => glob_match_any(topic_name, patterns),
        }
    }

    /// Check if exporting this topic to the network should be allowed.
    pub fn allow_export(&self, topic_name: &str) -> bool {
        // System topics bypass the guard only when the operator has not asked
        // for anything narrower. They used to bypass it UNCONDITIONALLY — the
        // same shape as the import-side bug behind GHSA-3frr-c2j9-hhr7 — which
        // meant `_horus.logs` (every Error and Warning line this node emits) and
        // `_horus.presence` (its full node inventory) were exported to any host
        // that announced an interest, and an operator who put them in
        // `deny_export` was silently ignored.
        //
        // `ExportMode::All` is still the default and still exports them, so
        // zero-config LAN replication is unchanged; the difference is that
        // `deny_export = ["_horus.logs"]` now actually works.
        if crate::registry::is_system_topic(topic_name) {
            return match &self.export_mode {
                ExportMode::All => true,
                ExportMode::AllowList(patterns) => glob_match_any(topic_name, patterns),
                ExportMode::DenyList(patterns) => !glob_match_any(topic_name, patterns),
            };
        }
        match &self.export_mode {
            ExportMode::All => true,
            ExportMode::AllowList(patterns) => glob_match_any(topic_name, patterns),
            ExportMode::DenyList(patterns) => !glob_match_any(topic_name, patterns),
        }
    }
}

/// Simple glob matching: `*` matches any characters within a segment.
///
/// Supports:
/// - Exact match: "imu" matches "imu"
/// - Wildcard suffix: "camera.*" matches "camera.rgb", "camera.depth"
/// - Wildcard prefix: "*.scan" matches "front.scan", "rear.scan"
/// - Full wildcard: "*" matches everything
///
/// Exposed for config.rs topic_config() glob matching.
pub fn glob_match_topic(topic: &str, pattern: &str) -> bool {
    glob_match(topic, pattern)
}

fn glob_match(topic: &str, pattern: &str) -> bool {
    if pattern == "*" {
        return true;
    }
    if !pattern.contains('*') {
        return topic == pattern;
    }

    // Split on '*' and check if all parts appear in order
    let parts: Vec<&str> = pattern.split('*').collect();

    if parts.len() == 2 {
        // Single wildcard: prefix*suffix
        let prefix = parts[0];
        let suffix = parts[1];
        return topic.starts_with(prefix)
            && topic.ends_with(suffix)
            && topic.len() >= prefix.len() + suffix.len();
    }

    // Multiple wildcards: check parts appear in order
    let mut remaining = topic;
    for (i, part) in parts.iter().enumerate() {
        if part.is_empty() {
            continue;
        }
        if i == 0 {
            // First part must be a prefix
            if !remaining.starts_with(part) {
                return false;
            }
            remaining = &remaining[part.len()..];
        } else if i == parts.len() - 1 {
            // Last part must be a suffix
            if !remaining.ends_with(part) {
                return false;
            }
        } else {
            // Middle part must appear somewhere
            match remaining.find(part) {
                Some(pos) => remaining = &remaining[pos + part.len()..],
                None => return false,
            }
        }
    }
    true
}

fn glob_match_any(topic: &str, patterns: &[String]) -> bool {
    patterns.iter().any(|p| glob_match(topic, p))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::{TopicRegistry, TopicRole};
    use crate::wire::topic_hash;

    fn make_registry() -> Arc<TopicRegistry> {
        let reg = Arc::new(TopicRegistry::new());
        // We subscribe to cmd_vel (but don't publish it) — should allow import
        reg.register(
            "cmd_vel",
            topic_hash("cmd_vel"),
            16,
            TopicRole::Subscriber,
            true,
        );
        // We publish imu (and also subscribe) — should deny import
        reg.register("imu", topic_hash("imu"), 64, TopicRole::Both, true);
        // We publish odom only — import irrelevant
        reg.register("odom", topic_hash("odom"), 48, TopicRole::Publisher, true);
        reg
    }

    // ─── Import Tests ───────────────────────────────────────────────────

    #[test]
    fn import_deny_rejects_everything() {
        let guard = ImportExportGuard::new(ImportMode::Deny, ExportMode::All, None);
        assert!(!guard.allow_import("cmd_vel"));
        assert!(!guard.allow_import("imu"));
        assert!(!guard.allow_import("anything"));
    }

    #[test]
    fn import_auto_allows_sub_only() {
        let reg = make_registry();
        let guard = ImportExportGuard::new_default(reg);

        // cmd_vel: role known to be Subscriber → allow
        assert!(guard.allow_import("cmd_vel"));
        // imu: registered `Both` — we publish it as well as subscribe — so a
        // remote write would fight the value produced here.
        assert!(!guard.allow_import("imu"));
        // odom: we only publish → deny
        assert!(!guard.allow_import("odom"));
        // unknown: not in registry → deny
        assert!(!guard.allow_import("unknown"));
    }

    #[test]
    fn auto_imports_what_we_subscribe_to_and_not_what_we_publish() {
        // The regression this guards, in both of its forms.
        //
        // `start_replicator` installs a hook that used to register every topic
        // as `TopicRole::Both`, because a `Topic<T>` handle can both send and
        // recv and the hook only learned that a topic was *created*. With
        // `Both` treated as "we publish", `Auto` — the default — admitted
        // nothing: a robot configured exactly as documented received no remote
        // data at all, with no error anywhere. With `Both` treated as "we
        // subscribe", imports worked but a remote peer could overwrite the very
        // commands this robot produces.
        //
        // horus_core now reports direction on first send / first recv, so these
        // roles are what production actually assigns.
        let reg = std::sync::Arc::new(crate::registry::TopicRegistry::new());
        // Created, then first recv.
        reg.register(
            "sensor.imu",
            topic_hash("sensor.imu"),
            64,
            TopicRole::Subscriber,
            true,
        );
        // Created, then first send: merges to `Both`.
        reg.register(
            "cmd_vel",
            topic_hash("cmd_vel"),
            16,
            TopicRole::Subscriber,
            true,
        );
        reg.register(
            "cmd_vel",
            topic_hash("cmd_vel"),
            16,
            TopicRole::Publisher,
            true,
        );

        let guard = ImportExportGuard::new_default(reg);

        assert!(
            guard.allow_import("sensor.imu"),
            "a topic this process only receives on must accept remote data — \
             otherwise no remote data ever arrives"
        );
        assert!(
            !guard.allow_import("cmd_vel"),
            "a topic this process publishes must not accept remote writes — \
             they would fight the commands produced here"
        );
        // A topic this process has never opened is still not imported, which is
        // what keeps an unrelated robot's traffic out.
        assert!(!guard.allow_import("someone.elses.topic"));
    }

    #[test]
    fn import_allowlist() {
        let guard = ImportExportGuard::new(
            ImportMode::AllowList(vec!["cmd_vel".into(), "estop".into()]),
            ExportMode::All,
            None,
        );
        assert!(guard.allow_import("cmd_vel"));
        assert!(guard.allow_import("estop"));
        assert!(!guard.allow_import("imu"));
        assert!(!guard.allow_import("odom"));
    }

    #[test]
    fn import_allowlist_with_glob() {
        let guard = ImportExportGuard::new(
            ImportMode::AllowList(vec!["cmd_*".into(), "nav.*".into()]),
            ExportMode::All,
            None,
        );
        assert!(guard.allow_import("cmd_vel"));
        assert!(guard.allow_import("cmd_arm"));
        assert!(guard.allow_import("nav.goal"));
        assert!(guard.allow_import("nav.path"));
        assert!(!guard.allow_import("imu"));
    }

    // ─── Export Tests ───────────────────────────────────────────────────

    #[test]
    fn export_all_allows_everything() {
        let guard = ImportExportGuard::new(ImportMode::Deny, ExportMode::All, None);
        assert!(guard.allow_export("imu"));
        assert!(guard.allow_export("camera.rgb"));
        assert!(guard.allow_export("anything"));
    }

    #[test]
    fn export_denylist() {
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

    #[test]
    fn export_allowlist() {
        let guard = ImportExportGuard::new(
            ImportMode::Deny,
            ExportMode::AllowList(vec!["imu".into(), "odom".into(), "estop".into()]),
            None,
        );
        assert!(guard.allow_export("imu"));
        assert!(guard.allow_export("odom"));
        assert!(guard.allow_export("estop"));
        assert!(!guard.allow_export("camera.rgb"));
        assert!(!guard.allow_export("debug.log"));
    }

    // ─── Glob Tests ─────────────────────────────────────────────────────

    #[test]
    fn glob_exact_match() {
        assert!(glob_match("imu", "imu"));
        assert!(!glob_match("imu", "odom"));
    }

    #[test]
    fn glob_wildcard_suffix() {
        assert!(glob_match("camera.rgb", "camera.*"));
        assert!(glob_match("camera.depth", "camera.*"));
        assert!(!glob_match("lidar.scan", "camera.*"));
    }

    #[test]
    fn glob_wildcard_prefix() {
        assert!(glob_match("front.scan", "*.scan"));
        assert!(glob_match("rear.scan", "*.scan"));
        assert!(!glob_match("front.imu", "*.scan"));
    }

    #[test]
    fn glob_wildcard_all() {
        assert!(glob_match("anything", "*"));
        assert!(glob_match("camera.rgb.compressed", "*"));
    }

    #[test]
    fn glob_no_wildcard_exact() {
        assert!(glob_match("exact_topic", "exact_topic"));
        assert!(!glob_match("other", "exact_topic"));
    }

    #[test]
    fn glob_middle_wildcard() {
        assert!(glob_match("robot.front.scan", "robot.*.scan"));
        assert!(glob_match("robot.rear.scan", "robot.*.scan"));
        assert!(!glob_match("robot.front.imu", "robot.*.scan"));
    }

    #[test]
    fn system_topics_respect_an_explicit_deny_export() {
        // Regression guard: system topics used to bypass allow_export
        // UNCONDITIONALLY, so _horus.logs (every Error/Warning line) and
        // _horus.presence (the node inventory) were exported to any announcer
        // and an operator's deny_export entry was silently ignored.
        let reg = std::sync::Arc::new(TopicRegistry::new());
        let guard = ImportExportGuard::new(
            ImportMode::Auto,
            ExportMode::DenyList(vec!["_horus.logs".into()]),
            Some(reg),
        );
        assert!(
            !guard.allow_export("_horus.logs"),
            "an explicit deny_export on a system topic must be honoured"
        );
        assert!(
            guard.allow_export("_horus.presence"),
            "system topics not named in the deny list still export"
        );
    }

    #[test]
    fn system_topics_still_export_by_default() {
        // The default must not change: zero-config LAN replication depends on
        // presence and logs flowing without configuration.
        let reg = std::sync::Arc::new(TopicRegistry::new());
        let guard = ImportExportGuard::new_default(reg);
        assert!(guard.allow_export("_horus.logs"));
        assert!(guard.allow_export("_horus.presence"));
        assert!(guard.allow_export("_horus.estop"));
    }

    #[test]
    fn system_topics_honour_an_allowlist() {
        let reg = std::sync::Arc::new(TopicRegistry::new());
        let guard = ImportExportGuard::new(
            ImportMode::Auto,
            ExportMode::AllowList(vec!["imu".into()]),
            Some(reg),
        );
        assert!(guard.allow_export("imu"));
        assert!(
            !guard.allow_export("_horus.logs"),
            "an allowlist that does not name a system topic must exclude it"
        );
    }
}
