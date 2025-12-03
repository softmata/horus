use bevy::prelude::*;

// Import actual HorusPublisher and HorusSubscriber from horus_bridge
pub use crate::horus_bridge::publisher::HorusPublisher;
pub use crate::horus_bridge::subscriber::HorusSubscriber;

/// Configuration for HORUS synchronization
#[derive(Resource, Clone)]
pub struct HorusSyncConfig {
    /// Enable HORUS synchronization
    pub enabled: bool,
    /// Publish rate in Hz (0 = every frame)
    pub publish_rate: f32,
    /// Last publish time
    pub last_publish: f32,
}

impl Default for HorusSyncConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            publish_rate: 50.0,           // 50 Hz default
            last_publish: -f32::INFINITY, // Start with negative infinity so first publish always succeeds
        }
    }
}

impl HorusSyncConfig {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_rate(mut self, rate: f32) -> Self {
        self.publish_rate = rate;
        self
    }

    pub fn disabled() -> Self {
        Self {
            enabled: false,
            ..default()
        }
    }

    pub fn should_publish(&self, current_time: f32) -> bool {
        if !self.enabled || self.publish_rate <= 0.0 {
            return self.enabled;
        }
        current_time - self.last_publish >= 1.0 / self.publish_rate
    }

    pub fn update_time(&mut self, current_time: f32) {
        self.last_publish = current_time;
    }
}

/// Main HORUS synchronization system - coordinates timing and configuration
pub fn horus_sync_system(
    time: Res<Time>,
    mut config: ResMut<HorusSyncConfig>,
    mut publisher: ResMut<HorusPublisher>,
    mut subscriber: ResMut<HorusSubscriber>,
) {
    let current_time = time.elapsed_secs();

    // Enable/disable publisher and subscriber based on config
    if config.enabled {
        publisher.enable();
        subscriber.enable();
    } else {
        publisher.disable();
        subscriber.disable();
        return;
    }

    // Rate limiting for publishing
    if config.should_publish(current_time) {
        config.update_time(current_time);

        // Actual publishing is done by the individual publish systems
        // in horus_bridge/publisher.rs (publish_hframe_system, publish_lidar3d_system, etc.)
        // This system just coordinates timing and configuration
    }
}

/// Resource to track HORUS sync statistics
#[derive(Resource, Default)]
pub struct HorusSyncStats {
    pub messages_published: u64,
    pub messages_received: u64,
    pub last_publish_time: f32,
    pub last_receive_time: f32,
    pub publish_errors: u64,
}

impl HorusSyncStats {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn record_publish(&mut self, count: u64, time: f32) {
        self.messages_published += count;
        self.last_publish_time = time;
    }

    pub fn record_receive(&mut self, count: u64, time: f32) {
        self.messages_received += count;
        self.last_receive_time = time;
    }

    pub fn record_error(&mut self) {
        self.publish_errors += 1;
    }

    pub fn reset(&mut self) {
        self.messages_published = 0;
        self.messages_received = 0;
        self.publish_errors = 0;
    }
}

/// System to track HORUS sync statistics
pub fn track_horus_sync_stats_system(
    time: Res<Time>,
    config: Res<HorusSyncConfig>,
    mut stats: ResMut<HorusSyncStats>,
    _publisher: Res<HorusPublisher>,
) {
    if !config.enabled {
        return;
    }

    // Track that sync is happening
    // Actual message counts would come from the publisher/subscriber
    stats.last_publish_time = time.elapsed_secs();
}

/// Event fired when HORUS sync completes
#[derive(Event)]
pub struct HorusSyncEvent {
    pub sync_time: f32,
    pub enabled: bool,
}

/// System to emit HORUS sync events
pub fn emit_horus_sync_event(
    time: Res<Time>,
    config: Res<HorusSyncConfig>,
    mut events: EventWriter<HorusSyncEvent>,
    mut last_sync: Local<f32>,
) {
    if config.enabled && time.elapsed_secs() - *last_sync > 1.0 {
        events.send(HorusSyncEvent {
            sync_time: time.elapsed_secs(),
            enabled: config.enabled,
        });
        *last_sync = time.elapsed_secs();
    }
}

/// Plugin to register HORUS sync systems
///
/// Note: HorusPublisher and HorusSubscriber are registered by HorusBridgePlugin,
/// so they should not be re-registered here.
pub struct HorusSyncPlugin;

impl Plugin for HorusSyncPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HorusSyncConfig>()
            .init_resource::<HorusSyncStats>()
            // Note: HorusPublisher and HorusSubscriber are registered by HorusBridgePlugin
            .add_event::<HorusSyncEvent>()
            .add_systems(
                Update,
                (
                    horus_sync_system,
                    track_horus_sync_stats_system,
                    emit_horus_sync_event,
                )
                    .chain(),
            );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_horus_sync_config() {
        let config = HorusSyncConfig::new();
        assert!(config.enabled);
        assert_eq!(config.publish_rate, 50.0);

        let config = HorusSyncConfig::new().with_rate(100.0);
        assert_eq!(config.publish_rate, 100.0);

        let disabled = HorusSyncConfig::disabled();
        assert!(!disabled.enabled);
    }

    #[test]
    fn test_horus_sync_rate_limiting() {
        let mut config = HorusSyncConfig::new().with_rate(10.0);

        assert!(config.should_publish(0.0));
        config.update_time(0.0);

        assert!(!config.should_publish(0.05)); // 50ms < 100ms
        assert!(config.should_publish(0.11)); // 110ms > 100ms
    }

    #[test]
    fn test_horus_sync_stats() {
        let mut stats = HorusSyncStats::new();
        assert_eq!(stats.messages_published, 0);

        stats.record_publish(10, 1.0);
        assert_eq!(stats.messages_published, 10);
        assert_eq!(stats.last_publish_time, 1.0);

        stats.record_receive(5, 1.5);
        assert_eq!(stats.messages_received, 5);

        stats.record_error();
        assert_eq!(stats.publish_errors, 1);

        stats.reset();
        assert_eq!(stats.messages_published, 0);
        assert_eq!(stats.messages_received, 0);
        assert_eq!(stats.publish_errors, 0);
    }
}
