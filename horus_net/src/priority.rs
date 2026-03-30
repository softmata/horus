//! Priority system — Immediate, RealTime, Normal, Bulk.
//!
//! See blueprint section 13. Auto-inference from topic name and node config.

/// Message priority level. Determines reliability tier and optimizer bypass.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Priority {
    /// Emergency stop. Bypass ALL optimizers. Redundant + latched delivery.
    Immediate = 0,
    /// Motor commands. Bypass fusion. Redundant (2x).
    RealTime = 1,
    /// Sensor data. Full optimizer pipeline. Fire-and-forget. Default.
    Normal = 2,
    /// Camera frames, logs. Aggressive batching, may drop under congestion.
    Bulk = 3,
}

impl Priority {
    /// Decode from wire byte. Returns Normal for unknown values.
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Immediate,
            1 => Self::RealTime,
            2 => Self::Normal,
            3 => Self::Bulk,
            _ => Self::Normal,
        }
    }

    /// Auto-infer priority from topic name.
    pub fn infer_from_topic(name: &str) -> Self {
        let lower = name.to_ascii_lowercase();
        if lower.contains("estop") || lower.contains("emergency") || lower.contains("safety") {
            Self::Immediate
        } else {
            Self::Normal
        }
    }

    /// Full auto-inference: topic name + node has RT budget + message size.
    ///
    /// Precedence: Immediate (topic name) > RealTime (budget/deadline) > Bulk (>64KB) > Normal.
    pub fn auto_infer(topic_name: &str, node_has_budget: bool, msg_size: usize) -> Self {
        // Topic name keywords always win
        let from_name = Self::infer_from_topic(topic_name);
        if from_name == Self::Immediate {
            return Self::Immediate;
        }
        // Nodes with budget/deadline → RealTime
        if node_has_budget {
            return Self::RealTime;
        }
        // Large messages → Bulk
        if msg_size > 64 * 1024 {
            return Self::Bulk;
        }
        Self::Normal
    }

    /// Get priority for a system topic.
    pub fn for_system_topic(name: &str) -> Self {
        use crate::registry::*;
        if name == SYSTEM_TOPIC_ESTOP {
            Self::Immediate
        } else if name == SYSTEM_TOPIC_PRESENCE {
            Self::Normal
        } else if name == SYSTEM_TOPIC_LOGS {
            Self::Bulk
        } else {
            Self::Normal
        }
    }

    /// Parse from string (for config). Case-insensitive.
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Self {
        match s.to_ascii_lowercase().as_str() {
            "immediate" => Self::Immediate,
            "realtime" | "real_time" | "rt" => Self::RealTime,
            "normal" => Self::Normal,
            "bulk" => Self::Bulk,
            _ => Self::Normal,
        }
    }
}

/// Reliability tier for message delivery.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Reliability {
    /// Send once. If lost, next message arrives soon anyway.
    None = 0,
    /// Send N copies, staggered. For control commands.
    Redundant = 1,
    /// Resend until ACK. For safety-critical state changes only.
    Latched = 2,
}

impl Reliability {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::None,
            1 => Self::Redundant,
            2 => Self::Latched,
            _ => Self::None,
        }
    }

    /// Get reliability for a system topic.
    pub fn for_system_topic(name: &str) -> Self {
        use crate::registry::*;
        if name == SYSTEM_TOPIC_ESTOP {
            Self::Latched // guaranteed delivery, resend until ACK
        } else {
            // best-effort for presence, logs, and all other topics
            Self::None
        }
    }

    /// Default reliability for a given priority.
    pub fn default_for(priority: Priority) -> Self {
        match priority {
            Priority::Immediate => Self::Latched,
            Priority::RealTime => Self::Redundant,
            Priority::Normal => Self::None,
            Priority::Bulk => Self::None,
        }
    }
}

/// Encoding tag — how payload bytes should be interpreted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Encoding {
    /// Raw POD bytes, little-endian (x86, most ARM).
    PodLe = 0,
    /// Raw POD bytes, big-endian (some ARM, PowerPC).
    PodBe = 1,
    /// Serde bincode (handles endianness automatically).
    Bincode = 2,
}

impl Encoding {
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::PodLe,
            1 => Self::PodBe,
            2 => Self::Bincode,
            _ => Self::Bincode,
        }
    }

    /// Returns the native encoding for this platform.
    pub fn native_pod() -> Self {
        if cfg!(target_endian = "little") {
            Self::PodLe
        } else {
            Self::PodBe
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn priority_ordering() {
        assert!(Priority::Immediate < Priority::RealTime);
        assert!(Priority::RealTime < Priority::Normal);
        assert!(Priority::Normal < Priority::Bulk);
    }

    #[test]
    fn priority_roundtrip() {
        for v in 0..=3u8 {
            let p = Priority::from_u8(v);
            assert_eq!(p as u8, v);
        }
    }

    #[test]
    fn priority_unknown_defaults_normal() {
        assert_eq!(Priority::from_u8(255), Priority::Normal);
    }

    #[test]
    fn priority_infer_estop() {
        assert_eq!(
            Priority::infer_from_topic("robot.estop"),
            Priority::Immediate
        );
        assert_eq!(
            Priority::infer_from_topic("emergency_stop"),
            Priority::Immediate
        );
        assert_eq!(
            Priority::infer_from_topic("safety.status"),
            Priority::Immediate
        );
        assert_eq!(Priority::infer_from_topic("robot.imu"), Priority::Normal);
    }

    #[test]
    fn reliability_defaults() {
        assert_eq!(
            Reliability::default_for(Priority::Immediate),
            Reliability::Latched
        );
        assert_eq!(
            Reliability::default_for(Priority::RealTime),
            Reliability::Redundant
        );
        assert_eq!(
            Reliability::default_for(Priority::Normal),
            Reliability::None
        );
        assert_eq!(Reliability::default_for(Priority::Bulk), Reliability::None);
    }

    #[test]
    fn encoding_native() {
        // x86_64 is always LE
        #[cfg(target_endian = "little")]
        assert_eq!(Encoding::native_pod(), Encoding::PodLe);
        #[cfg(target_endian = "big")]
        assert_eq!(Encoding::native_pod(), Encoding::PodBe);
    }

    #[test]
    fn auto_infer_estop_always_immediate() {
        assert_eq!(
            Priority::auto_infer("robot.estop", false, 8),
            Priority::Immediate
        );
        assert_eq!(
            Priority::auto_infer("robot.estop", true, 100_000),
            Priority::Immediate
        );
    }

    #[test]
    fn auto_infer_budget_means_realtime() {
        assert_eq!(
            Priority::auto_infer("cmd_vel", true, 16),
            Priority::RealTime
        );
    }

    #[test]
    fn auto_infer_large_msg_means_bulk() {
        assert_eq!(
            Priority::auto_infer("camera.image", false, 100_000),
            Priority::Bulk
        );
    }

    #[test]
    fn auto_infer_default_is_normal() {
        assert_eq!(Priority::auto_infer("imu", false, 64), Priority::Normal);
    }

    #[test]
    fn priority_from_str() {
        assert_eq!(Priority::from_str("immediate"), Priority::Immediate);
        assert_eq!(Priority::from_str("realtime"), Priority::RealTime);
        assert_eq!(Priority::from_str("real_time"), Priority::RealTime);
        assert_eq!(Priority::from_str("rt"), Priority::RealTime);
        assert_eq!(Priority::from_str("normal"), Priority::Normal);
        assert_eq!(Priority::from_str("bulk"), Priority::Bulk);
        assert_eq!(Priority::from_str("unknown"), Priority::Normal);
    }
}
