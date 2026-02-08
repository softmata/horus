use horus_core::core::LogSummary;
// Keyboard input message type for HORUS framework
//
// Provides a standardized way to represent keyboard events across all HORUS components.

use serde::{Deserialize, Serialize};

/// Standardized keyboard input message for cross-platform keyboard events.
///
/// This structure captures keyboard input events in a consistent format that can be
/// used across different input systems and platforms within the HORUS framework.
///
/// # Examples
///
/// ```
/// use horus_library::KeyboardInput;
///
/// // Create a simple key press
/// let key_press = KeyboardInput::new(
///     "a".to_string(),
///     65,
///     vec![],
///     true
/// );
///
/// // Create a key press with modifiers
/// let ctrl_s = KeyboardInput::new(
///     "s".to_string(),
///     83,
///     vec!["Ctrl".to_string()],
///     true
/// );
///
/// // Check for modifiers
/// assert!(ctrl_s.is_ctrl());
/// assert!(!key_press.has_modifier("Shift"));
/// ```
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct KeyboardInput {
    pub key_name: [u8; 32],  // Fixed-size key name buffer (null-terminated)
    pub code: u32,           // Raw key code
    pub modifier_flags: u32, // Bit flags for modifiers instead of Vec<String>
    pub pressed: bool,       // True for press, false for release
    pub timestamp_ms: u64,   // Unix timestamp_ms in milliseconds
    pub _padding: [u8; 16],  // Padding to match JoystickInput size (72 bytes)
}

// Modifier bit flags
pub const MODIFIER_CTRL: u32 = 1 << 0;
pub const MODIFIER_ALT: u32 = 1 << 1;
pub const MODIFIER_SHIFT: u32 = 1 << 2;
pub const MODIFIER_SUPER: u32 = 1 << 3;
pub const MODIFIER_HYPER: u32 = 1 << 4;
pub const MODIFIER_META: u32 = 1 << 5;

impl KeyboardInput {
    pub fn new(key: String, code: u32, modifiers: Vec<String>, pressed: bool) -> Self {
        // Convert key name to fixed-size array
        let mut key_name = [0u8; 32];
        let key_bytes = key.as_bytes();
        let copy_len = key_bytes.len().min(31); // Leave room for null terminator
        key_name[..copy_len].copy_from_slice(&key_bytes[..copy_len]);

        // Convert modifiers to bit flags
        let mut modifier_flags = 0u32;
        for modifier in modifiers {
            match modifier.as_str() {
                "Ctrl" => modifier_flags |= MODIFIER_CTRL,
                "Alt" => modifier_flags |= MODIFIER_ALT,
                "Shift" => modifier_flags |= MODIFIER_SHIFT,
                "Super" => modifier_flags |= MODIFIER_SUPER,
                "Hyper" => modifier_flags |= MODIFIER_HYPER,
                "Meta" => modifier_flags |= MODIFIER_META,
                _ => {} // Ignore unknown modifiers
            }
        }

        Self {
            key_name,
            code,
            modifier_flags,
            pressed,
            timestamp_ms: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64,
            _padding: [0; 16],
        }
    }

    pub fn has_modifier(&self, modifier: &str) -> bool {
        match modifier {
            "Ctrl" => (self.modifier_flags & MODIFIER_CTRL) != 0,
            "Alt" => (self.modifier_flags & MODIFIER_ALT) != 0,
            "Shift" => (self.modifier_flags & MODIFIER_SHIFT) != 0,
            "Super" => (self.modifier_flags & MODIFIER_SUPER) != 0,
            "Hyper" => (self.modifier_flags & MODIFIER_HYPER) != 0,
            "Meta" => (self.modifier_flags & MODIFIER_META) != 0,
            _ => false,
        }
    }

    /// Get the key name as a String
    pub fn get_key_name(&self) -> String {
        // Find null terminator or use full buffer
        let end = self
            .key_name
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(self.key_name.len());
        String::from_utf8_lossy(&self.key_name[..end])
            .into_owned()
            .to_string()
    }

    /// Get modifier names as `Vec<String>` (for compatibility)
    pub fn get_modifiers(&self) -> Vec<String> {
        let mut modifiers = Vec::new();
        if (self.modifier_flags & MODIFIER_CTRL) != 0 {
            modifiers.push("Ctrl".to_string());
        }
        if (self.modifier_flags & MODIFIER_ALT) != 0 {
            modifiers.push("Alt".to_string());
        }
        if (self.modifier_flags & MODIFIER_SHIFT) != 0 {
            modifiers.push("Shift".to_string());
        }
        if (self.modifier_flags & MODIFIER_SUPER) != 0 {
            modifiers.push("Super".to_string());
        }
        if (self.modifier_flags & MODIFIER_HYPER) != 0 {
            modifiers.push("Hyper".to_string());
        }
        if (self.modifier_flags & MODIFIER_META) != 0 {
            modifiers.push("Meta".to_string());
        }
        modifiers
    }

    pub fn is_ctrl(&self) -> bool {
        self.has_modifier("Ctrl")
    }

    pub fn is_shift(&self) -> bool {
        self.has_modifier("Shift")
    }

    pub fn is_alt(&self) -> bool {
        self.has_modifier("Alt")
    }
}

impl LogSummary for KeyboardInput {
    fn log_summary(&self) -> String {
        format!("{:?}", self)
    }
}
