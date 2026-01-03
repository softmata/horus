use crate::KeyboardInput;
use horus_core::error::HorusResult;
use horus_core::terminal::set_raw_mode;
use horus_core::{terminal_println, Hub, Node, NodeInfo};

// Type alias for cleaner signatures
type Result<T> = HorusResult<T>;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;
#[cfg(not(feature = "crossterm"))]
use std::time::{SystemTime, UNIX_EPOCH};

// Processor imports for hybrid pattern
use crate::nodes::processor::{
    ClosureProcessor, FilterProcessor, PassThrough, Pipeline, Processor,
};

#[cfg(feature = "crossterm")]
use crossterm::{
    event::{self, Event, KeyCode, KeyEvent, KeyModifiers},
    terminal::{disable_raw_mode, enable_raw_mode},
};

/// Standard keyboard keycodes
pub mod keycodes {
    // Letters
    pub const KEY_A: u32 = 65;
    pub const KEY_B: u32 = 66;
    pub const KEY_C: u32 = 67;
    pub const KEY_D: u32 = 68;
    pub const KEY_E: u32 = 69;
    pub const KEY_F: u32 = 70;
    pub const KEY_G: u32 = 71;
    pub const KEY_H: u32 = 72;
    pub const KEY_I: u32 = 73;
    pub const KEY_J: u32 = 74;
    pub const KEY_K: u32 = 75;
    pub const KEY_L: u32 = 76;
    pub const KEY_M: u32 = 77;
    pub const KEY_N: u32 = 78;
    pub const KEY_O: u32 = 79;
    pub const KEY_P: u32 = 80;
    pub const KEY_Q: u32 = 81;
    pub const KEY_R: u32 = 82;
    pub const KEY_S: u32 = 83;
    pub const KEY_T: u32 = 84;
    pub const KEY_U: u32 = 85;
    pub const KEY_V: u32 = 86;
    pub const KEY_W: u32 = 87;
    pub const KEY_X: u32 = 88;
    pub const KEY_Y: u32 = 89;
    pub const KEY_Z: u32 = 90;

    // Numbers
    pub const KEY_0: u32 = 48;
    pub const KEY_1: u32 = 49;
    pub const KEY_2: u32 = 50;
    pub const KEY_3: u32 = 51;
    pub const KEY_4: u32 = 52;
    pub const KEY_5: u32 = 53;
    pub const KEY_6: u32 = 54;
    pub const KEY_7: u32 = 55;
    pub const KEY_8: u32 = 56;
    pub const KEY_9: u32 = 57;

    // Function keys
    pub const KEY_F1: u32 = 112;
    pub const KEY_F2: u32 = 113;
    pub const KEY_F3: u32 = 114;
    pub const KEY_F4: u32 = 115;
    pub const KEY_F5: u32 = 116;
    pub const KEY_F6: u32 = 117;
    pub const KEY_F7: u32 = 118;
    pub const KEY_F8: u32 = 119;
    pub const KEY_F9: u32 = 120;
    pub const KEY_F10: u32 = 121;
    pub const KEY_F11: u32 = 122;
    pub const KEY_F12: u32 = 123;

    // Arrow keys
    pub const KEY_ARROW_LEFT: u32 = 37;
    pub const KEY_ARROW_UP: u32 = 38;
    pub const KEY_ARROW_RIGHT: u32 = 39;
    pub const KEY_ARROW_DOWN: u32 = 40;

    // Special keys
    pub const KEY_ESCAPE: u32 = 27;
    pub const KEY_ENTER: u32 = 13;
    pub const KEY_TAB: u32 = 9;
    pub const KEY_BACKSPACE: u32 = 8;
    pub const KEY_DELETE: u32 = 46;
    pub const KEY_SPACE: u32 = 32;
    pub const KEY_SHIFT: u32 = 16;
    pub const KEY_CONTROL: u32 = 17;
    pub const KEY_ALT: u32 = 18;
    pub const KEY_CAPSLOCK: u32 = 20;
    pub const KEY_NUMLOCK: u32 = 144;
    pub const KEY_SCROLLLOCK: u32 = 145;
    pub const KEY_PAUSE: u32 = 19;
    pub const KEY_INSERT: u32 = 45;
    pub const KEY_HOME: u32 = 36;
    pub const KEY_PAGEUP: u32 = 33;
    pub const KEY_END: u32 = 35;
    pub const KEY_PAGEDOWN: u32 = 34;

    // Numpad
    pub const KEY_NUMPAD_0: u32 = 96;
    pub const KEY_NUMPAD_1: u32 = 97;
    pub const KEY_NUMPAD_2: u32 = 98;
    pub const KEY_NUMPAD_3: u32 = 99;
    pub const KEY_NUMPAD_4: u32 = 100;
    pub const KEY_NUMPAD_5: u32 = 101;
    pub const KEY_NUMPAD_6: u32 = 102;
    pub const KEY_NUMPAD_7: u32 = 103;
    pub const KEY_NUMPAD_8: u32 = 104;
    pub const KEY_NUMPAD_9: u32 = 105;
    pub const KEY_NUMPAD_MULTIPLY: u32 = 106;
    pub const KEY_NUMPAD_ADD: u32 = 107;
    pub const KEY_NUMPAD_SEPARATOR: u32 = 108;
    pub const KEY_NUMPAD_SUBTRACT: u32 = 109;
    pub const KEY_NUMPAD_DECIMAL: u32 = 110;
    pub const KEY_NUMPAD_DIVIDE: u32 = 111;

    // Symbols and punctuation
    pub const KEY_SEMICOLON: u32 = 186;
    pub const KEY_EQUALS: u32 = 187;
    pub const KEY_COMMA: u32 = 188;
    pub const KEY_MINUS: u32 = 189;
    pub const KEY_PERIOD: u32 = 190;
    pub const KEY_SLASH: u32 = 191;
    pub const KEY_BACKTICK: u32 = 192;
    pub const KEY_LEFTBRACKET: u32 = 219;
    pub const KEY_BACKSLASH: u32 = 220;
    pub const KEY_RIGHTBRACKET: u32 = 221;
    pub const KEY_APOSTROPHE: u32 = 222;

    // OS keys
    pub const KEY_LEFT_SUPER: u32 = 91; // Windows/Command key
    pub const KEY_RIGHT_SUPER: u32 = 92;
    pub const KEY_CONTEXT_MENU: u32 = 93;
}

/// Keyboard Input Node - Keyboard input capture with customizable key mapping
///
/// This node captures keyboard events and publishes them to the horus system.
/// It supports custom key mappings that can be overridden by users.
///
/// # Hybrid Pattern
///
/// This node supports the hybrid pattern for custom processing:
///
/// ```rust,ignore
/// // Default mode - just captures keyboard input
/// let node = KeyboardInputNode::new()?;
///
/// // With custom processing - filter specific keys
/// let node = KeyboardInputNode::builder()
///     .with_topic("custom_keyboard")
///     .with_filter(|input| {
///         // Only pass through arrow keys
///         if input.code >= 37 && input.code <= 40 {
///             Some(input)
///         } else {
///             None
///         }
///     })
///     .build()?;
/// ```
pub struct KeyboardInputNode<P = PassThrough<KeyboardInput>>
where
    P: Processor<KeyboardInput>,
{
    publisher: Hub<KeyboardInput>,
    /// Custom key mapping: maps from input string/char to (key_name, keycode)
    custom_mapping: Arc<Mutex<HashMap<String, (String, u32)>>>,
    /// Flag to indicate if terminal mode is enabled
    #[cfg(feature = "crossterm")]
    terminal_enabled: bool,
    /// Processor for hybrid pattern
    processor: P,
}

impl KeyboardInputNode {
    /// Create a new keyboard input node with default topic "keyboard_input"
    pub fn new() -> Result<Self> {
        Self::new_with_topic("keyboard_input")
    }

    /// Create a new keyboard input node with custom topic
    pub fn new_with_topic(topic: &str) -> Result<Self> {
        let mut node = Self {
            publisher: Hub::new(topic)?,
            custom_mapping: Arc::new(Mutex::new(HashMap::new())),
            #[cfg(feature = "crossterm")]
            terminal_enabled: false,
            processor: PassThrough::new(),
        };

        // Initialize with default mappings
        node.init_default_mappings();

        // Enable raw terminal mode for capturing keyboard input
        #[cfg(feature = "crossterm")]
        {
            if enable_raw_mode().is_ok() {
                node.terminal_enabled = true;
                set_raw_mode(true);
                terminal_println!(" Terminal keyboard input enabled. Press arrow keys to control, ESC or Ctrl+C to quit.");
            }
        }

        Ok(node)
    }

    /// Create a builder for configuring the node with custom processing
    pub fn builder() -> KeyboardInputNodeBuilder<PassThrough<KeyboardInput>> {
        KeyboardInputNodeBuilder::new()
    }
}

impl<P> KeyboardInputNode<P>
where
    P: Processor<KeyboardInput>,
{
    /// Initialize default key mappings
    fn init_default_mappings(&mut self) {
        let mut mappings = self.custom_mapping.lock().unwrap();

        // Arrow keys - multiple input options
        mappings.insert(
            "up".to_string(),
            ("ArrowUp".to_string(), keycodes::KEY_ARROW_UP),
        );
        mappings.insert(
            "arrowup".to_string(),
            ("ArrowUp".to_string(), keycodes::KEY_ARROW_UP),
        );
        mappings.insert(
            "down".to_string(),
            ("ArrowDown".to_string(), keycodes::KEY_ARROW_DOWN),
        );
        mappings.insert(
            "arrowdown".to_string(),
            ("ArrowDown".to_string(), keycodes::KEY_ARROW_DOWN),
        );
        mappings.insert(
            "left".to_string(),
            ("ArrowLeft".to_string(), keycodes::KEY_ARROW_LEFT),
        );
        mappings.insert(
            "arrowleft".to_string(),
            ("ArrowLeft".to_string(), keycodes::KEY_ARROW_LEFT),
        );
        mappings.insert(
            "right".to_string(),
            ("ArrowRight".to_string(), keycodes::KEY_ARROW_RIGHT),
        );
        mappings.insert(
            "arrowright".to_string(),
            ("ArrowRight".to_string(), keycodes::KEY_ARROW_RIGHT),
        );

        // WASD keys for gaming
        mappings.insert("w".to_string(), ("W".to_string(), keycodes::KEY_W));
        mappings.insert("a".to_string(), ("A".to_string(), keycodes::KEY_A));
        mappings.insert("s".to_string(), ("S".to_string(), keycodes::KEY_S));
        mappings.insert("d".to_string(), ("D".to_string(), keycodes::KEY_D));

        // All letters
        for (i, ch) in ('a'..='z').enumerate() {
            let key = ch.to_string();
            let upper = ch.to_uppercase().to_string();
            mappings.insert(key, (upper.clone(), 65 + i as u32));
        }

        // Numbers
        for i in 0..=9 {
            let key = i.to_string();
            mappings.insert(key.clone(), (key.clone(), 48 + i as u32));
        }

        // Function keys
        for i in 1..=12 {
            let key = format!("f{}", i);
            mappings.insert(key.clone(), (format!("F{}", i), 111 + i as u32));
        }

        // Special keys
        mappings.insert(
            "space".to_string(),
            ("Space".to_string(), keycodes::KEY_SPACE),
        );
        mappings.insert(" ".to_string(), ("Space".to_string(), keycodes::KEY_SPACE));
        mappings.insert(
            "enter".to_string(),
            ("Enter".to_string(), keycodes::KEY_ENTER),
        );
        mappings.insert(
            "return".to_string(),
            ("Enter".to_string(), keycodes::KEY_ENTER),
        );
        mappings.insert("tab".to_string(), ("Tab".to_string(), keycodes::KEY_TAB));
        mappings.insert(
            "escape".to_string(),
            ("Escape".to_string(), keycodes::KEY_ESCAPE),
        );
        mappings.insert(
            "esc".to_string(),
            ("Escape".to_string(), keycodes::KEY_ESCAPE),
        );
        mappings.insert(
            "backspace".to_string(),
            ("Backspace".to_string(), keycodes::KEY_BACKSPACE),
        );
        mappings.insert(
            "delete".to_string(),
            ("Delete".to_string(), keycodes::KEY_DELETE),
        );
        mappings.insert(
            "shift".to_string(),
            ("Shift".to_string(), keycodes::KEY_SHIFT),
        );
        mappings.insert(
            "control".to_string(),
            ("Control".to_string(), keycodes::KEY_CONTROL),
        );
        mappings.insert(
            "ctrl".to_string(),
            ("Control".to_string(), keycodes::KEY_CONTROL),
        );
        mappings.insert("alt".to_string(), ("Alt".to_string(), keycodes::KEY_ALT));
        mappings.insert("home".to_string(), ("Home".to_string(), keycodes::KEY_HOME));
        mappings.insert("end".to_string(), ("End".to_string(), keycodes::KEY_END));
        mappings.insert(
            "pageup".to_string(),
            ("PageUp".to_string(), keycodes::KEY_PAGEUP),
        );
        mappings.insert(
            "pagedown".to_string(),
            ("PageDown".to_string(), keycodes::KEY_PAGEDOWN),
        );
        mappings.insert(
            "insert".to_string(),
            ("Insert".to_string(), keycodes::KEY_INSERT),
        );
    }

    /// Override key mapping for a specific input
    ///
    /// # Arguments
    /// * `input_key` - The input string to map (e.g., "w", "up", "space")
    /// * `key_name` - The name to use for the key event
    /// * `keycode` - The keycode to use for the key event
    pub fn set_key_mapping(&mut self, input_key: String, key_name: String, keycode: u32) {
        let mut mappings = self.custom_mapping.lock().unwrap();
        mappings.insert(input_key.to_lowercase(), (key_name, keycode));
    }

    /// Add multiple key mappings at once
    pub fn set_key_mappings(&mut self, mappings: HashMap<String, (String, u32)>) {
        let mut current_mappings = self.custom_mapping.lock().unwrap();
        for (input_key, value) in mappings {
            current_mappings.insert(input_key.to_lowercase(), value);
        }
    }

    /// Clear all custom mappings and restore defaults
    pub fn reset_mappings(&mut self) {
        let mut mappings = self.custom_mapping.lock().unwrap();
        mappings.clear();
        drop(mappings);
        self.init_default_mappings();
    }

    /// Get current key mapping for an input
    pub fn get_key_mapping(&self, input_key: &str) -> Option<(String, u32)> {
        let mappings = self.custom_mapping.lock().unwrap();
        mappings.get(&input_key.to_lowercase()).cloned()
    }

    /// Process raw input and convert to KeyboardInput using the mapping
    #[allow(dead_code)]
    fn process_input(&self, raw_input: &str) -> Option<KeyboardInput> {
        let input = raw_input.trim().to_lowercase();
        let mappings = self.custom_mapping.lock().unwrap();

        if let Some((key_name, keycode)) = mappings.get(&input) {
            Some(KeyboardInput::new(
                key_name.clone(),
                *keycode,
                vec![], // Modifiers can be added based on input parsing
                true,
            ))
        } else {
            None
        }
    }

    /// Capture real keyboard events using crossterm
    #[cfg(feature = "crossterm")]
    fn capture_keyboard_event(&mut self) -> Option<KeyboardInput> {
        // Check if a key event is available (non-blocking)
        if event::poll(Duration::from_millis(0)).unwrap_or(false) {
            if let Ok(Event::Key(key_event)) = event::read() {
                return self.process_key_event(key_event);
            }
        }
        None
    }

    /// Process crossterm KeyEvent into KeyboardInput
    #[cfg(feature = "crossterm")]
    fn process_key_event(&self, key_event: KeyEvent) -> Option<KeyboardInput> {
        let (key_name, keycode) = match key_event.code {
            KeyCode::Up => ("ArrowUp".to_string(), keycodes::KEY_ARROW_UP),
            KeyCode::Down => ("ArrowDown".to_string(), keycodes::KEY_ARROW_DOWN),
            KeyCode::Left => ("ArrowLeft".to_string(), keycodes::KEY_ARROW_LEFT),
            KeyCode::Right => ("ArrowRight".to_string(), keycodes::KEY_ARROW_RIGHT),
            KeyCode::Char('w') | KeyCode::Char('W') => ("W".to_string(), keycodes::KEY_W),
            KeyCode::Char('a') | KeyCode::Char('A') => ("A".to_string(), keycodes::KEY_A),
            KeyCode::Char('s') | KeyCode::Char('S') => ("S".to_string(), keycodes::KEY_S),
            KeyCode::Char('d') | KeyCode::Char('D') => ("D".to_string(), keycodes::KEY_D),
            // Space key - Issue #26 fix: spacebar was being dropped
            KeyCode::Char(' ') => ("Space".to_string(), keycodes::KEY_SPACE),
            KeyCode::Char(c) => {
                let key_str = c.to_string();
                let code = if c.is_ascii_alphabetic() {
                    c.to_ascii_uppercase() as u32
                } else if c.is_ascii_digit() {
                    c as u32
                } else if c.is_ascii_punctuation() || c.is_ascii_whitespace() {
                    // Handle punctuation and whitespace by their ASCII value
                    c as u32
                } else {
                    return None;
                };
                (key_str, code)
            }
            KeyCode::Enter => ("Enter".to_string(), keycodes::KEY_ENTER),
            KeyCode::Esc => ("Escape".to_string(), keycodes::KEY_ESCAPE),
            KeyCode::Tab => ("Tab".to_string(), keycodes::KEY_TAB),
            KeyCode::Backspace => ("Backspace".to_string(), keycodes::KEY_BACKSPACE),
            KeyCode::Delete => ("Delete".to_string(), keycodes::KEY_DELETE),
            KeyCode::Home => ("Home".to_string(), keycodes::KEY_HOME),
            KeyCode::End => ("End".to_string(), keycodes::KEY_END),
            KeyCode::PageUp => ("PageUp".to_string(), keycodes::KEY_PAGEUP),
            KeyCode::PageDown => ("PageDown".to_string(), keycodes::KEY_PAGEDOWN),
            KeyCode::Insert => ("Insert".to_string(), keycodes::KEY_INSERT),
            KeyCode::F(n) if (1..=12).contains(&n) => (format!("F{}", n), 111 + n as u32),
            _ => return None,
        };

        // Build modifiers list
        let mut modifiers = Vec::new();
        if key_event.modifiers.contains(KeyModifiers::CONTROL) {
            modifiers.push("Ctrl".to_string());
        }
        if key_event.modifiers.contains(KeyModifiers::ALT) {
            modifiers.push("Alt".to_string());
        }
        if key_event.modifiers.contains(KeyModifiers::SHIFT) {
            modifiers.push("Shift".to_string());
        }

        Some(KeyboardInput::new(
            key_name, keycode, modifiers, true, // Key press events only
        ))
    }
}

impl<P> Node for KeyboardInputNode<P>
where
    P: Processor<KeyboardInput>,
{
    fn name(&self) -> &'static str {
        "KeyboardInputNode"
    }

    fn init(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        // Call processor hook
        self.processor.on_start();
        ctx.log_info("KeyboardInputNode initialized with hybrid processor pattern");
        Ok(())
    }

    fn shutdown(&mut self, ctx: &mut NodeInfo) -> Result<()> {
        ctx.log_info("KeyboardInputNode shutting down - restoring terminal mode");

        // Call processor hook
        self.processor.on_shutdown();

        // Restore terminal mode if raw mode was enabled
        #[cfg(feature = "crossterm")]
        {
            if self.terminal_enabled {
                let _ = disable_raw_mode();
                set_raw_mode(false);
                self.terminal_enabled = false;
            }
        }

        ctx.log_info("Terminal mode restored");
        Ok(())
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Call processor tick hook
        self.processor.on_tick();

        // Try to capture real keyboard events if crossterm is enabled
        #[cfg(feature = "crossterm")]
        {
            if self.terminal_enabled {
                if let Some(key_input) = self.capture_keyboard_event() {
                    // Handle Ctrl+C to quit
                    if key_input.has_modifier("Ctrl") && key_input.code == keycodes::KEY_C {
                        use colored::Colorize;
                        terminal_println!(
                            "{}",
                            "\nReceived Ctrl+C, shutting down gracefully...".red()
                        );
                        let _ = disable_raw_mode();
                        set_raw_mode(false);
                        self.terminal_enabled = false;
                        // Signal to exit by triggering standard Ctrl+C handler
                        std::process::exit(0);
                    }

                    // Handle ESC key to quit
                    if key_input.code == keycodes::KEY_ESCAPE {
                        terminal_println!("\n Received ESC key, disabling raw terminal mode...");
                        let _ = disable_raw_mode();
                        set_raw_mode(false);
                        self.terminal_enabled = false;
                        return;
                    }

                    // Publish the keyboard event via horus Hub (through processor pipeline)
                    if let Some(processed) = self.processor.process(key_input) {
                        let _ = self.publisher.send(processed, &mut ctx);
                    }
                }
            }
        }

        // Note: Non-crossterm fallback removed since struct lacks demo_key_index/last_key_time fields
    }
}

// Default impl removed - use Node::new() instead which returns HorusResult

#[cfg(feature = "crossterm")]
impl<P> Drop for KeyboardInputNode<P>
where
    P: Processor<KeyboardInput>,
{
    fn drop(&mut self) {
        // Clean up terminal mode on drop
        if self.terminal_enabled {
            let _ = disable_raw_mode();
            set_raw_mode(false);
        }
    }
}

/// Builder for KeyboardInputNode with custom processor
pub struct KeyboardInputNodeBuilder<P>
where
    P: Processor<KeyboardInput>,
{
    topic: String,
    processor: P,
}

impl KeyboardInputNodeBuilder<PassThrough<KeyboardInput>> {
    /// Create a new builder with default settings
    pub fn new() -> Self {
        Self {
            topic: "keyboard_input".to_string(),
            processor: PassThrough::new(),
        }
    }
}

impl Default for KeyboardInputNodeBuilder<PassThrough<KeyboardInput>> {
    fn default() -> Self {
        Self::new()
    }
}

impl<P> KeyboardInputNodeBuilder<P>
where
    P: Processor<KeyboardInput>,
{
    /// Set the topic for publishing keyboard input
    pub fn with_topic(mut self, topic: &str) -> Self {
        self.topic = topic.to_string();
        self
    }

    /// Set a custom processor
    pub fn with_processor<P2>(self, processor: P2) -> KeyboardInputNodeBuilder<P2>
    where
        P2: Processor<KeyboardInput>,
    {
        KeyboardInputNodeBuilder {
            topic: self.topic,
            processor,
        }
    }

    /// Add a closure-based processor
    #[allow(clippy::type_complexity)]
    pub fn with_closure<F>(
        self,
        f: F,
    ) -> KeyboardInputNodeBuilder<
        Pipeline<
            KeyboardInput,
            KeyboardInput,
            KeyboardInput,
            P,
            ClosureProcessor<KeyboardInput, KeyboardInput, F>,
        >,
    >
    where
        F: FnMut(KeyboardInput) -> KeyboardInput + Send + 'static,
    {
        KeyboardInputNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, ClosureProcessor::new(f)),
        }
    }

    /// Add a filter processor
    #[allow(clippy::type_complexity)]
    pub fn with_filter<F>(
        self,
        f: F,
    ) -> KeyboardInputNodeBuilder<
        Pipeline<
            KeyboardInput,
            KeyboardInput,
            KeyboardInput,
            P,
            FilterProcessor<KeyboardInput, KeyboardInput, F>,
        >,
    >
    where
        F: FnMut(KeyboardInput) -> Option<KeyboardInput> + Send + 'static,
    {
        KeyboardInputNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, FilterProcessor::new(f)),
        }
    }

    /// Pipe to another processor (output must remain KeyboardInput)
    pub fn pipe<P2>(
        self,
        next: P2,
    ) -> KeyboardInputNodeBuilder<Pipeline<KeyboardInput, KeyboardInput, KeyboardInput, P, P2>>
    where
        P2: Processor<KeyboardInput, KeyboardInput>,
    {
        KeyboardInputNodeBuilder {
            topic: self.topic,
            processor: Pipeline::new(self.processor, next),
        }
    }

    /// Build the node
    pub fn build(self) -> Result<KeyboardInputNode<P>> {
        let mut node = KeyboardInputNode {
            publisher: Hub::new(&self.topic)?,
            custom_mapping: Arc::new(Mutex::new(HashMap::new())),
            #[cfg(feature = "crossterm")]
            terminal_enabled: false,
            processor: self.processor,
        };

        // Initialize with default mappings
        node.init_default_mappings();

        // Enable raw terminal mode for capturing keyboard input
        #[cfg(feature = "crossterm")]
        {
            if enable_raw_mode().is_ok() {
                node.terminal_enabled = true;
                set_raw_mode(true);
                terminal_println!(" Terminal keyboard input enabled. Press arrow keys to control, ESC or Ctrl+C to quit.");
            }
        }

        Ok(node)
    }
}
