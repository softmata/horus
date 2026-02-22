mod data;
mod state;
mod widgets;

// Terminal UI Monitor for HORUS
use anyhow::Result;
use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use horus_core::core::LogType;
use horus_core::core::log_buffer::GLOBAL_LOG_BUFFER;
use horus_core::memory::shm_topics_dir;
use ratatui::{
    backend::{Backend, CrosstermBackend},
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, Borders, Cell, Paragraph, Row, Table, TableState, Tabs},
    Frame, Terminal,
};
use std::io::stdout;
use std::time::{Duration, Instant};

// Import the monitoring structs and functions
#[derive(Debug, Clone)]
pub struct NodeStatus {
    pub name: String,
    pub status: String,
    pub priority: u32,
    pub process_id: u32,
    pub cpu_usage: f32,
    pub memory_usage: u64,
    pub publishers: Vec<String>,  // Topic names this node publishes to
    pub subscribers: Vec<String>, // Topic names this node subscribes from
}

#[derive(Clone)]
pub struct TuiDashboard {
    active_tab: Tab,
    selected_index: usize,
    scroll_offset: usize,

    // Data
    nodes: Vec<NodeStatus>,
    topics: Vec<TopicInfo>,
    params: std::sync::Arc<horus_core::RuntimeParams>,

    // State
    paused: bool,
    show_help: bool,
    last_update: Instant,

    // Log panel state
    show_log_panel: bool,
    panel_target: Option<LogPanelTarget>,
    panel_scroll_offset: usize,

    // Parameter editing state
    param_edit_mode: ParamEditMode,
    param_input_key: String,
    param_input_value: String,
    param_input_focus: ParamInputFocus,

    // Package navigation state
    package_view_mode: PackageViewMode,
    package_panel_focus: PackagePanelFocus,
    selected_workspace: Option<WorkspaceData>,

    // Overview panel focus
    overview_panel_focus: OverviewPanelFocus,

    // Workspace caching (to avoid repeated filesystem operations)
    workspace_cache: Vec<WorkspaceData>,
    workspace_cache_time: Instant,
    current_workspace_path: Option<std::path::PathBuf>,

    // Time-travel debugger state
    debugger_state: DebuggerViewState,
}

/// State for the time-travel debugger view
#[derive(Debug, Clone)]
struct DebuggerViewState {
    /// Currently selected recording session
    selected_session: Option<String>,
    /// Whether debugger is active
    active: bool,
    /// Current tick position
    current_tick: u64,
    /// Total ticks in recording
    total_ticks: u64,
    /// Playback state
    playback: PlaybackState,
    /// Playback speed (1.0 = normal)
    playback_speed: f64,
    /// Breakpoints (tick numbers)
    breakpoints: Vec<u64>,
    /// Watch expressions
    watches: Vec<String>,
    /// Selected panel (0=timeline, 1=data, 2=watches)
    selected_panel: usize,
    /// Recording list cache
    recordings_cache: Vec<RecordingInfo>,
    /// Cache time
    cache_time: Instant,
}

#[derive(Debug, Clone, PartialEq)]
enum PlaybackState {
    Stopped,
    Playing,
    Paused,
    SteppingForward,
    SteppingBackward,
}

#[derive(Debug, Clone)]
struct RecordingInfo {
    session_name: String,
    recording_type: RecordingType,
    file_count: usize,
    size_bytes: u64,
    total_ticks: Option<u64>,
}

#[derive(Debug, Clone, PartialEq)]
enum RecordingType {
    Standard,
    ZeroCopy,
    Distributed,
}

#[derive(Debug, Clone, PartialEq)]
enum ParamEditMode {
    None,
    Add,
    Edit(String),   // Stores the original key being edited
    Delete(String), // Stores the key to delete
}

#[derive(Debug, Clone, PartialEq)]
enum ParamInputFocus {
    Key,
    Value,
}

#[derive(Debug, Clone, PartialEq)]
enum PackageViewMode {
    List,             // Viewing all workspaces
    WorkspaceDetails, // Viewing packages inside a workspace
}

#[derive(Debug, Clone, PartialEq)]
enum PackagePanelFocus {
    LocalWorkspaces, // Focused on local workspaces panel
    GlobalPackages,  // Focused on global packages panel
}

#[derive(Debug, Clone, PartialEq)]
enum OverviewPanelFocus {
    Nodes,  // Focused on nodes panel
    Topics, // Focused on topics panel
}

#[derive(Debug, Clone)]
struct WorkspaceData {
    name: String,
    path: String,
    packages: Vec<PackageData>,
    dependencies: Vec<DependencyData>, // Declared in horus.yaml but not installed
    is_current: bool, // True if this is the current workspace (detected via find_workspace_root)
}

#[derive(Debug, Clone)]
struct PackageData {
    name: String,
    version: String,
    installed_packages: Vec<(String, String)>, // (name, version) pairs
}

#[derive(Debug, Clone)]
struct DependencyData {
    name: String,
    declared_version: String, // Version string from horus.yaml (e.g., "package@1.0.0" or just "package")
    _status: DependencyStatus,
}

#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
enum DependencyStatus {
    Missing,
    Installed,
}

#[derive(Debug, Clone)]
struct TopicInfo {
    name: String,
    msg_type: String,
    publishers: usize,
    subscribers: usize,
    rate: f32,
    publisher_nodes: Vec<String>,
    subscriber_nodes: Vec<String>,
    /// Topic lifecycle status (Active or Idle - Stale topics are filtered out)
    status: crate::discovery::TopicStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Tab {
    Overview,
    Nodes,
    Topics,
    Network,
    HFrame,
    Packages,
    Parameters,
    Recordings,
}

#[derive(Debug, Clone, PartialEq)]
enum LogPanelTarget {
    Node(String),
    Topic(String),
}

impl Tab {
    fn as_str(&self) -> &'static str {
        match self {
            Tab::Overview => "Overview",
            Tab::Nodes => "Nodes",
            Tab::Topics => "Topics",
            Tab::Network => "Network",
            Tab::HFrame => "HFrame",
            Tab::Packages => "Packages",
            Tab::Parameters => "Params",
            Tab::Recordings => "Recordings",
        }
    }

    fn all() -> Vec<Tab> {
        vec![
            Tab::Overview,
            Tab::Nodes,
            Tab::Topics,
            Tab::Network,
            Tab::HFrame,
            Tab::Packages,
            Tab::Parameters,
            Tab::Recordings,
        ]
    }
}

impl Default for TuiDashboard {
    fn default() -> Self {
        Self::new()
    }
}

impl TuiDashboard {
    pub fn new() -> Self {
        // Initialize real RuntimeParams
        let params = std::sync::Arc::new(
            horus_core::RuntimeParams::init()
                .unwrap_or_else(|_| horus_core::RuntimeParams::default()),
        );

        // Detect current workspace on startup
        let current_workspace_path = crate::workspace::find_workspace_root();

        Self {
            active_tab: Tab::Overview,
            selected_index: 0,
            scroll_offset: 0,

            nodes: Vec::new(),
            topics: Vec::new(),
            params,

            paused: false,
            show_help: false,
            last_update: Instant::now(),

            show_log_panel: false,
            panel_target: None,
            panel_scroll_offset: 0,

            param_edit_mode: ParamEditMode::None,
            param_input_key: String::new(),
            param_input_value: String::new(),
            param_input_focus: ParamInputFocus::Key,

            package_view_mode: PackageViewMode::List,
            package_panel_focus: PackagePanelFocus::LocalWorkspaces,
            selected_workspace: None,

            overview_panel_focus: OverviewPanelFocus::Nodes,

            // Initialize workspace cache as empty (will load on first access)
            workspace_cache: Vec::new(),
            workspace_cache_time: Instant::now() - Duration::from_secs(10), // Force initial load
            current_workspace_path,

            // Time-travel debugger state
            debugger_state: DebuggerViewState {
                selected_session: None,
                active: false,
                current_tick: 0,
                total_ticks: 0,
                playback: PlaybackState::Stopped,
                playback_speed: 1.0,
                breakpoints: Vec::new(),
                watches: Vec::new(),
                selected_panel: 0,
                recordings_cache: Vec::new(),
                cache_time: Instant::now() - Duration::from_secs(10),
            },
        }
    }

    pub fn run() -> Result<()> {
        // Setup terminal
        enable_raw_mode()?;
        let mut stdout = stdout();
        execute!(stdout, EnterAlternateScreen)?;
        let backend = CrosstermBackend::new(stdout);
        let mut terminal = Terminal::new(backend)?;

        // Create app and run
        let mut app = TuiDashboard::new();
        let res = app.run_app(&mut terminal);

        // Restore terminal
        disable_raw_mode()?;
        execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
        terminal.show_cursor()?;

        if let Err(err) = res {
            log::error!("TUI error: {:?}", err);
        }

        Ok(())
    }

    fn run_app<B: Backend>(&mut self, terminal: &mut Terminal<B>) -> Result<()>
    where
        B::Error: Send + Sync + 'static,
    {
        loop {
            // Update data if not paused (250ms refresh for real-time feel)
            if !self.paused && self.last_update.elapsed() > Duration::from_millis(crate::config::TUI_REFRESH_INTERVAL_MS) {
                self.update_data()?;
                self.last_update = Instant::now();
            }

            // Draw UI
            terminal.draw(|f| self.draw_ui(f))?;

            // Handle input
            if event::poll(Duration::from_millis(crate::config::TUI_POLL_INTERVAL_MS))? {
                if let Event::Key(key) = event::read()? {
                    if self.show_help {
                        self.show_help = false;
                        continue;
                    }

                    // Check if Shift is pressed
                    let shift_pressed = key.modifiers.contains(KeyModifiers::SHIFT);

                    match key.code {
                        KeyCode::Char('q') | KeyCode::Char('Q') => return Ok(()),

                        // ESC key: exit debugger, navigate back in packages, close log panel, or cancel edit mode
                        KeyCode::Esc => {
                            if self.active_tab == Tab::Recordings && self.debugger_state.active {
                                // Exit debugger mode
                                self.debugger_state.active = false;
                                self.debugger_state.playback = PlaybackState::Stopped;
                            } else if self.param_edit_mode != ParamEditMode::None {
                                // Cancel parameter editing
                                self.param_edit_mode = ParamEditMode::None;
                                self.param_input_key.clear();
                                self.param_input_value.clear();
                            } else if self.active_tab == Tab::Packages
                                && self.package_view_mode == PackageViewMode::WorkspaceDetails
                            {
                                // Navigate back to workspace list
                                self.package_view_mode = PackageViewMode::List;
                                self.package_panel_focus = PackagePanelFocus::LocalWorkspaces;
                                self.selected_workspace = None;
                                self.selected_index = 0;
                            } else if self.show_log_panel {
                                self.show_log_panel = false;
                                self.panel_target = None;
                                self.panel_scroll_offset = 0;
                            }
                        }

                        // Enter key: navigate packages or open log panel
                        KeyCode::Enter if self.param_edit_mode == ParamEditMode::None => {
                            if self.active_tab == Tab::Packages {
                                self.handle_package_enter();
                            } else if !self.show_log_panel {
                                self.open_log_panel();
                            }
                        }

                        KeyCode::Tab => self.next_tab(),
                        KeyCode::BackTab => self.prev_tab(),
                        KeyCode::Char('p') | KeyCode::Char('P') => self.paused = !self.paused,
                        KeyCode::Char('?') | KeyCode::Char('h') | KeyCode::Char('H') => {
                            self.show_help = true
                        }

                        // Up/Down keys with different behavior based on Shift
                        KeyCode::Up => {
                            if shift_pressed && self.show_log_panel {
                                // Shift+Up: Navigate to previous node/topic and update log panel
                                self.select_prev();
                                self.update_log_panel_target();
                            } else if self.show_log_panel {
                                // Up: Scroll logs up
                                self.panel_scroll_offset =
                                    self.panel_scroll_offset.saturating_sub(1);
                            } else {
                                // Up: Navigate list
                                self.select_prev();
                            }
                        }
                        KeyCode::Down => {
                            if shift_pressed && self.show_log_panel {
                                // Shift+Down: Navigate to next node/topic and update log panel
                                self.select_next();
                                self.update_log_panel_target();
                            } else if self.show_log_panel {
                                // Down: Scroll logs down
                                self.panel_scroll_offset =
                                    self.panel_scroll_offset.saturating_add(1);
                            } else {
                                // Down: Navigate list
                                self.select_next();
                            }
                        }

                        KeyCode::PageUp => {
                            if self.show_log_panel {
                                self.panel_scroll_offset =
                                    self.panel_scroll_offset.saturating_sub(10);
                            } else {
                                self.scroll_up(10);
                            }
                        }
                        KeyCode::PageDown => {
                            if self.show_log_panel {
                                self.panel_scroll_offset =
                                    self.panel_scroll_offset.saturating_add(10);
                            } else {
                                self.scroll_down(10);
                            }
                        }

                        // Parameter operations (only in Parameters tab)
                        KeyCode::Char('r') | KeyCode::Char('R')
                            if self.active_tab == Tab::Parameters
                                && self.param_edit_mode == ParamEditMode::None =>
                        {
                            // Refresh parameters from disk
                            self.params = std::sync::Arc::new(
                                horus_core::RuntimeParams::init()
                                    .unwrap_or_else(|_| horus_core::RuntimeParams::default()),
                            );
                        }
                        KeyCode::Char('s') | KeyCode::Char('S')
                            if self.active_tab == Tab::Parameters
                                && self.param_edit_mode == ParamEditMode::None =>
                        {
                            // Save parameters to disk
                            let _ = self.params.save_to_disk();
                        }
                        KeyCode::Char('a') | KeyCode::Char('A')
                            if self.active_tab == Tab::Parameters
                                && self.param_edit_mode == ParamEditMode::None =>
                        {
                            // Start adding a new parameter
                            self.param_edit_mode = ParamEditMode::Add;
                            self.param_input_key.clear();
                            self.param_input_value.clear();
                            self.param_input_focus = ParamInputFocus::Key;
                        }
                        KeyCode::Char('e') | KeyCode::Char('E')
                            if self.active_tab == Tab::Parameters
                                && self.param_edit_mode == ParamEditMode::None =>
                        {
                            // Start editing selected parameter
                            self.start_edit_parameter();
                        }
                        KeyCode::Char('d') | KeyCode::Char('D')
                            if self.active_tab == Tab::Parameters
                                && self.param_edit_mode == ParamEditMode::None =>
                        {
                            // Delete selected parameter (with confirmation)
                            self.start_delete_parameter();
                        }

                        // Switch between Nodes/Topics panels in Overview tab
                        KeyCode::Left
                            if self.active_tab == Tab::Overview && !self.show_log_panel =>
                        {
                            self.overview_panel_focus = OverviewPanelFocus::Nodes;
                            self.selected_index = 0;
                            self.scroll_offset = 0;
                        }
                        KeyCode::Right
                            if self.active_tab == Tab::Overview && !self.show_log_panel =>
                        {
                            self.overview_panel_focus = OverviewPanelFocus::Topics;
                            self.selected_index = 0;
                            self.scroll_offset = 0;
                        }

                        // Switch between Local/Global panels in Packages tab
                        KeyCode::Left
                            if self.active_tab == Tab::Packages
                                && self.package_view_mode == PackageViewMode::List
                                && !self.show_log_panel =>
                        {
                            self.package_panel_focus = PackagePanelFocus::LocalWorkspaces;
                            self.selected_index = 0;
                        }
                        KeyCode::Right
                            if self.active_tab == Tab::Packages
                                && self.package_view_mode == PackageViewMode::List
                                && !self.show_log_panel =>
                        {
                            self.package_panel_focus = PackagePanelFocus::GlobalPackages;
                            self.selected_index = 0;
                        }

                        // Handle input when in parameter edit mode
                        KeyCode::Char(c) if self.param_edit_mode != ParamEditMode::None => {
                            match self.param_edit_mode {
                                ParamEditMode::Add | ParamEditMode::Edit(_) => {
                                    match self.param_input_focus {
                                        ParamInputFocus::Key => self.param_input_key.push(c),
                                        ParamInputFocus::Value => self.param_input_value.push(c),
                                    }
                                }
                                ParamEditMode::Delete(_) => {
                                    // In delete confirmation, 'y' confirms, 'n' or ESC cancels
                                    if c == 'y' || c == 'Y' {
                                        self.confirm_delete_parameter();
                                    } else if c == 'n' || c == 'N' {
                                        self.param_edit_mode = ParamEditMode::None;
                                    }
                                }
                                _ => {}
                            }
                        }
                        KeyCode::Backspace if self.param_edit_mode != ParamEditMode::None => {
                            match self.param_edit_mode {
                                ParamEditMode::Add | ParamEditMode::Edit(_) => {
                                    match self.param_input_focus {
                                        ParamInputFocus::Key => {
                                            self.param_input_key.pop();
                                        }
                                        ParamInputFocus::Value => {
                                            self.param_input_value.pop();
                                        }
                                    }
                                }
                                _ => {}
                            }
                        }
                        KeyCode::Enter if self.param_edit_mode != ParamEditMode::None => {
                            match &self.param_edit_mode {
                                ParamEditMode::Add => {
                                    if self.param_input_focus == ParamInputFocus::Key {
                                        // Move to value input
                                        self.param_input_focus = ParamInputFocus::Value;
                                    } else {
                                        // Confirm add
                                        self.confirm_add_parameter();
                                    }
                                }
                                ParamEditMode::Edit(_) => {
                                    if self.param_input_focus == ParamInputFocus::Key {
                                        // Move to value input
                                        self.param_input_focus = ParamInputFocus::Value;
                                    } else {
                                        // Confirm edit
                                        self.confirm_edit_parameter();
                                    }
                                }
                                ParamEditMode::Delete(_) => {
                                    // Enter confirms delete
                                    self.confirm_delete_parameter();
                                }
                                _ => {}
                            }
                        }

                        // Debugger controls (only in Recordings tab)
                        KeyCode::Enter
                            if self.active_tab == Tab::Recordings
                                && !self.debugger_state.active =>
                        {
                            // Load selected recording into debugger
                            self.load_recording_into_debugger();
                        }
                        KeyCode::Char(' ')
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Play/Pause
                            match self.debugger_state.playback {
                                PlaybackState::Playing => {
                                    self.debugger_state.playback = PlaybackState::Paused
                                }
                                _ => self.debugger_state.playback = PlaybackState::Playing,
                            }
                        }
                        KeyCode::Char('.') | KeyCode::Char('>')
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Step forward
                            self.debugger_step_forward();
                        }
                        KeyCode::Char(',') | KeyCode::Char('<')
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Step backward
                            self.debugger_step_backward();
                        }
                        KeyCode::Char('b') | KeyCode::Char('B')
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Toggle breakpoint at current tick
                            self.toggle_breakpoint();
                        }
                        KeyCode::Home
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Jump to start
                            self.debugger_state.current_tick = 0;
                            self.debugger_state.playback = PlaybackState::Paused;
                        }
                        KeyCode::End
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Jump to end
                            self.debugger_state.current_tick =
                                self.debugger_state.total_ticks.saturating_sub(1);
                            self.debugger_state.playback = PlaybackState::Paused;
                        }
                        KeyCode::Char('[')
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Decrease playback speed
                            self.debugger_state.playback_speed =
                                (self.debugger_state.playback_speed * 0.5).max(0.125);
                        }
                        KeyCode::Char(']')
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Increase playback speed
                            self.debugger_state.playback_speed =
                                (self.debugger_state.playback_speed * 2.0).min(8.0);
                        }
                        KeyCode::Left
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Switch debugger panel focus
                            self.debugger_state.selected_panel =
                                self.debugger_state.selected_panel.saturating_sub(1);
                        }
                        KeyCode::Right
                            if self.active_tab == Tab::Recordings && self.debugger_state.active =>
                        {
                            // Switch debugger panel focus
                            self.debugger_state.selected_panel =
                                (self.debugger_state.selected_panel + 1).min(2);
                        }

                        _ => {}
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
