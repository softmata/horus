// Terminal UI Monitor for HORUS
use anyhow::Result;
use crossterm::{
    event::{self, Event, KeyCode, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use horus_core::core::{LogType, GLOBAL_LOG_BUFFER};
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

    /// Refresh workspace cache if stale (5 second TTL)
    fn refresh_workspace_cache_if_needed(&mut self) {
        const CACHE_TTL: Duration = Duration::from_secs(5);

        if self.workspace_cache_time.elapsed() > CACHE_TTL {
            self.workspace_cache = get_local_workspaces(&self.current_workspace_path);
            self.workspace_cache_time = Instant::now();
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
            eprintln!("Error: {:?}", err);
        }

        Ok(())
    }

    fn run_app<B: Backend>(&mut self, terminal: &mut Terminal<B>) -> Result<()>
    where
        B::Error: Send + Sync + 'static,
    {
        loop {
            // Update data if not paused (250ms refresh for real-time feel)
            if !self.paused && self.last_update.elapsed() > Duration::from_millis(250) {
                self.update_data()?;
                self.last_update = Instant::now();
            }

            // Draw UI
            terminal.draw(|f| self.draw_ui(f))?;

            // Handle input
            if event::poll(Duration::from_millis(100))? {
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

    fn draw_ui(&mut self, f: &mut Frame) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(4), // Header (increased for status + tabs)
                Constraint::Min(0),    // Content
                Constraint::Length(2), // Footer
            ])
            .split(f.area());

        self.draw_header(f, chunks[0]);

        // Split content area horizontally if log panel is open
        let content_area = chunks[1];
        if self.show_log_panel {
            let horizontal_chunks = Layout::default()
                .direction(Direction::Horizontal)
                .constraints([
                    Constraint::Percentage(25), // Narrow list showing only names
                    Constraint::Percentage(75), // Large log panel
                ])
                .split(content_area);

            // Draw simplified main content (only names)
            if self.show_help {
                self.draw_help(f, horizontal_chunks[0]);
            } else {
                match self.active_tab {
                    Tab::Overview => self.draw_overview(f, horizontal_chunks[0]),
                    Tab::Nodes => self.draw_nodes_simple(f, horizontal_chunks[0]),
                    Tab::Topics => self.draw_topics_simple(f, horizontal_chunks[0]),
                    Tab::Network => self.draw_network(f, horizontal_chunks[0]),
                    Tab::HFrame => self.draw_hframe(f, horizontal_chunks[0]),
                    Tab::Packages => self.draw_packages(f, horizontal_chunks[0]),
                    Tab::Parameters => self.draw_parameters(f, horizontal_chunks[0]),
                    Tab::Recordings => self.draw_recordings(f, horizontal_chunks[0]),
                }
            }

            // Draw log panel
            self.draw_log_panel(f, horizontal_chunks[1]);
        } else {
            // Normal full-width content
            if self.show_help {
                self.draw_help(f, content_area);
            } else {
                match self.active_tab {
                    Tab::Overview => self.draw_overview(f, content_area),
                    Tab::Nodes => self.draw_nodes(f, content_area),
                    Tab::Topics => self.draw_topics(f, content_area),
                    Tab::Network => self.draw_network(f, content_area),
                    Tab::HFrame => self.draw_hframe(f, content_area),
                    Tab::Packages => self.draw_packages(f, content_area),
                    Tab::Parameters => self.draw_parameters(f, content_area),
                    Tab::Recordings => self.draw_recordings(f, content_area),
                }
            }
        }

        self.draw_footer(f, chunks[2]);

        // Draw parameter edit dialog overlay if in edit mode
        if self.param_edit_mode != ParamEditMode::None {
            self.draw_param_edit_dialog(f);
        }
    }

    fn draw_header(&self, f: &mut Frame, area: Rect) {
        // Create a block for the entire header area
        let header_block = Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Blue));

        let inner_area = header_block.inner(area);
        f.render_widget(header_block, area);

        // Split the inner area into status line and tabs
        let header_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(1), // Status line
                Constraint::Length(1), // Tabs
            ])
            .split(inner_area);

        // Draw status line - exclude placeholder entries from count
        let node_count = self.get_active_node_count();
        let topic_count = self.get_active_topic_count();
        let status = if self.paused { "PAUSED" } else { "LIVE" };

        let status_text = vec![
            Span::styled(
                "HORUS TUI ",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw("v0.1.8 | "),
            Span::styled(
                status.to_string(),
                Style::default().fg(if self.paused {
                    Color::Yellow
                } else {
                    Color::Green
                }),
            ),
            Span::raw(" | Nodes: "),
            Span::styled(format!("{}", node_count), Style::default().fg(Color::Green)),
            Span::raw(" | Topics: "),
            Span::styled(format!("{}", topic_count), Style::default().fg(Color::Cyan)),
        ];

        let status_line = Paragraph::new(Line::from(status_text)).alignment(Alignment::Center);
        f.render_widget(status_line, header_chunks[0]);

        // Draw tabs
        let titles: Vec<Line> = Tab::all()
            .iter()
            .map(|t| Line::from(vec![Span::raw(t.as_str())]))
            .collect();

        let selected = Tab::all()
            .iter()
            .position(|&t| t == self.active_tab)
            .unwrap();

        let tabs = Tabs::new(titles)
            .select(selected)
            .style(Style::default().fg(Color::Gray))
            .highlight_style(
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            )
            .divider(Span::raw(" | "));

        f.render_widget(tabs, header_chunks[1]);
    }

    fn draw_overview(&self, f: &mut Frame, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Percentage(50), // Nodes summary
                Constraint::Percentage(50), // Topics summary
            ])
            .split(area);

        // Active Nodes Summary (top 10)
        self.draw_node_summary(f, chunks[0]);

        // Active Topics Summary (top 10)
        self.draw_topic_summary(f, chunks[1]);
    }

    fn draw_node_summary(&self, f: &mut Frame, area: Rect) {
        // Calculate how many rows can fit in the panel
        let available_height = area.height.saturating_sub(3); // Subtract borders and header
        let page_size = available_height as usize;

        let rows: Vec<Row> = self
            .nodes
            .iter()
            .skip(self.scroll_offset)
            .take(page_size)
            .map(|node| {
                let is_running = node.status == "active";
                let status_symbol = if is_running { "●" } else { "○" };
                let status_color = if is_running { Color::Green } else { Color::Red };

                Row::new(vec![
                    Cell::from(status_symbol).style(Style::default().fg(status_color)),
                    Cell::from(node.name.clone()),
                    Cell::from(node.process_id.to_string()),
                    Cell::from(format!("{} MB", node.memory_usage / 1024 / 1024)),
                ])
            })
            .collect();

        let is_focused = self.overview_panel_focus == OverviewPanelFocus::Nodes;
        let border_color = if is_focused {
            Color::Cyan
        } else {
            Color::White
        };

        let widths = [
            Constraint::Length(2),
            Constraint::Min(30),
            Constraint::Length(8),
            Constraint::Length(12),
        ];
        let table = Table::new(rows, widths)
            .header(
                Row::new(vec!["", "Name", "PID", "Memory"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title(format!(
                        "Active Nodes ({}) - Use Left/Right to switch panels",
                        self.get_active_node_count()
                    ))
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(border_color)),
            )
            .row_highlight_style(
                Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol(" ");

        let mut table_state = TableState::default();
        if is_focused && !self.nodes.is_empty() {
            // Highlight the currently selected item within the visible page
            let selected = self.selected_index.min(self.nodes.len() - 1);
            if selected >= self.scroll_offset && selected < self.scroll_offset + page_size {
                table_state.select(Some(selected - self.scroll_offset));
            }
        }

        f.render_stateful_widget(table, area, &mut table_state);
    }

    fn draw_topic_summary(&self, f: &mut Frame, area: Rect) {
        // Calculate how many rows can fit in the panel
        let available_height = area.height.saturating_sub(3); // Subtract borders and header
        let page_size = available_height as usize;

        let rows: Vec<Row> = self
            .topics
            .iter()
            .skip(self.scroll_offset)
            .take(page_size)
            .map(|topic| {
                // Format node names compactly
                let pub_count = topic.publishers;
                let sub_count = topic.subscribers;
                let pub_label = if pub_count > 0 {
                    format!(
                        "{}:{}",
                        pub_count,
                        topic.publisher_nodes.first().unwrap_or(&"-".to_string())
                    )
                } else {
                    "-".to_string()
                };
                let sub_label = if sub_count > 0 {
                    format!(
                        "{}:{}",
                        sub_count,
                        topic.subscriber_nodes.first().unwrap_or(&"-".to_string())
                    )
                } else {
                    "-".to_string()
                };

                Row::new(vec![
                    Cell::from(topic.name.clone()),
                    Cell::from(topic.msg_type.clone()),
                    Cell::from(pub_label).style(Style::default().fg(Color::Green)),
                    Cell::from(sub_label).style(Style::default().fg(Color::Blue)),
                    Cell::from(format!("{:.1} Hz", topic.rate)),
                ])
            })
            .collect();

        let is_focused = self.overview_panel_focus == OverviewPanelFocus::Topics;
        let border_color = if is_focused {
            Color::Cyan
        } else {
            Color::White
        };

        let widths = [
            Constraint::Percentage(30),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
            Constraint::Length(10),
        ];
        let table = Table::new(rows, widths)
            .header(
                Row::new(vec![
                    "Topic",
                    "Type",
                    "Pub (N:Node)",
                    "Sub (N:Node)",
                    "Rate",
                ])
                .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title(format!(
                        "Active Topics ({}) - Use Left/Right to switch panels",
                        self.get_active_topic_count()
                    ))
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(border_color)),
            )
            .row_highlight_style(
                Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol(" ");

        let mut table_state = TableState::default();
        if is_focused && !self.topics.is_empty() {
            // Highlight the currently selected item within the visible page
            let selected = self.selected_index.min(self.topics.len() - 1);
            if selected >= self.scroll_offset && selected < self.scroll_offset + page_size {
                table_state.select(Some(selected - self.scroll_offset));
            }
        }

        f.render_stateful_widget(table, area, &mut table_state);
    }

    fn draw_topics_simple(&self, f: &mut Frame, area: Rect) {
        // Simplified view showing only active topic names (ROS-like)
        let rows: Vec<Row> = self
            .topics
            .iter()
            .map(|topic| {
                // Green for active, yellow for idle (stale topics are filtered out)
                let status_color = match topic.status {
                    crate::discovery::TopicStatus::Active => Color::Green,
                    crate::discovery::TopicStatus::Idle => Color::Yellow,
                    crate::discovery::TopicStatus::Stale => Color::DarkGray, // shouldn't appear
                };

                Row::new(vec![
                    Cell::from("●").style(Style::default().fg(status_color)),
                    Cell::from(topic.name.clone()),
                ])
            })
            .collect();

        let widths = [Constraint::Length(2), Constraint::Min(10)];
        let table = Table::new(rows, widths)
            .header(
                Row::new(vec!["", "Topic Name"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(Block::default().title("Topics").borders(Borders::ALL))
            .row_highlight_style(
                Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol(" ");

        // Create table state with current selection
        let mut table_state = TableState::default();
        if !self.topics.is_empty() {
            let selected = self.selected_index.min(self.topics.len() - 1);
            table_state.select(Some(selected));
        }

        f.render_stateful_widget(table, area, &mut table_state);
    }

    fn draw_topics(&self, f: &mut Frame, area: Rect) {
        let rows: Vec<Row> = self
            .topics
            .iter()
            .map(|topic| {
                // Format publisher and subscriber node names
                let pub_nodes = if topic.publishers == 0 {
                    "-".to_string()
                } else {
                    topic.publisher_nodes.join(", ")
                };

                let sub_nodes = if topic.subscribers == 0 {
                    "-".to_string()
                } else {
                    topic.subscriber_nodes.join(", ")
                };

                Row::new(vec![
                    Cell::from(topic.name.clone()),
                    Cell::from(topic.msg_type.clone()),
                    Cell::from(format!("{:.1}", topic.rate)),
                    Cell::from(pub_nodes).style(Style::default().fg(Color::Green)),
                    Cell::from(sub_nodes).style(Style::default().fg(Color::Blue)),
                ])
            })
            .collect();

        let widths = [
            Constraint::Percentage(25),
            Constraint::Percentage(20),
            Constraint::Length(8),
            Constraint::Percentage(27),
            Constraint::Percentage(28),
        ];
        let table = Table::new(rows, widths)
            .header(
                Row::new(vec!["Topic", "Type", "Hz", "Publishers", "Subscribers"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title("Topics - Use  to select, Enter to view logs")
                    .borders(Borders::ALL),
            )
            .row_highlight_style(
                Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol(" ");

        // Create table state with current selection
        let mut table_state = TableState::default();
        if !self.topics.is_empty() {
            // Clamp selected_index to valid range
            let selected = self.selected_index.min(self.topics.len() - 1);
            table_state.select(Some(selected));
        }

        f.render_stateful_widget(table, area, &mut table_state);
    }

    fn draw_network(&self, f: &mut Frame, area: Rect) {
        let summary = crate::discovery::get_network_summary();

        // Split area into summary panel and details table
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(8), Constraint::Min(5)])
            .split(area);

        // Draw summary panel
        let transport_info: String = if summary.transport_breakdown.is_empty() {
            "No active transports".to_string()
        } else {
            summary
                .transport_breakdown
                .iter()
                .map(|(t, c)| format!("{}: {}", t, c))
                .collect::<Vec<_>>()
                .join(" | ")
        };

        let summary_text = vec![
            Line::from(vec![
                Span::styled("Active Nodes: ", Style::default().fg(Color::Cyan)),
                Span::styled(
                    format!("{}", summary.total_nodes),
                    Style::default()
                        .fg(Color::Green)
                        .add_modifier(Modifier::BOLD),
                ),
            ]),
            Line::from(vec![
                Span::styled("Transports: ", Style::default().fg(Color::Cyan)),
                Span::raw(transport_info),
            ]),
            Line::from(vec![
                Span::styled("Bytes Sent: ", Style::default().fg(Color::Cyan)),
                Span::styled(
                    format_bytes(summary.total_bytes_sent),
                    Style::default().fg(Color::Yellow),
                ),
                Span::raw(" | "),
                Span::styled("Received: ", Style::default().fg(Color::Cyan)),
                Span::styled(
                    format_bytes(summary.total_bytes_received),
                    Style::default().fg(Color::Yellow),
                ),
            ]),
            Line::from(vec![
                Span::styled("Packets Sent: ", Style::default().fg(Color::Cyan)),
                Span::styled(
                    format!("{}", summary.total_packets_sent),
                    Style::default().fg(Color::Magenta),
                ),
                Span::raw(" | "),
                Span::styled("Received: ", Style::default().fg(Color::Cyan)),
                Span::styled(
                    format!("{}", summary.total_packets_received),
                    Style::default().fg(Color::Magenta),
                ),
            ]),
            Line::from(vec![
                Span::styled("Endpoints: ", Style::default().fg(Color::Cyan)),
                Span::raw(if summary.unique_endpoints.is_empty() {
                    "None discovered".to_string()
                } else {
                    summary.unique_endpoints.join(", ")
                }),
            ]),
        ];

        let summary_paragraph = Paragraph::new(summary_text).block(
            Block::default()
                .title("Network Summary")
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Blue)),
        );

        f.render_widget(summary_paragraph, chunks[0]);

        // Draw node network status table
        let rows: Vec<Row> = summary
            .node_statuses
            .iter()
            .map(|status| {
                let transport_color = match status.transport_type.as_str() {
                    "SharedMemory" => Color::Green,
                    "BatchUdp" | "Udp" => Color::Cyan,
                    "Quic" => Color::Magenta,
                    "UnixSocket" => Color::Yellow,
                    "IoUring" => Color::LightGreen,
                    _ => Color::White,
                };

                let endpoints = if status.remote_endpoints.is_empty() {
                    "-".to_string()
                } else {
                    status.remote_endpoints.join(", ")
                };

                let topics_pub = if status.network_topics_pub.is_empty() {
                    "-".to_string()
                } else {
                    status.network_topics_pub.join(", ")
                };

                Row::new(vec![
                    Cell::from(status.node_name.clone()),
                    Cell::from(status.transport_type.clone())
                        .style(Style::default().fg(transport_color)),
                    Cell::from(
                        status
                            .local_endpoint
                            .clone()
                            .unwrap_or_else(|| "-".to_string()),
                    ),
                    Cell::from(endpoints),
                    Cell::from(topics_pub),
                    Cell::from(format_bytes(status.bytes_sent)),
                    Cell::from(format_bytes(status.bytes_received)),
                ])
            })
            .collect();

        let widths = [
            Constraint::Percentage(15),
            Constraint::Length(12),
            Constraint::Percentage(15),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
            Constraint::Length(10),
            Constraint::Length(10),
        ];

        let table = Table::new(rows, widths)
            .header(
                Row::new(vec![
                    "Node",
                    "Transport",
                    "Local",
                    "Remote",
                    "Topics",
                    "Sent",
                    "Recv",
                ])
                .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title("Node Network Status")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Blue)),
            )
            .row_highlight_style(
                Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            );

        f.render_widget(table, chunks[1]);
    }

    /// Draw the HFrame (coordinate transform) visualization
    fn draw_hframe(&self, f: &mut Frame, area: Rect) {
        use crate::commands::hf::HFrameReader;
        use std::collections::{HashMap, HashSet};

        // Read live frame data from shared memory
        let mut reader = HFrameReader::new();
        let has_data = reader.read_from_shm();

        // Split into summary and frame tree
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Length(10), Constraint::Min(10)])
            .split(area);

        // Draw HFrame summary panel
        let summary_text = if !has_data || reader.frame_data.is_empty() {
            vec![
                Line::from(vec![
                    Span::styled("Status: ", Style::default().fg(Color::Cyan)),
                    Span::styled("No active transforms", Style::default().fg(Color::Yellow)),
                ]),
                Line::from(""),
                Line::from(vec![
                    Span::styled(
                        "Tip: ",
                        Style::default()
                            .fg(Color::Gray)
                            .add_modifier(Modifier::ITALIC),
                    ),
                    Span::styled(
                        "Start a HORUS application with HFrame publishing enabled",
                        Style::default().fg(Color::Gray),
                    ),
                ]),
                Line::from(vec![Span::styled(
                    "     Use sim3d or add HFramePublisher to your nodes",
                    Style::default().fg(Color::Gray),
                )]),
                Line::from(""),
                Line::from(vec![Span::styled(
                    "CLI: horus hf list | horus hf tree | horus hf echo <src> <dst>",
                    Style::default().fg(Color::DarkGray),
                )]),
            ]
        } else {
            let all_frames = reader.get_all_frames();
            let static_count = reader.frame_data.values().filter(|d| d.is_static).count();
            let dynamic_count = reader.frame_data.len() - static_count;

            let mut lines = vec![
                Line::from(vec![
                    Span::styled("Frames: ", Style::default().fg(Color::Cyan)),
                    Span::styled(
                        format!("{}", all_frames.len()),
                        Style::default()
                            .fg(Color::Green)
                            .add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(" (", Style::default().fg(Color::Gray)),
                    Span::styled(
                        format!("{} static", static_count),
                        Style::default().fg(Color::Blue),
                    ),
                    Span::styled(", ", Style::default().fg(Color::Gray)),
                    Span::styled(
                        format!("{} dynamic", dynamic_count),
                        Style::default().fg(Color::Yellow),
                    ),
                    Span::styled(")", Style::default().fg(Color::Gray)),
                ]),
                Line::from(vec![
                    Span::styled("Transforms: ", Style::default().fg(Color::Cyan)),
                    Span::styled(
                        format!("{}", reader.frame_data.len()),
                        Style::default()
                            .fg(Color::Green)
                            .add_modifier(Modifier::BOLD),
                    ),
                ]),
            ];

            // Show some frame examples
            lines.push(Line::from(""));
            lines.push(Line::from(vec![Span::styled(
                "Recent transforms:",
                Style::default().fg(Color::Gray),
            )]));

            for (child, data) in reader.frame_data.iter().take(4) {
                let type_marker = if data.is_static {
                    Span::styled("[S]", Style::default().fg(Color::Blue))
                } else {
                    Span::styled("[D]", Style::default().fg(Color::Yellow))
                };
                lines.push(Line::from(vec![
                    Span::raw("  "),
                    type_marker,
                    Span::raw(" "),
                    Span::styled(&data.parent, Style::default().fg(Color::Cyan)),
                    Span::styled(" → ", Style::default().fg(Color::Gray)),
                    Span::styled(child, Style::default().fg(Color::White)),
                ]));
            }

            if reader.frame_data.len() > 4 {
                lines.push(Line::from(vec![Span::styled(
                    format!("  ... and {} more transforms", reader.frame_data.len() - 4),
                    Style::default().fg(Color::DarkGray),
                )]));
            }

            lines
        };

        let summary_paragraph = Paragraph::new(summary_text).block(
            Block::default()
                .title("HFrame - Coordinate Transforms")
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Magenta)),
        );

        f.render_widget(summary_paragraph, chunks[0]);

        // Draw actual frame tree from live data
        let mut tree_text = vec![
            Line::from(vec![Span::styled(
                "Frame Tree:",
                Style::default()
                    .fg(Color::Cyan)
                    .add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
        ];

        if reader.frame_data.is_empty() {
            tree_text.push(Line::from(vec![Span::styled(
                "  (No frames available)",
                Style::default()
                    .fg(Color::DarkGray)
                    .add_modifier(Modifier::ITALIC),
            )]));
        } else {
            // Build tree structure
            let mut tree: HashMap<String, Vec<String>> = HashMap::new();
            let mut all_children: HashSet<String> = HashSet::new();

            for (child, data) in &reader.frame_data {
                tree.entry(data.parent.clone())
                    .or_default()
                    .push(child.clone());
                all_children.insert(child.clone());
            }

            // Sort children
            for children in tree.values_mut() {
                children.sort();
            }

            // Find root frames (frames that are parents but not children)
            let mut roots: Vec<String> = tree
                .keys()
                .filter(|p| !all_children.contains(*p))
                .cloned()
                .collect();
            roots.sort();

            // Helper function to recursively build tree lines
            fn build_tree_lines(
                frame: &str,
                tree: &HashMap<String, Vec<String>>,
                frame_data: &HashMap<String, crate::commands::hf::FrameData>,
                prefix: &str,
                is_last: bool,
                lines: &mut Vec<Line<'static>>,
            ) {
                let connector = if is_last { "└── " } else { "├── " };
                let child_prefix = if is_last { "    " } else { "│   " };

                // Determine color based on frame type
                let is_static = frame_data.get(frame).map(|d| d.is_static).unwrap_or(false);
                let frame_color = if is_static { Color::Blue } else { Color::Cyan };

                lines.push(Line::from(vec![
                    Span::styled(prefix.to_string(), Style::default().fg(Color::DarkGray)),
                    Span::styled(connector, Style::default().fg(Color::DarkGray)),
                    Span::styled(frame.to_string(), Style::default().fg(frame_color)),
                ]));

                if let Some(children) = tree.get(frame) {
                    for (i, child) in children.iter().enumerate() {
                        let new_prefix = format!("{}{}", prefix, child_prefix);
                        let is_last_child = i == children.len() - 1;
                        build_tree_lines(
                            child,
                            tree,
                            frame_data,
                            &new_prefix,
                            is_last_child,
                            lines,
                        );
                    }
                }
            }

            // Build tree starting from roots
            for (i, root) in roots.iter().enumerate() {
                // Draw root frame
                tree_text.push(Line::from(vec![Span::styled(
                    format!("  {}", root),
                    Style::default()
                        .fg(Color::White)
                        .add_modifier(Modifier::BOLD),
                )]));

                // Draw children
                if let Some(children) = tree.get(root) {
                    for (j, child) in children.iter().enumerate() {
                        let is_last = j == children.len() - 1;
                        build_tree_lines(
                            child,
                            &tree,
                            &reader.frame_data,
                            "  ",
                            is_last,
                            &mut tree_text,
                        );
                    }
                }

                // Add spacing between root trees
                if i < roots.len() - 1 {
                    tree_text.push(Line::from(""));
                }
            }
        }

        let tree_paragraph = Paragraph::new(tree_text).block(
            Block::default()
                .title("Frame Tree (Live)")
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Blue)),
        );

        f.render_widget(tree_paragraph, chunks[1]);
    }

    fn draw_packages(&mut self, f: &mut Frame, area: Rect) {
        match self.package_view_mode {
            PackageViewMode::List => self.draw_workspace_list(f, area),
            PackageViewMode::WorkspaceDetails => self.draw_workspace_details(f, area),
        }
    }

    fn draw_workspace_list(&mut self, f: &mut Frame, area: Rect) {
        // Refresh workspace cache if needed (5 second TTL instead of every frame)
        self.refresh_workspace_cache_if_needed();

        let workspaces = &self.workspace_cache;
        let (_, global_packages) = get_installed_packages();

        // Split the area into two sections: workspaces (top) and global (bottom)
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Percentage(50), // Local workspaces
                Constraint::Percentage(50), // Global packages
            ])
            .split(area);

        // Determine which panel is focused
        let local_focused = self.package_panel_focus == PackagePanelFocus::LocalWorkspaces;
        let global_focused = self.package_panel_focus == PackagePanelFocus::GlobalPackages;

        // Draw workspaces table
        let workspace_rows: Vec<Row> = workspaces
            .iter()
            .enumerate()
            .map(|(idx, workspace)| {
                let is_selected = local_focused && idx == self.selected_index;

                // Build workspace name with current marker
                let workspace_display = if workspace.is_current {
                    format!("> {} (current)", workspace.name)
                } else {
                    workspace.name.clone()
                };

                // Style: selected gets reversed, current workspace gets green color
                let style = if is_selected {
                    Style::default().add_modifier(Modifier::REVERSED)
                } else if workspace.is_current {
                    Style::default()
                        .fg(Color::Green)
                        .add_modifier(Modifier::BOLD)
                } else {
                    Style::default()
                };

                // Format package/dependency counts
                let pkg_count = workspace.packages.len();
                let missing_count = workspace.dependencies.len();
                let count_display = if missing_count > 0 {
                    format!("{} ({} missing)", pkg_count, missing_count)
                } else {
                    pkg_count.to_string()
                };

                Row::new(vec![
                    Cell::from(workspace_display),
                    Cell::from(count_display),
                    Cell::from(workspace.path.clone()),
                ])
                .style(style)
            })
            .collect();

        let workspace_widths = [
            Constraint::Length(25),
            Constraint::Length(10),
            Constraint::Min(30),
        ];
        let workspace_table = Table::new(workspace_rows, workspace_widths)
            .header(
                Row::new(vec!["Workspace", "Pkgs (Missing)", "Path"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title(format!(
                        "Local Workspaces ({}) {}",
                        workspaces.len(),
                        if local_focused {
                            "[FOCUSED - Press ← →]"
                        } else {
                            "[Press → to focus]"
                        }
                    ))
                    .borders(Borders::ALL)
                    .border_style(if local_focused {
                        Style::default().fg(Color::Yellow)
                    } else {
                        Style::default().fg(Color::DarkGray)
                    }),
            )
            .row_highlight_style(Style::default().add_modifier(Modifier::REVERSED));

        // Add TableState for scrolling support
        let mut workspace_state = TableState::default();
        if local_focused && !workspaces.is_empty() {
            let selected = self.selected_index.min(workspaces.len() - 1);
            workspace_state.select(Some(selected));
        }

        f.render_stateful_widget(workspace_table, chunks[0], &mut workspace_state);

        // Draw global packages table with selection support
        let global_rows: Vec<Row> = global_packages
            .iter()
            .enumerate()
            .map(|(idx, (name, version, size))| {
                let is_selected = global_focused && idx == self.selected_index;
                let style = if is_selected {
                    Style::default().add_modifier(Modifier::REVERSED)
                } else {
                    Style::default()
                };

                Row::new(vec![
                    Cell::from(name.clone()),
                    Cell::from(version.clone()),
                    Cell::from(size.clone()),
                ])
                .style(style)
            })
            .collect();

        let global_widths = [
            Constraint::Min(30),
            Constraint::Length(15),
            Constraint::Length(12),
        ];
        let global_table = Table::new(global_rows, global_widths)
            .header(
                Row::new(vec!["Package", "Version", "Size"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title(format!(
                        "Global Packages ({}) {}",
                        global_packages.len(),
                        if global_focused {
                            "[FOCUSED - Press ← →]"
                        } else {
                            "[Press ← to focus]"
                        }
                    ))
                    .borders(Borders::ALL)
                    .border_style(if global_focused {
                        Style::default().fg(Color::Cyan)
                    } else {
                        Style::default().fg(Color::DarkGray)
                    }),
            )
            .row_highlight_style(Style::default().add_modifier(Modifier::REVERSED));

        let mut global_state = TableState::default();
        if global_focused && !global_packages.is_empty() {
            let selected = self.selected_index.min(global_packages.len() - 1);
            global_state.select(Some(selected));
        }
        f.render_stateful_widget(global_table, chunks[1], &mut global_state);
    }

    fn draw_workspace_details(&self, f: &mut Frame, area: Rect) {
        if let Some(ref workspace) = self.selected_workspace {
            // Split area into two sections: Installed Packages and Missing Dependencies
            let has_missing = !workspace.dependencies.is_empty();

            let chunks = if has_missing {
                Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Percentage(60), // Installed packages
                        Constraint::Percentage(40), // Missing dependencies
                    ])
                    .split(area)
            } else {
                // Create a single-element slice for consistency
                use std::rc::Rc;
                Rc::from(vec![area])
            };

            // Display installed packages
            let package_rows: Vec<Row> = workspace
                .packages
                .iter()
                .enumerate()
                .map(|(idx, pkg)| {
                    let is_selected = idx == self.selected_index;
                    let style = if is_selected {
                        Style::default().add_modifier(Modifier::REVERSED)
                    } else {
                        Style::default()
                    };

                    // Format nested packages as a comma-separated list
                    let installed = if pkg.installed_packages.is_empty() {
                        "-".to_string()
                    } else {
                        pkg.installed_packages
                            .iter()
                            .map(|(name, _)| name.clone())
                            .collect::<Vec<_>>()
                            .join(", ")
                    };

                    Row::new(vec![
                        Cell::from(pkg.name.clone()).style(Style::default().fg(Color::Green)),
                        Cell::from(pkg.version.clone()),
                        Cell::from(pkg.installed_packages.len().to_string()),
                        Cell::from(installed),
                    ])
                    .style(style)
                })
                .collect();

            let package_widths = [
                Constraint::Length(25),
                Constraint::Length(12),
                Constraint::Length(6),
                Constraint::Min(30),
            ];
            let package_table = Table::new(package_rows, package_widths)
                .header(
                    Row::new(vec!["Package", "Version", "Deps", "Installed Packages"])
                        .style(Style::default().add_modifier(Modifier::BOLD)),
                )
                .block(
                    Block::default()
                        .title(format!(
                            "Workspace: {} - Installed Packages ({}) - Press Esc to return",
                            workspace.name,
                            workspace.packages.len()
                        ))
                        .borders(Borders::ALL)
                        .border_style(Style::default().fg(Color::Green)),
                );

            f.render_widget(package_table, chunks[0]);

            // Display missing dependencies if any
            if has_missing {
                let dep_rows: Vec<Row> = workspace
                    .dependencies
                    .iter()
                    .map(|dep| {
                        Row::new(vec![
                            Cell::from(dep.name.clone()).style(Style::default().fg(Color::Yellow)),
                            Cell::from(dep.declared_version.clone()),
                            Cell::from("MISSING").style(Style::default().fg(Color::Red)),
                        ])
                    })
                    .collect();

                let dep_widths = [
                    Constraint::Length(25),
                    Constraint::Length(30),
                    Constraint::Min(15),
                ];
                let dep_table = Table::new(dep_rows, dep_widths)
                    .header(
                        Row::new(vec!["Package", "Declared (horus.yaml)", "Status"])
                            .style(Style::default().add_modifier(Modifier::BOLD)),
                    )
                    .block(
                        Block::default()
                            .title(format!(
                                "Missing Dependencies ({}) - Run 'horus run' to install",
                                workspace.dependencies.len()
                            ))
                            .borders(Borders::ALL)
                            .border_style(Style::default().fg(Color::Red)),
                    );

                f.render_widget(dep_table, chunks[1]);
            }
        } else {
            // Fallback: No workspace selected
            let block = Block::default()
                .title("No workspace selected - Press Esc to return")
                .borders(Borders::ALL);
            f.render_widget(block, area);
        }
    }

    fn draw_parameters(&self, f: &mut Frame, area: Rect) {
        // Get REAL runtime parameters from RuntimeParams
        let params_map = self.params.get_all();

        let params: Vec<_> = params_map
            .iter()
            .map(|(key, value)| {
                // Determine type from value using string matching to avoid version conflicts
                let type_str = if value.is_number() {
                    "number"
                } else if value.is_string() {
                    "string"
                } else if value.is_boolean() {
                    "bool"
                } else if value.is_array() {
                    "array"
                } else if value.is_object() {
                    "object"
                } else {
                    "null"
                };

                // Format value for display
                let value_str = if let Some(s) = value.as_str() {
                    s.to_string()
                } else {
                    value.to_string()
                };

                (key.clone(), value_str, type_str.to_string())
            })
            .collect();

        let rows = params
            .iter()
            .enumerate()
            .map(|(idx, (name, value, type_))| {
                let is_selected = idx == self.selected_index && self.active_tab == Tab::Parameters;
                let style = if is_selected {
                    Style::default().add_modifier(Modifier::REVERSED)
                } else {
                    Style::default()
                };

                Row::new(vec![
                    Cell::from(name.clone()).style(Style::default().fg(Color::Cyan)),
                    Cell::from(value.clone()),
                    Cell::from(type_.clone()).style(Style::default().fg(Color::Yellow)),
                ])
                .style(style)
            });

        let help_text = if params.is_empty() {
            "No parameters set. Press 'a' to add"
        } else {
            "[a] Add | [e] Edit | [d] Delete | [s] Save | [r] Refresh"
        };

        let widths = [
            Constraint::Percentage(35),
            Constraint::Percentage(50),
            Constraint::Percentage(15),
        ];
        let table = Table::new(rows, widths)
            .header(
                Row::new(vec!["Parameter", "Value", "Type"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title(format!(
                        "Runtime Parameters ({}) - {}",
                        params.len(),
                        help_text
                    ))
                    .borders(Borders::ALL),
            );

        f.render_widget(table, area);
    }

    fn draw_recordings(&self, f: &mut Frame, area: Rect) {
        if self.debugger_state.active {
            // Show visual time-travel debugger
            self.draw_time_travel_debugger(f, area);
        } else {
            // Show recordings list
            self.draw_recordings_list(f, area);
        }
    }

    /// Draw the visual time-travel debugger interface
    fn draw_time_travel_debugger(&self, f: &mut Frame, area: Rect) {
        // Layout: Timeline at top, Data panel + Watches below, Controls at bottom
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(8), // Timeline + progress
                Constraint::Min(10),   // Data panels
                Constraint::Length(4), // Controls
            ])
            .split(area);

        // === Timeline Section ===
        let timeline_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Session info
                Constraint::Length(3), // Visual timeline
                Constraint::Length(2), // Tick info
            ])
            .split(chunks[0]);

        // Session info header
        let session_name = self
            .debugger_state
            .selected_session
            .as_deref()
            .unwrap_or("Unknown");
        let playback_indicator = match self.debugger_state.playback {
            PlaybackState::Playing => "▶ PLAYING",
            PlaybackState::Paused => "⏸ PAUSED",
            PlaybackState::Stopped => "⏹ STOPPED",
            PlaybackState::SteppingForward => "⏩ STEP FWD",
            PlaybackState::SteppingBackward => "⏪ STEP BWD",
        };
        let playback_color = match self.debugger_state.playback {
            PlaybackState::Playing => Color::Green,
            PlaybackState::Paused => Color::Yellow,
            _ => Color::Cyan,
        };

        let session_info = Paragraph::new(Line::from(vec![
            Span::styled("Session: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(session_name, Style::default().fg(Color::Cyan)),
            Span::raw("  |  "),
            Span::styled(
                playback_indicator,
                Style::default()
                    .fg(playback_color)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw("  |  "),
            Span::styled(
                format!("Speed: {:.2}x", self.debugger_state.playback_speed),
                Style::default().fg(Color::Magenta),
            ),
        ]))
        .block(
            Block::default()
                .title("Time-Travel Debugger")
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Magenta)),
        );
        f.render_widget(session_info, timeline_chunks[0]);

        // Visual timeline bar
        self.render_timeline_bar(f, timeline_chunks[1]);

        // Tick info
        let progress = if self.debugger_state.total_ticks > 0 {
            (self.debugger_state.current_tick as f64 / self.debugger_state.total_ticks as f64
                * 100.0) as u8
        } else {
            0
        };
        let tick_info = Paragraph::new(Line::from(vec![
            Span::styled("Tick: ", Style::default().add_modifier(Modifier::BOLD)),
            Span::styled(
                format!("{}", self.debugger_state.current_tick),
                Style::default()
                    .fg(Color::Green)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw(format!(" / {} ", self.debugger_state.total_ticks)),
            Span::styled(format!("({}%)", progress), Style::default().fg(Color::Cyan)),
            Span::raw("  |  "),
            Span::styled(
                "Breakpoints: ",
                Style::default().add_modifier(Modifier::BOLD),
            ),
            Span::styled(
                format!("{}", self.debugger_state.breakpoints.len()),
                Style::default().fg(Color::Red),
            ),
        ]))
        .alignment(Alignment::Center);
        f.render_widget(tick_info, timeline_chunks[2]);

        // === Data Panels ===
        let data_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Percentage(60), // Node data
                Constraint::Percentage(40), // Watches + Breakpoints
            ])
            .split(chunks[1]);

        // Node data panel (left)
        let panel_border = if self.debugger_state.selected_panel == 0 {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::Gray)
        };

        let data_content = vec![
            Line::from(vec![
                Span::styled(
                    "Node States at Tick ",
                    Style::default().add_modifier(Modifier::BOLD),
                ),
                Span::styled(
                    format!("{}", self.debugger_state.current_tick),
                    Style::default().fg(Color::Green),
                ),
            ]),
            Line::from(""),
            Line::from(vec![
                Span::styled("  sensor_imu: ", Style::default().fg(Color::Cyan)),
                Span::raw("{ accel: [0.1, 9.8, 0.0], gyro: [0.0, 0.0, 0.0] }"),
            ]),
            Line::from(vec![
                Span::styled("  sensor_camera: ", Style::default().fg(Color::Cyan)),
                Span::raw("{ frame_id: 1234, timestamp: 1234567890 }"),
            ]),
            Line::from(vec![
                Span::styled("  controller: ", Style::default().fg(Color::Cyan)),
                Span::raw("{ mode: 'AUTO', target: [10.0, 5.0] }"),
            ]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Topics Published:",
                Style::default().add_modifier(Modifier::BOLD),
            )]),
            Line::from("  /sensor/imu  /sensor/camera  /control/cmd"),
        ];

        let data_panel = Paragraph::new(data_content).block(
            Block::default()
                .title("Node Data")
                .borders(Borders::ALL)
                .border_style(panel_border),
        );
        f.render_widget(data_panel, data_chunks[0]);

        // Watches + Breakpoints panel (right)
        let right_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Percentage(50), // Watches
                Constraint::Percentage(50), // Breakpoints
            ])
            .split(data_chunks[1]);

        let watches_border = if self.debugger_state.selected_panel == 1 {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::Gray)
        };

        let watches_content: Vec<Line> = if self.debugger_state.watches.is_empty() {
            vec![
                Line::from(""),
                Line::from(vec![Span::styled(
                    "  No watches configured",
                    Style::default().fg(Color::DarkGray),
                )]),
                Line::from(""),
                Line::from(vec![Span::styled(
                    "  Add via API: ",
                    Style::default().fg(Color::DarkGray),
                )]),
                Line::from(vec![Span::styled(
                    "  POST /api/debug/watch",
                    Style::default().fg(Color::Cyan),
                )]),
            ]
        } else {
            self.debugger_state
                .watches
                .iter()
                .map(|w| {
                    Line::from(vec![
                        Span::styled("  ", Style::default()),
                        Span::styled(w, Style::default().fg(Color::Cyan)),
                        Span::raw(": "),
                        Span::styled("<value>", Style::default().fg(Color::Green)),
                    ])
                })
                .collect()
        };

        let watches_panel = Paragraph::new(watches_content).block(
            Block::default()
                .title("Watches")
                .borders(Borders::ALL)
                .border_style(watches_border),
        );
        f.render_widget(watches_panel, right_chunks[0]);

        let breakpoints_border = if self.debugger_state.selected_panel == 2 {
            Style::default()
                .fg(Color::Yellow)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::Gray)
        };

        let breakpoints_content: Vec<Line> = if self.debugger_state.breakpoints.is_empty() {
            vec![
                Line::from(""),
                Line::from(vec![Span::styled(
                    "  No breakpoints set",
                    Style::default().fg(Color::DarkGray),
                )]),
                Line::from(vec![Span::styled(
                    "  Press 'b' to toggle",
                    Style::default().fg(Color::DarkGray),
                )]),
            ]
        } else {
            self.debugger_state
                .breakpoints
                .iter()
                .map(|&tick| {
                    let is_current = tick == self.debugger_state.current_tick;
                    let style = if is_current {
                        Style::default()
                            .fg(Color::Yellow)
                            .add_modifier(Modifier::BOLD)
                    } else {
                        Style::default().fg(Color::Red)
                    };
                    Line::from(vec![
                        Span::styled("  ● ", style),
                        Span::styled(format!("Tick {}", tick), style),
                        if is_current {
                            Span::styled(" ← current", Style::default().fg(Color::Yellow))
                        } else {
                            Span::raw("")
                        },
                    ])
                })
                .collect()
        };

        let breakpoints_panel = Paragraph::new(breakpoints_content).block(
            Block::default()
                .title("Breakpoints")
                .borders(Borders::ALL)
                .border_style(breakpoints_border),
        );
        f.render_widget(breakpoints_panel, right_chunks[1]);

        // === Controls ===
        let controls = vec![
            Line::from(vec![
                Span::styled("[Space]", Style::default().fg(Color::Yellow)),
                Span::raw(" Play/Pause  "),
                Span::styled("[</>]", Style::default().fg(Color::Yellow)),
                Span::raw(" Step  "),
                Span::styled("[b]", Style::default().fg(Color::Yellow)),
                Span::raw(" Breakpoint  "),
                Span::styled("[Home/End]", Style::default().fg(Color::Yellow)),
                Span::raw(" Jump  "),
                Span::styled("[/]]", Style::default().fg(Color::Yellow)),
                Span::raw(" Speed  "),
                Span::styled("[Esc]", Style::default().fg(Color::Yellow)),
                Span::raw(" Exit"),
            ]),
            Line::from(vec![
                Span::styled("[←/→]", Style::default().fg(Color::Cyan)),
                Span::raw(" Switch Panel  "),
                Span::styled("[↑/↓]", Style::default().fg(Color::Cyan)),
                Span::raw(" Navigate"),
            ]),
        ];

        let controls_block = Paragraph::new(controls)
            .alignment(Alignment::Center)
            .block(Block::default().title("Controls").borders(Borders::ALL));
        f.render_widget(controls_block, chunks[2]);
    }

    /// Render the visual timeline bar with breakpoint markers
    fn render_timeline_bar(&self, f: &mut Frame, area: Rect) {
        let inner = area.inner(ratatui::layout::Margin {
            horizontal: 1,
            vertical: 0,
        });
        let width = inner.width as usize;

        if width < 10 || self.debugger_state.total_ticks == 0 {
            return;
        }

        // Build timeline string
        let mut timeline = String::with_capacity(width);
        let current_pos = (self.debugger_state.current_tick as f64
            / self.debugger_state.total_ticks as f64
            * width as f64) as usize;

        for i in 0..width {
            let tick_at_pos =
                (i as f64 / width as f64 * self.debugger_state.total_ticks as f64) as u64;
            let is_breakpoint = self.debugger_state.breakpoints.contains(&tick_at_pos);
            let is_current = i == current_pos;

            if is_current {
                timeline.push('▼');
            } else if is_breakpoint {
                timeline.push('●');
            } else if i == 0 {
                timeline.push('├');
            } else if i == width - 1 {
                timeline.push('┤');
            } else {
                timeline.push('─');
            }
        }

        // Color the timeline
        let mut spans = Vec::new();
        for (i, ch) in timeline.chars().enumerate() {
            let tick_at_pos =
                (i as f64 / width as f64 * self.debugger_state.total_ticks as f64) as u64;
            let style = if i == current_pos {
                Style::default()
                    .fg(Color::Green)
                    .add_modifier(Modifier::BOLD)
            } else if self.debugger_state.breakpoints.contains(&tick_at_pos) {
                Style::default().fg(Color::Red)
            } else if (i as f64 / width as f64 * self.debugger_state.total_ticks as f64) as u64
                <= self.debugger_state.current_tick
            {
                Style::default().fg(Color::Blue)
            } else {
                Style::default().fg(Color::DarkGray)
            };
            spans.push(Span::styled(ch.to_string(), style));
        }

        let timeline_widget = Paragraph::new(Line::from(spans))
            .alignment(Alignment::Center)
            .block(Block::default().borders(Borders::NONE));
        f.render_widget(timeline_widget, inner);
    }

    /// Draw the recordings list (when debugger is not active)
    fn draw_recordings_list(&self, f: &mut Frame, area: Rect) {
        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Percentage(60), // Recordings list
                Constraint::Percentage(25), // Debug info
                Constraint::Percentage(15), // Controls
            ])
            .split(area);

        // Use cached recordings
        let recordings = &self.debugger_state.recordings_cache;

        let rows: Vec<Row> = recordings
            .iter()
            .enumerate()
            .map(|(idx, info)| {
                let style = if idx == self.selected_index {
                    Style::default().add_modifier(Modifier::REVERSED)
                } else {
                    Style::default()
                };

                let type_indicator = match info.recording_type {
                    RecordingType::ZeroCopy => "⚡",    // Fast zero-copy
                    RecordingType::Distributed => "🌐", // Distributed/fleet
                    RecordingType::Standard => "📁",    // Standard
                };

                let type_color = match info.recording_type {
                    RecordingType::ZeroCopy => Color::Green,
                    RecordingType::Distributed => Color::Cyan,
                    RecordingType::Standard => Color::Yellow,
                };

                let size_str = if info.size_bytes > 1024 * 1024 {
                    format!("{:.1} MB", info.size_bytes as f64 / 1024.0 / 1024.0)
                } else if info.size_bytes > 1024 {
                    format!("{:.1} KB", info.size_bytes as f64 / 1024.0)
                } else {
                    format!("{} B", info.size_bytes)
                };

                let ticks_str = info
                    .total_ticks
                    .map(|t| format!("{} ticks", t))
                    .unwrap_or_else(|| "? ticks".to_string());

                Row::new(vec![
                    Cell::from(type_indicator).style(Style::default().fg(type_color)),
                    Cell::from(info.session_name.clone()).style(Style::default().fg(Color::Cyan)),
                    Cell::from(format!("{} files", info.file_count)),
                    Cell::from(size_str).style(Style::default().fg(Color::Yellow)),
                    Cell::from(ticks_str).style(Style::default().fg(Color::Green)),
                ])
                .style(style)
            })
            .collect();

        let widths = [
            Constraint::Length(3),
            Constraint::Percentage(35),
            Constraint::Percentage(15),
            Constraint::Percentage(20),
            Constraint::Percentage(20),
        ];

        let recordings_table = Table::new(rows, widths)
            .header(
                Row::new(vec!["", "Session", "Files", "Size", "Ticks"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title(format!(
                        "Recording Sessions ({}) - Press Enter to Debug",
                        recordings.len()
                    ))
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Cyan)),
            );

        f.render_widget(recordings_table, chunks[0]);

        // Recording type legend + features
        let legend_info = vec![
            Line::from(vec![Span::styled(
                "Recording Types: ",
                Style::default().add_modifier(Modifier::BOLD),
            )]),
            Line::from(vec![
                Span::styled("  ⚡ ZeroCopy", Style::default().fg(Color::Green)),
                Span::raw(" - Memory-mapped, fastest replay"),
            ]),
            Line::from(vec![
                Span::styled("  🌐 Distributed", Style::default().fg(Color::Cyan)),
                Span::raw(" - Multi-robot fleet recording"),
            ]),
            Line::from(vec![
                Span::styled("  📁 Standard", Style::default().fg(Color::Yellow)),
                Span::raw(" - Portable JSON format"),
            ]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Debugger Features: ",
                Style::default().add_modifier(Modifier::BOLD),
            )]),
            Line::from("  • Step forward/backward through execution"),
            Line::from("  • Set breakpoints at specific ticks"),
            Line::from("  • Watch expressions for values"),
        ];

        let legend_block = Paragraph::new(legend_info).block(
            Block::default()
                .title("Info")
                .borders(Borders::ALL)
                .border_style(Style::default().fg(Color::Yellow)),
        );

        f.render_widget(legend_block, chunks[1]);

        // Controls
        let controls = vec![Line::from(vec![
            Span::styled("[Enter]", Style::default().fg(Color::Yellow)),
            Span::raw(" Load into Debugger  "),
            Span::styled("[↑/↓]", Style::default().fg(Color::Yellow)),
            Span::raw(" Navigate  "),
            Span::styled("[Tab]", Style::default().fg(Color::Yellow)),
            Span::raw(" Switch Tab"),
        ])];

        let controls_block = Paragraph::new(controls)
            .alignment(Alignment::Center)
            .block(Block::default().title("Controls").borders(Borders::ALL));

        f.render_widget(controls_block, chunks[2]);
    }

    fn draw_nodes_simple(&self, f: &mut Frame, area: Rect) {
        // Simplified view showing only node names
        let rows: Vec<Row> = self
            .nodes
            .iter()
            .map(|node| {
                let is_running = node.status == "active";
                let status_symbol = if is_running { "●" } else { "○" };
                let status_color = if is_running { Color::Green } else { Color::Red };

                Row::new(vec![
                    Cell::from(status_symbol).style(Style::default().fg(status_color)),
                    Cell::from(node.name.clone()),
                ])
            })
            .collect();

        let widths = [Constraint::Length(2), Constraint::Min(10)];
        let table = Table::new(rows, widths)
            .header(
                Row::new(vec!["", "Node Name"])
                    .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(Block::default().title("Nodes").borders(Borders::ALL))
            .row_highlight_style(
                Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol(" ");

        // Create table state with current selection
        let mut table_state = TableState::default();
        if !self.nodes.is_empty() {
            let selected = self.selected_index.min(self.nodes.len() - 1);
            table_state.select(Some(selected));
        }

        f.render_stateful_widget(table, area, &mut table_state);
    }

    fn draw_nodes(&self, f: &mut Frame, area: Rect) {
        let rows: Vec<Row> = self
            .nodes
            .iter()
            .map(|node| {
                let is_running = node.status == "active";
                let status = if is_running { "Running" } else { "Stopped" };
                let status_color = if is_running { Color::Green } else { Color::Red };

                // Format publishers and subscribers compactly
                let pubs = if node.publishers.is_empty() {
                    "-".to_string()
                } else {
                    node.publishers.join(", ")
                };

                let subs = if node.subscribers.is_empty() {
                    "-".to_string()
                } else {
                    node.subscribers.join(", ")
                };

                Row::new(vec![
                    Cell::from(node.name.clone()),
                    Cell::from(node.process_id.to_string()),
                    Cell::from(format!("{:.1}%", node.cpu_usage)),
                    Cell::from(format!("{} MB", node.memory_usage / 1024 / 1024)),
                    Cell::from(status).style(Style::default().fg(status_color)),
                    Cell::from(pubs).style(Style::default().fg(Color::Green)),
                    Cell::from(subs).style(Style::default().fg(Color::Blue)),
                ])
            })
            .collect();

        let widths = [
            Constraint::Percentage(15),
            Constraint::Length(8),
            Constraint::Length(8),
            Constraint::Length(10),
            Constraint::Length(10),
            Constraint::Percentage(24),
            Constraint::Percentage(25),
        ];
        let table = Table::new(rows, widths)
            .header(
                Row::new(vec![
                    "Name",
                    "PID",
                    "CPU",
                    "Memory",
                    "Status",
                    "Publishes",
                    "Subscribes",
                ])
                .style(Style::default().add_modifier(Modifier::BOLD)),
            )
            .block(
                Block::default()
                    .title("Node Details - Use  to select, Enter to view logs")
                    .borders(Borders::ALL),
            )
            .row_highlight_style(
                Style::default()
                    .bg(Color::DarkGray)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol(" ");

        // Create table state with current selection
        let mut table_state = TableState::default();
        if !self.nodes.is_empty() {
            // Clamp selected_index to valid range
            let selected = self.selected_index.min(self.nodes.len() - 1);
            table_state.select(Some(selected));
        }

        f.render_stateful_widget(table, area, &mut table_state);
    }

    fn draw_help(&self, f: &mut Frame, area: Rect) {
        let help_text = vec![
            Line::from(""),
            Line::from(vec![Span::styled(
                "HORUS Terminal Monitor - Help",
                Style::default().add_modifier(Modifier::BOLD),
            )]),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Navigation:",
                Style::default().fg(Color::Cyan),
            )]),
            Line::from(
                "  Tab        - Next tab (Overview  Nodes  Topics  Network  Packages  Params)",
            ),
            Line::from("  Shift+Tab  - Previous tab"),
            Line::from("  ↑/↓        - Navigate lists"),
            Line::from("  PgUp/PgDn  - Scroll quickly"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "General Actions:",
                Style::default().fg(Color::Cyan),
            )]),
            Line::from("  p          - Pause/Resume updates"),
            Line::from("  q          - Quit monitor"),
            Line::from("  ?/h        - Show this help"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Nodes/Topics Tab:",
                Style::default().fg(Color::Cyan),
            )]),
            Line::from("  Enter      - Open log panel for selected node/topic"),
            Line::from("  ESC        - Close log panel"),
            Line::from("  Shift+↑↓   - Switch between nodes/topics while log panel is open"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Packages Tab:",
                Style::default().fg(Color::Cyan),
            )]),
            Line::from("  ← →        - Switch between Local Workspaces and Global Packages"),
            Line::from(
                "  Enter      - Drill into selected workspace to view packages & dependencies",
            ),
            Line::from("  ESC        - Navigate back to workspace list"),
            Line::from("  Note       - Missing dependencies (from horus.yaml) shown in red"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Parameters Tab:",
                Style::default().fg(Color::Cyan),
            )]),
            Line::from("  a          - Add new parameter"),
            Line::from("  e          - Edit selected parameter"),
            Line::from("  d          - Delete selected parameter"),
            Line::from("  r          - Refresh parameters from disk"),
            Line::from("  s          - Save parameters to disk"),
            Line::from(""),
            Line::from(vec![Span::styled(
                "Tab Descriptions:",
                Style::default().fg(Color::Cyan),
            )]),
            Line::from("  Overview   - Summary of nodes and topics (top 10)"),
            Line::from("  Nodes      - Full list of detected HORUS nodes with details"),
            Line::from("  Topics     - Full list of shared memory topics"),
            Line::from("  Network    - Network statistics and connections"),
            Line::from("  Packages   - Local workspaces and global packages (hierarchical)"),
            Line::from("  Params     - Runtime configuration parameters (editable)"),
            Line::from(""),
            Line::from(vec![
                Span::styled("Data Source: ", Style::default().fg(Color::Yellow)),
                Span::raw("Real-time from HORUS detect backend"),
            ]),
            Line::from("  • Nodes from /proc scan + registry"),
            Line::from(format!("  • Topics from {}", shm_topics_dir().display())),
            Line::from("  • Packages from ~/.horus/cache + local .horus/ directories"),
            Line::from("  • Params from ~/.horus/params.yaml (RuntimeParams)"),
            Line::from(""),
            Line::from("Press any key to close this help..."),
        ];

        let help = Paragraph::new(help_text)
            .block(
                Block::default()
                    .title("Help")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Cyan)),
            )
            .alignment(Alignment::Left);

        f.render_widget(help, area);
    }

    fn draw_log_panel(&self, f: &mut Frame, area: Rect) {
        // Get logs based on panel target
        let (_title, logs) = match &self.panel_target {
            Some(LogPanelTarget::Node(node_name)) => {
                let logs = GLOBAL_LOG_BUFFER.get_for_node(node_name);
                (format!("Logs: {}", node_name), logs)
            }
            Some(LogPanelTarget::Topic(topic_name)) => {
                let logs = GLOBAL_LOG_BUFFER.get_for_topic(topic_name);
                (format!("Logs: {}", topic_name), logs)
            }
            None => ("Logs".to_string(), Vec::new()),
        };

        // Format logs as lines
        let log_lines: Vec<Line> = if logs.is_empty() {
            vec![
                Line::from(""),
                Line::from(Span::styled(
                    "No logs available",
                    Style::default().fg(Color::DarkGray),
                )),
                Line::from(""),
                Line::from(Span::styled(
                    "Logs will appear here when the node/topic",
                    Style::default().fg(Color::DarkGray),
                )),
                Line::from(Span::styled(
                    "starts publishing or subscribing",
                    Style::default().fg(Color::DarkGray),
                )),
            ]
        } else {
            logs.iter()
                .skip(self.panel_scroll_offset)
                .map(|entry| {
                    // Color based on log type
                    let (type_str, type_color) = match entry.log_type {
                        LogType::Publish => ("PUB", Color::Green),
                        LogType::Subscribe => ("SUB", Color::Blue),
                        LogType::Info => ("INFO", Color::Cyan),
                        LogType::Warning => ("WARN", Color::Yellow),
                        LogType::Error => ("ERR", Color::Red),
                        LogType::Debug => ("DBG", Color::Magenta),
                        LogType::TopicRead => ("READ", Color::Blue),
                        LogType::TopicWrite => ("WRITE", Color::Green),
                        LogType::TopicMap => ("MAP", Color::Cyan),
                        LogType::TopicUnmap => ("UNMAP", Color::DarkGray),
                        LogType::RemoteDeploy => ("DEPLOY", Color::Magenta),
                        LogType::RemoteCompile => ("COMPILE", Color::Magenta),
                        LogType::RemoteExecute => ("EXEC", Color::Magenta),
                    };

                    // Format: [TIME] TYPE topic: message
                    let time_str = if let Some(time_part) = entry.timestamp.split('T').nth(1) {
                        time_part.split('.').next().unwrap_or(&entry.timestamp)
                    } else {
                        &entry.timestamp
                    };

                    let mut spans = vec![
                        Span::styled(
                            format!("[{}] ", time_str),
                            Style::default().fg(Color::DarkGray),
                        ),
                        Span::styled(
                            format!("{:<6} ", type_str),
                            Style::default().fg(type_color).add_modifier(Modifier::BOLD),
                        ),
                    ];

                    // Add topic if present
                    if let Some(topic) = &entry.topic {
                        spans.push(Span::styled(
                            format!("{}: ", topic),
                            Style::default().fg(Color::Cyan),
                        ));
                    }

                    // Add message
                    spans.push(Span::raw(&entry.message));

                    Line::from(spans)
                })
                .collect()
        };

        let help_text = format!("Showing {} logs |  Scroll | ESC Close", logs.len());

        // Create block with title
        let block = Block::default()
            .title(Line::from(vec![Span::styled(
                if let Some(target) = &self.panel_target {
                    match target {
                        LogPanelTarget::Node(name) => format!("Node: {}", name),
                        LogPanelTarget::Topic(name) => format!("Topic: {}", name),
                    }
                } else {
                    "Logs".to_string()
                },
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD),
            )]))
            .borders(Borders::ALL)
            .border_style(Style::default().fg(Color::Yellow));

        let panel = Paragraph::new(log_lines)
            .block(block)
            .alignment(Alignment::Left);

        f.render_widget(panel, area);

        // Draw help text at bottom
        let help_area = Rect {
            x: area.x + 1,
            y: area.y + area.height - 2,
            width: area.width.saturating_sub(2),
            height: 1,
        };

        let help_paragraph = Paragraph::new(Line::from(vec![Span::styled(
            help_text,
            Style::default().fg(Color::DarkGray),
        )]));

        f.render_widget(help_paragraph, help_area);
    }

    fn draw_footer(&self, f: &mut Frame, area: Rect) {
        let footer_text = if self.show_help {
            "Press any key to close help"
        } else if self.show_log_panel {
            "[ESC] Close | [] Scroll Logs | [Shift+] Switch Node/Topic | [Q] Quit"
        } else if self.active_tab == Tab::Parameters && self.param_edit_mode == ParamEditMode::None
        {
            "[A] Add | [E] Edit | [D] Delete | [R] Refresh | [S] Save | [TAB] Switch Tab | [?] Help | [Q] Quit"
        } else if self.active_tab == Tab::Parameters {
            "[TAB] Next Field | [ENTER] Confirm | [ESC] Cancel | [BACKSPACE] Delete Char"
        } else if self.active_tab == Tab::Packages
            && self.package_view_mode == PackageViewMode::List
        {
            "[ENTER] View Packages | [↑↓] Navigate | [TAB] Switch Tab | [?] Help | [Q] Quit"
        } else if self.active_tab == Tab::Packages
            && self.package_view_mode == PackageViewMode::WorkspaceDetails
        {
            "[ESC] Back to Workspaces | [↑↓] Navigate | [TAB] Switch Tab | [?] Help | [Q] Quit"
        } else if self.active_tab == Tab::Nodes || self.active_tab == Tab::Topics {
            "[ENTER] View Logs | [↑↓] Navigate | [TAB] Switch Tab | [P] Pause | [?] Help | [Q] Quit"
        } else {
            "[TAB] Switch Tab | [↑↓] Navigate | [P] Pause | [?] Help | [Q] Quit"
        };

        let footer = Paragraph::new(footer_text)
            .alignment(Alignment::Center)
            .style(Style::default().fg(Color::DarkGray));

        f.render_widget(footer, area);
    }

    fn update_data(&mut self) -> Result<()> {
        // Update nodes from detect backend
        if let Ok(nodes) = get_active_nodes() {
            self.nodes = nodes;
        }

        // Update topics from detect backend
        if let Ok(topics) = get_active_topics() {
            self.topics = topics;
        }

        Ok(())
    }

    fn next_tab(&mut self) {
        let tabs = Tab::all();
        let current = tabs.iter().position(|&t| t == self.active_tab).unwrap();
        self.active_tab = tabs[(current + 1) % tabs.len()];
        self.selected_index = 0;

        // Refresh recordings cache when switching to Recordings tab
        if self.active_tab == Tab::Recordings {
            self.refresh_recordings_cache();
        }
    }

    fn prev_tab(&mut self) {
        let tabs = Tab::all();
        let current = tabs.iter().position(|&t| t == self.active_tab).unwrap();
        self.active_tab = tabs[if current == 0 {
            tabs.len() - 1
        } else {
            current - 1
        }];
        self.selected_index = 0;

        // Refresh recordings cache when switching to Recordings tab
        if self.active_tab == Tab::Recordings {
            self.refresh_recordings_cache();
        }
    }

    fn select_next(&mut self) {
        // Get max index based on current tab
        let max_index = match self.active_tab {
            Tab::Overview => match self.overview_panel_focus {
                OverviewPanelFocus::Nodes => self.nodes.len().saturating_sub(1),
                OverviewPanelFocus::Topics => self.topics.len().saturating_sub(1),
            },
            Tab::Nodes => self.nodes.len().saturating_sub(1),
            Tab::Topics => self.topics.len().saturating_sub(1),
            Tab::Parameters => {
                let params_map = self.params.get_all();
                params_map.len().saturating_sub(1)
            }
            Tab::Packages => {
                if self.package_view_mode == PackageViewMode::List {
                    match self.package_panel_focus {
                        PackagePanelFocus::LocalWorkspaces => {
                            self.workspace_cache.len().saturating_sub(1)
                        }
                        PackagePanelFocus::GlobalPackages => {
                            let (_, global_packages) = get_installed_packages();
                            global_packages.len().saturating_sub(1)
                        }
                    }
                } else {
                    0
                }
            }
            _ => 0,
        };

        if self.selected_index < max_index {
            self.selected_index += 1;
        }
    }

    fn select_prev(&mut self) {
        self.selected_index = self.selected_index.saturating_sub(1);
    }

    fn scroll_up(&mut self, amount: usize) {
        self.scroll_offset = self.scroll_offset.saturating_sub(amount);
    }

    fn scroll_down(&mut self, amount: usize) {
        self.scroll_offset = self.scroll_offset.saturating_add(amount);
    }

    fn open_log_panel(&mut self) {
        match self.active_tab {
            Tab::Nodes => {
                // Open panel for selected node
                if self.selected_index < self.nodes.len() {
                    let node = &self.nodes[self.selected_index];
                    // Don't open panel for placeholder entries
                    if !node.name.contains("No HORUS nodes") {
                        self.panel_target = Some(LogPanelTarget::Node(node.name.clone()));
                        self.show_log_panel = true;
                        self.panel_scroll_offset = 0;
                    }
                }
            }
            Tab::Topics => {
                // Open panel for selected topic
                if self.selected_index < self.topics.len() {
                    let topic = &self.topics[self.selected_index];
                    // Don't open panel for placeholder entries
                    if !topic.name.contains("No active topics") {
                        self.panel_target = Some(LogPanelTarget::Topic(topic.name.clone()));
                        self.show_log_panel = true;
                        self.panel_scroll_offset = 0;
                    }
                }
            }
            _ => {
                // Log panel not supported for other tabs
            }
        }
    }

    fn update_log_panel_target(&mut self) {
        // Update the log panel to show logs for the currently selected node/topic
        // This is called when using Shift+Up/Down to navigate while panel is open
        match self.active_tab {
            Tab::Nodes => {
                if self.selected_index < self.nodes.len() {
                    let node = &self.nodes[self.selected_index];
                    // Don't update for placeholder entries
                    if !node.name.contains("No HORUS nodes") {
                        self.panel_target = Some(LogPanelTarget::Node(node.name.clone()));
                        self.panel_scroll_offset = 0; // Reset scroll when switching
                    }
                }
            }
            Tab::Topics => {
                if self.selected_index < self.topics.len() {
                    let topic = &self.topics[self.selected_index];
                    // Don't update for placeholder entries
                    if !topic.name.contains("No active topics") {
                        self.panel_target = Some(LogPanelTarget::Topic(topic.name.clone()));
                        self.panel_scroll_offset = 0; // Reset scroll when switching
                    }
                }
            }
            _ => {}
        }
    }

    /// Get the count of active nodes, excluding placeholder entries
    fn get_active_node_count(&self) -> usize {
        if self.nodes.len() == 1 && self.nodes[0].name.contains("No HORUS nodes") {
            0
        } else {
            self.nodes.len()
        }
    }

    /// Get the count of active topics, excluding placeholder entries
    fn get_active_topic_count(&self) -> usize {
        if self.topics.len() == 1 && self.topics[0].name.contains("No active topics") {
            0
        } else {
            self.topics.len()
        }
    }

    fn handle_package_enter(&mut self) {
        match self.package_view_mode {
            PackageViewMode::List => {
                // Drill down into selected workspace (use cached data)
                if self.selected_index < self.workspace_cache.len() {
                    self.selected_workspace =
                        Some(self.workspace_cache[self.selected_index].clone());
                    self.package_view_mode = PackageViewMode::WorkspaceDetails;
                    self.selected_index = 0;
                    self.scroll_offset = 0;
                }
            }
            PackageViewMode::WorkspaceDetails => {
                // Could expand nested packages here in the future
            }
        }
    }

    fn draw_param_edit_dialog(&self, f: &mut Frame) {
        // Create centered popup area
        let area = f.area();
        let popup_width = 60.min(area.width - 4);
        let popup_height = 10.min(area.height - 4);
        let popup_x = (area.width - popup_width) / 2;
        let popup_y = (area.height - popup_height) / 2;

        let popup_area = Rect {
            x: popup_x,
            y: popup_y,
            width: popup_width,
            height: popup_height,
        };

        // Clear the popup area
        let clear_block = Block::default().style(Style::default().bg(Color::Reset));
        f.render_widget(clear_block, popup_area);

        match &self.param_edit_mode {
            ParamEditMode::Add => {
                let title = "Add New Parameter [ESC to cancel]";
                let block = Block::default()
                    .title(title)
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Green));

                let inner = block.inner(popup_area);
                f.render_widget(block, popup_area);

                // Split into key and value sections
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3), // Key input
                        Constraint::Length(3), // Value input
                        Constraint::Min(1),    // Help text
                    ])
                    .split(inner);

                // Draw key input
                let key_focused = self.param_input_focus == ParamInputFocus::Key;
                let key_block = Block::default()
                    .title("Key")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(if key_focused {
                        Color::Yellow
                    } else {
                        Color::Gray
                    }));
                let key_text = Paragraph::new(self.param_input_key.as_str()).block(key_block);
                f.render_widget(key_text, chunks[0]);

                // Draw value input
                let value_focused = self.param_input_focus == ParamInputFocus::Value;
                let value_block = Block::default()
                    .title("Value (JSON or string)")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(if value_focused {
                        Color::Yellow
                    } else {
                        Color::Gray
                    }));
                let value_text = Paragraph::new(self.param_input_value.as_str()).block(value_block);
                f.render_widget(value_text, chunks[1]);

                // Help text
                let help = Paragraph::new("Press [Enter] to move to next field or confirm")
                    .style(Style::default().fg(Color::DarkGray))
                    .alignment(Alignment::Center);
                f.render_widget(help, chunks[2]);
            }
            ParamEditMode::Edit(original_key) => {
                let title = format!("Edit Parameter: {} [ESC to cancel]", original_key);
                let block = Block::default()
                    .title(title)
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Cyan));

                let inner = block.inner(popup_area);
                f.render_widget(block, popup_area);

                // Split into key and value sections
                let chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(3), // Key input
                        Constraint::Length(3), // Value input
                        Constraint::Min(1),    // Help text
                    ])
                    .split(inner);

                // Draw key input
                let key_focused = self.param_input_focus == ParamInputFocus::Key;
                let key_block = Block::default()
                    .title("Key")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(if key_focused {
                        Color::Yellow
                    } else {
                        Color::Gray
                    }));
                let key_text = Paragraph::new(self.param_input_key.as_str()).block(key_block);
                f.render_widget(key_text, chunks[0]);

                // Draw value input
                let value_focused = self.param_input_focus == ParamInputFocus::Value;
                let value_block = Block::default()
                    .title("Value (JSON or string)")
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(if value_focused {
                        Color::Yellow
                    } else {
                        Color::Gray
                    }));
                let value_text = Paragraph::new(self.param_input_value.as_str()).block(value_block);
                f.render_widget(value_text, chunks[1]);

                // Help text
                let help = Paragraph::new("Press [Enter] to move to next field or confirm")
                    .style(Style::default().fg(Color::DarkGray))
                    .alignment(Alignment::Center);
                f.render_widget(help, chunks[2]);
            }
            ParamEditMode::Delete(key) => {
                let title = "Delete Parameter [ESC to cancel]";
                let block = Block::default()
                    .title(title)
                    .borders(Borders::ALL)
                    .border_style(Style::default().fg(Color::Red));

                let inner = block.inner(popup_area);
                f.render_widget(block, popup_area);

                // Show confirmation message
                let message = vec![
                    Line::from(""),
                    Line::from(Span::styled(
                        format!("Delete parameter '{}'?", key),
                        Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
                    )),
                    Line::from(""),
                    Line::from("This action cannot be undone."),
                    Line::from(""),
                    Line::from(vec![
                        Span::styled("Press ", Style::default().fg(Color::DarkGray)),
                        Span::styled("[Y]", Style::default().fg(Color::Green)),
                        Span::styled(" to confirm or ", Style::default().fg(Color::DarkGray)),
                        Span::styled("[N]", Style::default().fg(Color::Red)),
                        Span::styled(" to cancel", Style::default().fg(Color::DarkGray)),
                    ]),
                ];

                let paragraph = Paragraph::new(message).alignment(Alignment::Center);
                f.render_widget(paragraph, inner);
            }
            _ => {}
        }
    }

    // Parameter editing operations
    fn start_edit_parameter(&mut self) {
        let params_map = self.params.get_all();
        let params: Vec<_> = params_map.iter().collect();

        if self.selected_index < params.len() {
            let (key, value) = params[self.selected_index];
            self.param_edit_mode = ParamEditMode::Edit(key.clone());
            self.param_input_key = key.clone();
            self.param_input_value = value.to_string();
            self.param_input_focus = ParamInputFocus::Key;
        }
    }

    fn start_delete_parameter(&mut self) {
        let params_map = self.params.get_all();
        let params: Vec<_> = params_map.iter().collect();

        if self.selected_index < params.len() {
            let (key, _) = params[self.selected_index];
            self.param_edit_mode = ParamEditMode::Delete(key.clone());
        }
    }

    fn confirm_add_parameter(&mut self) {
        if !self.param_input_key.is_empty() {
            // Try to parse as JSON, fallback to string
            let value = if let Ok(json_value) = serde_json::from_str(&self.param_input_value) {
                json_value
            } else {
                serde_json::Value::String(self.param_input_value.clone())
            };

            // Create a mutable copy of the params
            let new_params = horus_core::RuntimeParams::default();
            for (k, v) in self.params.get_all().iter() {
                let _ = new_params.set(k, v.clone());
            }

            // Add the new parameter
            let _ = new_params.set(&self.param_input_key, value);

            // Replace the Arc
            self.params = std::sync::Arc::new(new_params);

            // Exit edit mode
            self.param_edit_mode = ParamEditMode::None;
            self.param_input_key.clear();
            self.param_input_value.clear();
        }
    }

    fn confirm_edit_parameter(&mut self) {
        if let ParamEditMode::Edit(original_key) = &self.param_edit_mode.clone() {
            // Try to parse as JSON, fallback to string
            let value = if let Ok(json_value) = serde_json::from_str(&self.param_input_value) {
                json_value
            } else {
                serde_json::Value::String(self.param_input_value.clone())
            };

            // Create a mutable copy of the params
            let new_params = horus_core::RuntimeParams::default();
            for (k, v) in self.params.get_all().iter() {
                if k != original_key {
                    let _ = new_params.set(k, v.clone());
                }
            }

            // Add the (possibly renamed) parameter with new value
            let _ = new_params.set(&self.param_input_key, value);

            // Replace the Arc
            self.params = std::sync::Arc::new(new_params);

            // Exit edit mode
            self.param_edit_mode = ParamEditMode::None;
            self.param_input_key.clear();
            self.param_input_value.clear();
        }
    }

    fn confirm_delete_parameter(&mut self) {
        if let ParamEditMode::Delete(key_to_delete) = &self.param_edit_mode.clone() {
            // Create a mutable copy of the params
            let new_params = horus_core::RuntimeParams::default();
            for (k, v) in self.params.get_all().iter() {
                if k != key_to_delete {
                    let _ = new_params.set(k, v.clone());
                }
            }

            // Replace the Arc
            self.params = std::sync::Arc::new(new_params);

            // Exit edit mode
            self.param_edit_mode = ParamEditMode::None;
        }
    }

    // =========================================================================
    // Time-Travel Debugger Methods
    // =========================================================================

    /// Refresh recordings cache if stale
    fn refresh_recordings_cache(&mut self) {
        const CACHE_TTL: Duration = Duration::from_secs(5);

        if self.debugger_state.cache_time.elapsed() > CACHE_TTL {
            self.debugger_state.recordings_cache = self.scan_recordings();
            self.debugger_state.cache_time = Instant::now();
        }
    }

    /// Scan for recordings in the recordings directory
    fn scan_recordings(&self) -> Vec<RecordingInfo> {
        let recordings_dir = dirs::home_dir()
            .unwrap_or_else(|| std::path::PathBuf::from("."))
            .join(".horus/recordings");

        let mut recordings = Vec::new();

        if recordings_dir.exists() {
            if let Ok(entries) = std::fs::read_dir(&recordings_dir) {
                for entry in entries.flatten() {
                    if entry.file_type().map(|t| t.is_dir()).unwrap_or(false) {
                        let session_name = entry.file_name().to_string_lossy().to_string();
                        let session_path = entry.path();

                        // Detect recording type
                        let recording_type = if session_path.join("data.bin").exists() {
                            RecordingType::ZeroCopy
                        } else if session_path.join("coordinator.json").exists() {
                            RecordingType::Distributed
                        } else {
                            RecordingType::Standard
                        };

                        // Count files
                        let file_count = session_path.read_dir().map(|d| d.count()).unwrap_or(0);

                        // Get total size
                        let size_bytes: u64 = session_path
                            .read_dir()
                            .map(|d| {
                                d.filter_map(|e| e.ok())
                                    .filter_map(|e| e.metadata().ok())
                                    .map(|m| m.len())
                                    .sum()
                            })
                            .unwrap_or(0);

                        // Try to get tick count from metadata
                        let total_ticks = self.get_recording_tick_count(&session_path);

                        recordings.push(RecordingInfo {
                            session_name,
                            recording_type,
                            file_count,
                            size_bytes,
                            total_ticks,
                        });
                    }
                }
            }
        }

        recordings.sort_by(|a, b| a.session_name.cmp(&b.session_name));
        recordings
    }

    /// Get tick count from recording metadata
    fn get_recording_tick_count(&self, path: &std::path::Path) -> Option<u64> {
        let metadata_path = path.join("metadata.json");
        if metadata_path.exists() {
            if let Ok(content) = std::fs::read_to_string(&metadata_path) {
                if let Ok(json) = serde_json::from_str::<serde_json::Value>(&content) {
                    return json.get("total_ticks").and_then(|v| v.as_u64());
                }
            }
        }
        None
    }

    /// Load selected recording into debugger
    fn load_recording_into_debugger(&mut self) {
        self.refresh_recordings_cache();

        if self.selected_index < self.debugger_state.recordings_cache.len() {
            let recording = &self.debugger_state.recordings_cache[self.selected_index];
            self.debugger_state.selected_session = Some(recording.session_name.clone());
            self.debugger_state.total_ticks = recording.total_ticks.unwrap_or(100);
            self.debugger_state.current_tick = 0;
            self.debugger_state.active = true;
            self.debugger_state.playback = PlaybackState::Paused;
            self.debugger_state.breakpoints.clear();
            self.debugger_state.watches.clear();
        }
    }

    /// Step forward in debugger
    fn debugger_step_forward(&mut self) {
        self.debugger_state.playback = PlaybackState::SteppingForward;
        if self.debugger_state.current_tick < self.debugger_state.total_ticks.saturating_sub(1) {
            self.debugger_state.current_tick += 1;

            // Check for breakpoints
            if self
                .debugger_state
                .breakpoints
                .contains(&self.debugger_state.current_tick)
            {
                self.debugger_state.playback = PlaybackState::Paused;
            }
        }
    }

    /// Step backward in debugger
    fn debugger_step_backward(&mut self) {
        self.debugger_state.playback = PlaybackState::SteppingBackward;
        if self.debugger_state.current_tick > 0 {
            self.debugger_state.current_tick -= 1;
        }
    }

    /// Toggle breakpoint at current tick
    fn toggle_breakpoint(&mut self) {
        let tick = self.debugger_state.current_tick;
        if let Some(pos) = self
            .debugger_state
            .breakpoints
            .iter()
            .position(|&t| t == tick)
        {
            self.debugger_state.breakpoints.remove(pos);
        } else {
            self.debugger_state.breakpoints.push(tick);
            self.debugger_state.breakpoints.sort();
        }
    }
}

// Unified backend functions using monitor module

fn get_active_nodes() -> Result<Vec<NodeStatus>> {
    // Use unified backend from monitor module
    let discovered_nodes = crate::discovery::discover_nodes().unwrap_or_default();

    if discovered_nodes.is_empty() {
        // Show demo data if no real nodes detected
        Ok(vec![NodeStatus {
            name: "No HORUS nodes detected".to_string(),
            status: "inactive".to_string(),
            cpu_usage: 0.0,
            memory_usage: 0,
            process_id: 0,
            priority: 0,
            publishers: Vec::new(),
            subscribers: Vec::new(),
        }])
    } else {
        Ok(discovered_nodes
            .into_iter()
            .map(|n| NodeStatus {
                name: n.name.clone(),
                status: if n.status == "Running" {
                    "active".to_string()
                } else {
                    "inactive".to_string()
                },
                cpu_usage: n.cpu_usage,
                memory_usage: n.memory_usage,
                process_id: n.process_id,
                priority: n.priority,
                publishers: n.publishers.iter().map(|p| p.topic.clone()).collect(),
                subscribers: n.subscribers.iter().map(|s| s.topic.clone()).collect(),
            })
            .collect())
    }
}

fn get_local_workspaces(current_workspace_path: &Option<std::path::PathBuf>) -> Vec<WorkspaceData> {
    use std::collections::HashSet;
    use std::fs;

    let mut workspaces = Vec::new();

    // Use unified workspace discovery
    let discovered = crate::workspace::discover_all_workspaces(current_workspace_path);

    for ws in discovered {
        let env_path_buf = ws.path;
        let horus_dir = env_path_buf.join(".horus");

        // Read dependencies from horus.yaml
        let horus_yaml_path = env_path_buf.join("horus.yaml");
        let yaml_dependencies = if horus_yaml_path.exists() {
            fs::read_to_string(&horus_yaml_path)
                .ok()
                .and_then(|content| serde_yaml::from_str::<serde_yaml::Value>(&content).ok())
                .and_then(|yaml| {
                    yaml.get("dependencies")
                        .and_then(|deps| deps.as_sequence())
                        .map(|seq| {
                            seq.iter()
                                .filter_map(|v| v.as_str().map(|s| s.to_string()))
                                .collect::<Vec<String>>()
                        })
                })
                .unwrap_or_default()
        } else {
            Vec::new()
        };

        // Get packages inside this workspace
        let packages_dir = horus_dir.join("packages");
        let mut packages = Vec::new();
        let mut installed_packages_set: HashSet<String> = HashSet::new();

        if packages_dir.exists() {
            if let Ok(pkg_entries) = fs::read_dir(&packages_dir) {
                for pkg_entry in pkg_entries.flatten() {
                    // Check if it's a directory OR a symlink (include broken symlinks to show them)
                    let file_type = pkg_entry.file_type().ok();
                    let is_dir = file_type.as_ref().map(|t| t.is_dir()).unwrap_or(false);
                    let is_symlink = file_type.as_ref().map(|t| t.is_symlink()).unwrap_or(false);
                    let is_pkg_entry = is_dir || is_symlink;

                    if is_pkg_entry {
                        let pkg_name = pkg_entry.file_name().to_string_lossy().to_string();
                        let pkg_path = pkg_entry.path();

                        // Check if symlink target exists (for broken symlink detection)
                        let symlink_broken = is_symlink && !pkg_path.exists();

                        // Try to get version from metadata.json (follow symlinks if valid)
                        let metadata_path = pkg_path.join("metadata.json");
                        let version = if symlink_broken {
                            "broken link".to_string()
                        } else if metadata_path.exists() {
                            fs::read_to_string(&metadata_path)
                                .ok()
                                .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
                                .and_then(|j| {
                                    j.get("version")
                                        .and_then(|v| v.as_str())
                                        .map(|s| s.to_string())
                                })
                                .unwrap_or_else(|| "unknown".to_string())
                        } else {
                            "unknown".to_string()
                        };

                        // Scan for installed packages inside this package's .horus/packages/
                        let nested_packages_dir = pkg_entry.path().join(".horus/packages");
                        let mut installed_packages = Vec::new();

                        if nested_packages_dir.exists() {
                            if let Ok(nested_entries) = fs::read_dir(&nested_packages_dir) {
                                for nested_entry in nested_entries.flatten() {
                                    if nested_entry
                                        .file_type()
                                        .map(|t| t.is_dir())
                                        .unwrap_or(false)
                                    {
                                        let nested_name =
                                            nested_entry.file_name().to_string_lossy().to_string();

                                        // Try to get version
                                        let nested_metadata_path =
                                            nested_entry.path().join("metadata.json");
                                        let nested_version = if nested_metadata_path.exists() {
                                            fs::read_to_string(&nested_metadata_path)
                                                .ok()
                                                .and_then(|s| {
                                                    serde_json::from_str::<serde_json::Value>(&s)
                                                        .ok()
                                                })
                                                .and_then(|j| {
                                                    j.get("version")
                                                        .and_then(|v| v.as_str())
                                                        .map(|s| s.to_string())
                                                })
                                                .unwrap_or_else(|| "unknown".to_string())
                                        } else {
                                            "unknown".to_string()
                                        };

                                        installed_packages.push((nested_name, nested_version));
                                    }
                                }
                            }
                        }

                        installed_packages_set.insert(pkg_name.clone());
                        packages.push(PackageData {
                            name: pkg_name,
                            version,
                            installed_packages,
                        });
                    }
                }
            }
        }

        // Process dependencies from horus.yaml - only include those NOT already installed
        let dependencies: Vec<DependencyData> = yaml_dependencies
            .iter()
            .filter_map(|dep_str| {
                let dep_name = dep_str.split('@').next().unwrap_or(dep_str);

                // Skip if already in installed packages
                if installed_packages_set.contains(dep_name) {
                    return None;
                }

                // This dependency is declared but not installed
                Some(DependencyData {
                    name: dep_name.to_string(),
                    declared_version: dep_str.clone(),
                    _status: DependencyStatus::Missing,
                })
            })
            .collect();

        // Always add the workspace, even if it has no packages
        workspaces.push(WorkspaceData {
            name: ws.name,
            path: env_path_buf.to_string_lossy().to_string(),
            packages,
            dependencies,
            is_current: ws.is_current,
        });
    }

    // Sort by: current workspace first, then alphabetically
    workspaces.sort_by(|a, b| {
        match (a.is_current, b.is_current) {
            (true, false) => std::cmp::Ordering::Less, // Current workspace comes first
            (false, true) => std::cmp::Ordering::Greater, // Current workspace comes first
            _ => a.name.cmp(&b.name),                  // Otherwise sort alphabetically
        }
    });
    workspaces
}

type PackageInfo = (String, String, String);
type InstalledPackages = (Vec<PackageInfo>, Vec<PackageInfo>);

/// Recursively calculate total size of a directory
fn calculate_dir_size(path: &std::path::Path) -> u64 {
    let mut total = 0;

    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            if let Ok(metadata) = entry.metadata() {
                if metadata.is_file() {
                    total += metadata.len();
                } else if metadata.is_dir() {
                    total += calculate_dir_size(&entry.path());
                }
            }
        }
    }

    total
}

/// Read version from package metadata.json
fn get_package_version(pkg_path: &std::path::Path) -> String {
    let metadata_path = pkg_path.join("metadata.json");

    if let Ok(content) = std::fs::read_to_string(&metadata_path) {
        if let Ok(json) = serde_json::from_str::<serde_json::Value>(&content) {
            if let Some(version) = json.get("version").and_then(|v| v.as_str()) {
                return version.to_string();
            }
        }
    }

    "unknown".to_string()
}

fn get_installed_packages() -> InstalledPackages {
    let mut local_packages = Vec::new();
    let mut global_packages = Vec::new();
    let mut seen = std::collections::HashSet::new();

    // Check local .horus/packages first (project-specific installed packages)
    let local_packages_dir = std::env::current_dir()
        .ok()
        .map(|d| d.join(".horus/packages"));

    if let Some(ref packages_dir) = local_packages_dir {
        if packages_dir.exists() {
            if let Ok(entries) = std::fs::read_dir(packages_dir) {
                for entry in entries.flatten() {
                    if let Some(name) = entry.file_name().to_str() {
                        if seen.insert(name.to_string()) {
                            let pkg_path = entry.path();

                            // Calculate real directory size
                            let total_bytes = calculate_dir_size(&pkg_path);
                            let size = if total_bytes == 0 {
                                "Unknown".to_string()
                            } else {
                                let kb = total_bytes / 1024;
                                if kb < 1024 {
                                    format!("{} KB", kb)
                                } else {
                                    format!("{:.1} MB", kb as f64 / 1024.0)
                                }
                            };

                            // Read real version from metadata.json
                            let version = get_package_version(&pkg_path);

                            local_packages.push((name.to_string(), version, size));
                        }
                    }
                }
            }
        }
    }

    // Check global ~/.horus/cache (system-wide)
    let global_cache = dirs::home_dir().map(|h| h.join(".horus/cache"));

    if let Some(ref cache_dir) = global_cache {
        if cache_dir.exists() {
            if let Ok(entries) = std::fs::read_dir(cache_dir) {
                for entry in entries.flatten() {
                    if let Some(name) = entry.file_name().to_str() {
                        if seen.insert(name.to_string()) {
                            let pkg_path = entry.path();

                            // Calculate real directory size
                            let total_bytes = calculate_dir_size(&pkg_path);
                            let size = if total_bytes == 0 {
                                "Unknown".to_string()
                            } else {
                                let kb = total_bytes / 1024;
                                if kb < 1024 {
                                    format!("{} KB", kb)
                                } else {
                                    format!("{:.1} MB", kb as f64 / 1024.0)
                                }
                            };

                            // Read real version from metadata.json
                            let version = get_package_version(&pkg_path);

                            global_packages.push((name.to_string(), version, size));
                        }
                    }
                }
            }
        }
    }

    // Sort both lists
    local_packages.sort_by(|a, b| a.0.cmp(&b.0));
    global_packages.sort_by(|a, b| a.0.cmp(&b.0));

    // Add placeholder if both are empty
    if local_packages.is_empty() && global_packages.is_empty() {
        local_packages.push((
            "No packages found".to_string(),
            "-".to_string(),
            "-".to_string(),
        ));
    }

    (local_packages, global_packages)
}

// Removed: get_runtime_parameters() - now using real RuntimeParams from horus_core

/// Format bytes into human-readable string (B, KB, MB, GB)
fn format_bytes(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;

    if bytes >= GB {
        format!("{:.1}GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.1}MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.1}KB", bytes as f64 / KB as f64)
    } else {
        format!("{}B", bytes)
    }
}

fn get_active_topics() -> Result<Vec<TopicInfo>> {
    // Use unified backend from monitor module
    let discovered_topics = crate::discovery::discover_shared_memory().unwrap_or_default();

    // Filter to only show active topics (ROS-like behavior)
    // Stale topics (no live processes, old modification time) are hidden
    let active_topics: Vec<_> = discovered_topics
        .into_iter()
        .filter(|t| t.status != crate::discovery::TopicStatus::Stale)
        .collect();

    if active_topics.is_empty() {
        // Return empty - no placeholder needed (clean like ROS)
        Ok(Vec::new())
    } else {
        Ok(active_topics
            .into_iter()
            .map(|t| {
                // Shorten type names for readability
                let short_type = t
                    .message_type
                    .as_ref()
                    .map(|ty| ty.split("::").last().unwrap_or(ty).to_string())
                    .unwrap_or_else(|| "Unknown".to_string());

                TopicInfo {
                    name: t.topic_name,
                    msg_type: short_type,
                    publishers: t.publishers.len(),
                    subscribers: t.subscribers.len(),
                    rate: t.message_rate_hz,
                    publisher_nodes: t.publishers,
                    subscriber_nodes: t.subscribers,
                    status: t.status,
                }
            })
            .collect())
    }
}

// ============================================================================
// TUI Dashboard Unit Tests
// ============================================================================

#[cfg(test)]
mod tests {
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
                _status: DependencyStatus::Missing,
            }],
            is_current: true,
        };

        assert_eq!(workspace.name, "my_robot");
        assert!(workspace.is_current);
        assert_eq!(workspace.packages.len(), 1);
        assert_eq!(workspace.dependencies.len(), 1);
    }

    #[test]
    fn test_dependency_status() {
        assert_ne!(DependencyStatus::Missing, DependencyStatus::Installed);

        let dep = DependencyData {
            name: "test_dep".to_string(),
            declared_version: "1.0.0".to_string(),
            _status: DependencyStatus::Missing,
        };
        assert_eq!(dep._status, DependencyStatus::Missing);
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
}
