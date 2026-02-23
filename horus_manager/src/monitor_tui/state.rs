use super::data::get_installed_packages;
use super::*;

impl TuiDashboard {
    pub(super) fn next_tab(&mut self) {
        let tabs = Tab::all();
        let current = tabs.iter().position(|&t| t == self.active_tab).unwrap_or(0);
        self.active_tab = tabs[(current + 1) % tabs.len()];
        self.selected_index = 0;

        // Refresh recordings cache when switching to Recordings tab
        if self.active_tab == Tab::Recordings {
            self.refresh_recordings_cache();
        }
    }

    pub(super) fn prev_tab(&mut self) {
        let tabs = Tab::all();
        let current = tabs.iter().position(|&t| t == self.active_tab).unwrap_or(0);
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

    pub(super) fn select_next(&mut self) {
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

    pub(super) fn select_prev(&mut self) {
        self.selected_index = self.selected_index.saturating_sub(1);
    }

    pub(super) fn scroll_up(&mut self, amount: usize) {
        self.scroll_offset = self.scroll_offset.saturating_sub(amount);
    }

    pub(super) fn scroll_down(&mut self, amount: usize) {
        self.scroll_offset = self.scroll_offset.saturating_add(amount);
    }

    pub(super) fn open_log_panel(&mut self) {
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

    pub(super) fn update_log_panel_target(&mut self) {
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
    pub(super) fn get_active_node_count(&self) -> usize {
        if self.nodes.len() == 1 && self.nodes[0].name.contains("No HORUS nodes") {
            0
        } else {
            self.nodes.len()
        }
    }

    /// Get the count of active topics, excluding placeholder entries
    pub(super) fn get_active_topic_count(&self) -> usize {
        if self.topics.len() == 1 && self.topics[0].name.contains("No active topics") {
            0
        } else {
            self.topics.len()
        }
    }

    pub(super) fn handle_package_enter(&mut self) {
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
    // Parameter editing operations
    pub(super) fn start_edit_parameter(&mut self) {
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

    pub(super) fn start_delete_parameter(&mut self) {
        let params_map = self.params.get_all();
        let params: Vec<_> = params_map.iter().collect();

        if self.selected_index < params.len() {
            let (key, _) = params[self.selected_index];
            self.param_edit_mode = ParamEditMode::Delete(key.clone());
        }
    }

    pub(super) fn confirm_add_parameter(&mut self) {
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

    pub(super) fn confirm_edit_parameter(&mut self) {
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

    pub(super) fn confirm_delete_parameter(&mut self) {
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
    pub(super) fn refresh_recordings_cache(&mut self) {
        const CACHE_TTL: Duration = Duration::from_secs(5);

        if self.debugger_state.cache_time.elapsed() > CACHE_TTL {
            self.debugger_state.recordings_cache = self.scan_recordings();
            self.debugger_state.cache_time = Instant::now();
        }
    }

    /// Scan for recordings in the recordings directory
    fn scan_recordings(&self) -> Vec<RecordingInfo> {
        let recordings_dir = crate::paths::recordings_dir()
            .unwrap_or_else(|_| std::path::PathBuf::from(".horus/recordings"));

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
    pub(super) fn load_recording_into_debugger(&mut self) {
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
    pub(super) fn debugger_step_forward(&mut self) {
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
    pub(super) fn debugger_step_backward(&mut self) {
        self.debugger_state.playback = PlaybackState::SteppingBackward;
        if self.debugger_state.current_tick > 0 {
            self.debugger_state.current_tick -= 1;
        }
    }

    /// Toggle breakpoint at current tick
    pub(super) fn toggle_breakpoint(&mut self) {
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
