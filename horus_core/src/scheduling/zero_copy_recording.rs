//! Zero-Copy Recording System
//!
//! High-performance recording using memory-mapped files and arena allocation.
//! Achieves near-zero overhead by writing messages directly to memory without serialization.
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                    Zero-Copy Recording                          │
//! ├─────────────────────────────────────────────────────────────────┤
//! │  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐ │
//! │  │   Arena     │───▶│  MMap File  │───▶│   Index File        │ │
//! │  │  Allocator  │    │  (data.bin) │    │ (offsets + metadata)│ │
//! │  └─────────────┘    └─────────────┘    └─────────────────────┘ │
//! │         │                  │                      │             │
//! │         ▼                  ▼                      ▼             │
//! │  [Bump allocate]   [Direct memcpy]    [Binary index entries]   │
//! │  [No serialize]    [OS page cache]    [Random access replay]   │
//! └─────────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage
//!
//! ```rust,ignore
//! use horus_core::scheduling::zero_copy_recording::*;
//!
//! // Create recorder
//! let mut recorder = ZeroCopyRecorder::new("session_001", 1024 * 1024 * 100)?; // 100MB
//!
//! // Record messages (zero-copy)
//! recorder.record_raw(tick, topic, &message_bytes)?;
//!
//! // Finalize and get recording
//! let recording = recorder.finalize()?;
//!
//! // Replay
//! let replayer = ZeroCopyReplayer::open(&recording.path)?;
//! for snapshot in replayer.iter() {
//!     // Process snapshot
//! }
//! ```

use std::collections::HashMap;
use std::fs::{self, File, OpenOptions};
use std::io::{self, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use memmap2::{MmapMut, MmapOptions};
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Errors that can occur during zero-copy recording
#[derive(Error, Debug)]
pub enum ZeroCopyError {
    #[error("IO error: {0}")]
    Io(#[from] io::Error),

    #[error("Recording buffer full (capacity: {capacity}, needed: {needed})")]
    BufferFull { capacity: usize, needed: usize },

    #[error("Invalid recording format: {0}")]
    InvalidFormat(String),

    #[error("Recording not finalized")]
    NotFinalized,

    #[error("Recording already finalized")]
    AlreadyFinalized,

    #[error("Seek out of bounds: tick {tick} not in range [0, {max_tick})")]
    SeekOutOfBounds { tick: u64, max_tick: u64 },

    #[error("Topic not found: {0}")]
    TopicNotFound(String),

    #[error("Mmap error: {0}")]
    MmapError(String),
}

pub type Result<T> = std::result::Result<T, ZeroCopyError>;

/// Magic bytes for zero-copy recording format
const MAGIC: &[u8; 8] = b"HORUS_ZC";
const VERSION: u32 = 1;

/// Header for the data file
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct DataFileHeader {
    pub magic: [u8; 8],
    pub version: u32,
    pub flags: u32,
    pub created_timestamp_ns: u64,
    pub total_entries: u64,
    pub total_data_bytes: u64,
    pub reserved: [u8; 32],
}

/// Entry header in the data file (fixed size for alignment)
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct EntryHeader {
    /// Tick number
    pub tick: u64,
    /// Topic ID (interned)
    pub topic_id: u32,
    /// Entry type (0 = input, 1 = output, 2 = state)
    pub entry_type: u8,
    /// Flags (reserved)
    pub flags: u8,
    /// Padding for alignment
    pub _padding: [u8; 2],
    /// Timestamp in nanoseconds
    pub timestamp_ns: u64,
    /// Data length
    pub data_len: u32,
    /// CRC32 of data (optional, 0 if not computed)
    pub crc32: u32,
}

impl EntryHeader {
    pub const SIZE: usize = std::mem::size_of::<Self>();
}

/// Index entry for fast random access
#[repr(C, align(8))]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Serialize, Deserialize)]
pub struct IndexEntry {
    /// Tick number
    pub tick: u64,
    /// Offset in data file
    pub data_offset: u64,
    /// Total size of this tick's entries
    pub total_size: u32,
    /// Number of entries in this tick
    pub entry_count: u16,
    /// Flags
    pub flags: u16,
}

/// Topic string interning table
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicTable {
    /// Topic name to ID mapping
    pub topics: HashMap<String, u32>,
    /// ID to topic name mapping (for replay)
    pub reverse: Vec<String>,
}

impl Default for TopicTable {
    fn default() -> Self {
        Self::new()
    }
}

impl TopicTable {
    pub fn new() -> Self {
        Self {
            topics: HashMap::new(),
            reverse: Vec::new(),
        }
    }

    /// Intern a topic name, returning its ID
    pub fn intern(&mut self, topic: &str) -> u32 {
        if let Some(&id) = self.topics.get(topic) {
            return id;
        }
        let id = self.reverse.len() as u32;
        self.topics.insert(topic.to_string(), id);
        self.reverse.push(topic.to_string());
        id
    }

    /// Get topic name by ID
    pub fn get(&self, id: u32) -> Option<&str> {
        self.reverse.get(id as usize).map(|s| s.as_str())
    }
}

/// Recording metadata (stored as JSON alongside data)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecordingMetadata {
    pub session_id: String,
    pub node_name: Option<String>,
    pub created_at: chrono::DateTime<chrono::Utc>,
    pub finalized_at: Option<chrono::DateTime<chrono::Utc>>,
    pub total_ticks: u64,
    pub total_entries: u64,
    pub total_bytes: u64,
    pub topics: TopicTable,
    pub custom: HashMap<String, String>,
}

/// Zero-copy recorder using memory-mapped files
pub struct ZeroCopyRecorder {
    /// Session identifier
    session_id: String,
    /// Node name (optional)
    node_name: Option<String>,
    /// Base directory for recording files
    base_dir: PathBuf,
    /// Memory-mapped data file
    mmap: Option<MmapMut>,
    /// Current write position in mmap
    write_pos: usize,
    /// Capacity of the mmap
    capacity: usize,
    /// Index entries
    index: Vec<IndexEntry>,
    /// Topic interning table
    topics: TopicTable,
    /// Current tick being recorded
    current_tick: u64,
    /// Current tick's start offset
    current_tick_offset: u64,
    /// Current tick's entry count
    current_tick_entries: u16,
    /// Total entries recorded
    total_entries: u64,
    /// Creation timestamp
    created_at: chrono::DateTime<chrono::Utc>,
    /// Whether recording is finalized
    finalized: bool,
    /// Data file handle
    data_file: Option<File>,
}

impl ZeroCopyRecorder {
    /// Create a new zero-copy recorder
    ///
    /// # Arguments
    /// * `session_id` - Unique session identifier
    /// * `capacity` - Maximum recording size in bytes
    pub fn new(session_id: &str, capacity: usize) -> Result<Self> {
        Self::with_node_name(session_id, None, capacity)
    }

    /// Create a new zero-copy recorder with node name
    pub fn with_node_name(
        session_id: &str,
        node_name: Option<&str>,
        capacity: usize,
    ) -> Result<Self> {
        let base_dir = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(".horus")
            .join("recordings")
            .join("zero_copy")
            .join(session_id);

        fs::create_dir_all(&base_dir)?;

        let data_path = base_dir.join("data.bin");
        let data_file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&data_path)?;

        // Pre-allocate file
        data_file.set_len(capacity as u64)?;

        // SAFETY: File was just created and pre-allocated to `capacity` bytes.
        // We have exclusive write access; no other process maps this file yet.
        let mmap = unsafe { MmapOptions::new().len(capacity).map_mut(&data_file)? };

        let mut recorder = Self {
            session_id: session_id.to_string(),
            node_name: node_name.map(|s| s.to_string()),
            base_dir,
            mmap: Some(mmap),
            write_pos: std::mem::size_of::<DataFileHeader>(),
            capacity,
            index: Vec::with_capacity(10000),
            topics: TopicTable::new(),
            current_tick: 0,
            current_tick_offset: std::mem::size_of::<DataFileHeader>() as u64,
            current_tick_entries: 0,
            total_entries: 0,
            created_at: chrono::Utc::now(),
            finalized: false,
            data_file: Some(data_file),
        };

        // Write header
        recorder.write_header()?;

        Ok(recorder)
    }

    /// Write the file header
    fn write_header(&mut self) -> Result<()> {
        let header = DataFileHeader {
            magic: *MAGIC,
            version: VERSION,
            flags: 0,
            created_timestamp_ns: self.created_at.timestamp_nanos_opt().unwrap_or(0) as u64,
            total_entries: 0,
            total_data_bytes: 0,
            reserved: [0; 32],
        };

        let mmap = self.mmap.as_mut().ok_or(ZeroCopyError::AlreadyFinalized)?;
        let header_bytes = bytemuck::bytes_of(&header);
        mmap[..header_bytes.len()].copy_from_slice(header_bytes);

        Ok(())
    }

    /// Start a new tick
    pub fn begin_tick(&mut self, tick: u64) -> Result<()> {
        if self.finalized {
            return Err(ZeroCopyError::AlreadyFinalized);
        }

        // Finalize previous tick if needed
        if self.current_tick_entries > 0 {
            self.finalize_tick()?;
        }

        self.current_tick = tick;
        self.current_tick_offset = self.write_pos as u64;
        self.current_tick_entries = 0;

        Ok(())
    }

    /// Finalize the current tick and add to index
    fn finalize_tick(&mut self) -> Result<()> {
        if self.current_tick_entries > 0 {
            let entry = IndexEntry {
                tick: self.current_tick,
                data_offset: self.current_tick_offset,
                total_size: (self.write_pos as u64 - self.current_tick_offset) as u32,
                entry_count: self.current_tick_entries,
                flags: 0,
            };
            self.index.push(entry);
        }
        Ok(())
    }

    /// Record raw bytes for a topic (zero-copy)
    ///
    /// # Arguments
    /// * `topic` - Topic name
    /// * `entry_type` - 0 = input, 1 = output, 2 = state
    /// * `data` - Raw message bytes
    pub fn record_raw(&mut self, topic: &str, entry_type: u8, data: &[u8]) -> Result<()> {
        if self.finalized {
            return Err(ZeroCopyError::AlreadyFinalized);
        }

        let topic_id = self.topics.intern(topic);
        let entry_size = EntryHeader::SIZE + data.len();

        // Check capacity
        if self.write_pos + entry_size > self.capacity {
            return Err(ZeroCopyError::BufferFull {
                capacity: self.capacity,
                needed: self.write_pos + entry_size,
            });
        }

        let mmap = self.mmap.as_mut().ok_or(ZeroCopyError::AlreadyFinalized)?;

        // Write entry header
        let header = EntryHeader {
            tick: self.current_tick,
            topic_id,
            entry_type,
            flags: 0,
            _padding: [0; 2],
            timestamp_ns: chrono::Utc::now().timestamp_nanos_opt().unwrap_or(0) as u64,
            data_len: data.len() as u32,
            crc32: 0, // Could compute CRC32 if needed
        };

        let header_bytes = bytemuck::bytes_of(&header);
        mmap[self.write_pos..self.write_pos + EntryHeader::SIZE].copy_from_slice(header_bytes);
        self.write_pos += EntryHeader::SIZE;

        // Write data (direct memcpy - zero copy!)
        mmap[self.write_pos..self.write_pos + data.len()].copy_from_slice(data);
        self.write_pos += data.len();

        self.current_tick_entries += 1;
        self.total_entries += 1;

        Ok(())
    }

    /// Record a serializable message
    pub fn record<T: Serialize>(&mut self, topic: &str, entry_type: u8, message: &T) -> Result<()> {
        let data = bincode::serialize(message).map_err(|e| {
            ZeroCopyError::InvalidFormat(format!("Failed to serialize message: {}", e))
        })?;
        self.record_raw(topic, entry_type, &data)
    }

    /// Get current recording stats
    pub fn stats(&self) -> RecordingStats {
        RecordingStats {
            total_entries: self.total_entries,
            total_bytes: self.write_pos as u64,
            total_ticks: self.index.len() as u64
                + if self.current_tick_entries > 0 { 1 } else { 0 },
            topics_count: self.topics.reverse.len(),
            capacity: self.capacity,
            utilization: self.write_pos as f64 / self.capacity as f64,
        }
    }

    /// Finalize the recording
    pub fn finalize(mut self) -> Result<ZeroCopyRecording> {
        if self.finalized {
            return Err(ZeroCopyError::AlreadyFinalized);
        }

        // Finalize last tick
        self.finalize_tick()?;

        // Update header with final stats
        {
            let mmap = self.mmap.as_mut().ok_or(ZeroCopyError::AlreadyFinalized)?;
            let header = DataFileHeader {
                magic: *MAGIC,
                version: VERSION,
                flags: 1, // Finalized flag
                created_timestamp_ns: self.created_at.timestamp_nanos_opt().unwrap_or(0) as u64,
                total_entries: self.total_entries,
                total_data_bytes: self.write_pos as u64,
                reserved: [0; 32],
            };
            let header_bytes = bytemuck::bytes_of(&header);
            mmap[..header_bytes.len()].copy_from_slice(header_bytes);

            // Flush to disk
            mmap.flush()?;
        }

        // Truncate file to actual size
        if let Some(ref file) = self.data_file {
            file.set_len(self.write_pos as u64)?;
        }

        // Write index file
        let index_path = self.base_dir.join("index.bin");
        let mut index_file = BufWriter::new(File::create(&index_path)?);
        for entry in &self.index {
            index_file.write_all(bytemuck::bytes_of(entry))?;
        }
        index_file.flush()?;

        // Write metadata
        let metadata = RecordingMetadata {
            session_id: self.session_id.clone(),
            node_name: self.node_name.clone(),
            created_at: self.created_at,
            finalized_at: Some(chrono::Utc::now()),
            total_ticks: self.index.len() as u64,
            total_entries: self.total_entries,
            total_bytes: self.write_pos as u64,
            topics: self.topics.clone(),
            custom: HashMap::new(),
        };

        let metadata_path = self.base_dir.join("metadata.json");
        let metadata_file = File::create(&metadata_path)?;
        serde_json::to_writer_pretty(metadata_file, &metadata).map_err(|e| {
            ZeroCopyError::InvalidFormat(format!("Failed to write metadata: {}", e))
        })?;

        self.finalized = true;

        Ok(ZeroCopyRecording {
            path: self.base_dir.clone(),
            metadata,
        })
    }
}

/// Statistics about a recording
#[derive(Debug, Clone)]
pub struct RecordingStats {
    pub total_entries: u64,
    pub total_bytes: u64,
    pub total_ticks: u64,
    pub topics_count: usize,
    pub capacity: usize,
    pub utilization: f64,
}

/// A finalized zero-copy recording
#[derive(Debug, Clone)]
pub struct ZeroCopyRecording {
    pub path: PathBuf,
    pub metadata: RecordingMetadata,
}

impl ZeroCopyRecording {
    /// Open an existing recording
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_path_buf();
        let metadata_path = path.join("metadata.json");

        let metadata_file = File::open(&metadata_path)?;
        let metadata: RecordingMetadata = serde_json::from_reader(metadata_file)
            .map_err(|e| ZeroCopyError::InvalidFormat(format!("Failed to read metadata: {}", e)))?;

        Ok(Self { path, metadata })
    }

    /// Create a replayer for this recording
    pub fn replayer(&self) -> Result<ZeroCopyReplayer> {
        ZeroCopyReplayer::open(&self.path)
    }
}

/// Zero-copy replayer using memory-mapped files
pub struct ZeroCopyReplayer {
    /// Memory-mapped data file (read-only)
    mmap: memmap2::Mmap,
    /// Index entries
    index: Vec<IndexEntry>,
    /// Topic table
    topics: TopicTable,
    /// Metadata
    metadata: RecordingMetadata,
    /// Current position in index
    current_index: usize,
    /// Base path
    _path: PathBuf,
}

impl ZeroCopyReplayer {
    /// Open a recording for replay
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref().to_path_buf();

        // Read metadata
        let metadata_path = path.join("metadata.json");
        let metadata_file = File::open(&metadata_path)?;
        let metadata: RecordingMetadata = serde_json::from_reader(metadata_file)
            .map_err(|e| ZeroCopyError::InvalidFormat(format!("Failed to read metadata: {}", e)))?;

        // Memory map data file
        let data_path = path.join("data.bin");
        let data_file = File::open(&data_path)?;
        // SAFETY: File is opened read-only for replay. Header is validated immediately after mapping.
        let mmap = unsafe { MmapOptions::new().map(&data_file)? };

        // Verify header
        if mmap.len() < std::mem::size_of::<DataFileHeader>() {
            return Err(ZeroCopyError::InvalidFormat("File too small".to_string()));
        }

        // Copy to aligned buffer to satisfy bytemuck alignment requirements
        let mut header: DataFileHeader = bytemuck::Zeroable::zeroed();
        let header_bytes: &mut [u8] = bytemuck::bytes_of_mut(&mut header);
        header_bytes.copy_from_slice(&mmap[..std::mem::size_of::<DataFileHeader>()]);

        if &header.magic != MAGIC {
            return Err(ZeroCopyError::InvalidFormat(
                "Invalid magic bytes".to_string(),
            ));
        }
        if header.version != VERSION {
            return Err(ZeroCopyError::InvalidFormat(format!(
                "Unsupported version: {}",
                header.version
            )));
        }

        // Read index
        let index_path = path.join("index.bin");
        let mut index_file = File::open(&index_path)?;
        let mut index_data = Vec::new();
        index_file.read_to_end(&mut index_data)?;

        // Parse index entries safely (copy to aligned buffer)
        let entry_size = std::mem::size_of::<IndexEntry>();
        let index: Vec<IndexEntry> = index_data
            .chunks_exact(entry_size)
            .map(|chunk| {
                // Copy to aligned buffer to satisfy bytemuck alignment requirements
                let mut aligned: IndexEntry = bytemuck::Zeroable::zeroed();
                let aligned_bytes: &mut [u8] = bytemuck::bytes_of_mut(&mut aligned);
                aligned_bytes.copy_from_slice(chunk);
                aligned
            })
            .collect();

        Ok(Self {
            mmap,
            index,
            topics: metadata.topics.clone(),
            metadata,
            current_index: 0,
            _path: path,
        })
    }

    /// Get total number of ticks
    pub fn total_ticks(&self) -> u64 {
        self.index.len() as u64
    }

    /// Get current tick position
    pub fn current_tick(&self) -> Option<u64> {
        self.index.get(self.current_index).map(|e| e.tick)
    }

    /// Seek to a specific tick
    pub fn seek_to_tick(&mut self, tick: u64) -> Result<()> {
        // Binary search for the tick
        match self.index.binary_search_by_key(&tick, |e| e.tick) {
            Ok(idx) => {
                self.current_index = idx;
                Ok(())
            }
            Err(_) => Err(ZeroCopyError::SeekOutOfBounds {
                tick,
                max_tick: self.index.last().map(|e| e.tick).unwrap_or(0),
            }),
        }
    }

    /// Seek to index position
    pub fn seek_to_index(&mut self, index: usize) -> Result<()> {
        if index >= self.index.len() {
            return Err(ZeroCopyError::SeekOutOfBounds {
                tick: index as u64,
                max_tick: self.index.len() as u64,
            });
        }
        self.current_index = index;
        Ok(())
    }

    /// Read the current tick's entries
    pub fn read_current(&self) -> Option<TickData> {
        let idx_entry = self.index.get(self.current_index)?;
        self.read_tick_at_offset(idx_entry)
    }

    /// Read tick data at a specific index entry
    fn read_tick_at_offset(&self, idx_entry: &IndexEntry) -> Option<TickData> {
        let mut entries = Vec::with_capacity(idx_entry.entry_count as usize);
        let mut offset = idx_entry.data_offset as usize;

        for _ in 0..idx_entry.entry_count {
            if offset + EntryHeader::SIZE > self.mmap.len() {
                break;
            }

            // Copy to aligned buffer to satisfy bytemuck alignment requirements
            let mut header: EntryHeader = bytemuck::Zeroable::zeroed();
            let header_bytes: &mut [u8] = bytemuck::bytes_of_mut(&mut header);
            header_bytes.copy_from_slice(&self.mmap[offset..offset + EntryHeader::SIZE]);
            offset += EntryHeader::SIZE;

            let data_end = offset + header.data_len as usize;
            if data_end > self.mmap.len() {
                break;
            }

            let data = &self.mmap[offset..data_end];
            offset = data_end;

            entries.push(EntryData {
                topic_id: header.topic_id,
                topic_name: self.topics.get(header.topic_id).map(|s| s.to_string()),
                entry_type: header.entry_type,
                timestamp_ns: header.timestamp_ns,
                data: data.to_vec(), // Could return slice for true zero-copy
            });
        }

        Some(TickData {
            tick: idx_entry.tick,
            entries,
        })
    }

    /// Advance to next tick
    pub fn next_tick(&mut self) -> Option<TickData> {
        if self.current_index >= self.index.len() {
            return None;
        }
        let data = self.read_current();
        self.current_index += 1;
        data
    }

    /// Go to previous tick
    pub fn prev(&mut self) -> Option<TickData> {
        if self.current_index == 0 {
            return None;
        }
        self.current_index -= 1;
        self.read_current()
    }

    /// Reset to beginning
    pub fn reset(&mut self) {
        self.current_index = 0;
    }

    /// Create an iterator over all ticks
    pub fn iter(&self) -> TickIterator<'_> {
        TickIterator {
            replayer: self,
            index: 0,
        }
    }

    /// Get metadata
    pub fn metadata(&self) -> &RecordingMetadata {
        &self.metadata
    }

    /// Get raw slice of data for a tick (true zero-copy access)
    pub fn raw_tick_data(&self, tick_index: usize) -> Option<&[u8]> {
        let idx_entry = self.index.get(tick_index)?;
        let start = idx_entry.data_offset as usize;
        let end = start + idx_entry.total_size as usize;
        if end <= self.mmap.len() {
            Some(&self.mmap[start..end])
        } else {
            None
        }
    }
}

/// Data for a single tick
#[derive(Debug, Clone)]
pub struct TickData {
    pub tick: u64,
    pub entries: Vec<EntryData>,
}

/// Data for a single entry
#[derive(Debug, Clone)]
pub struct EntryData {
    pub topic_id: u32,
    pub topic_name: Option<String>,
    pub entry_type: u8,
    pub timestamp_ns: u64,
    pub data: Vec<u8>,
}

impl EntryData {
    /// Deserialize the data as a specific type
    pub fn deserialize<T: for<'de> Deserialize<'de>>(
        &self,
    ) -> std::result::Result<T, bincode::Error> {
        bincode::deserialize(&self.data)
    }

    /// Get raw data slice
    pub fn raw(&self) -> &[u8] {
        &self.data
    }

    /// Entry type as string
    pub fn entry_type_str(&self) -> &'static str {
        match self.entry_type {
            0 => "input",
            1 => "output",
            2 => "state",
            _ => "unknown",
        }
    }
}

/// Iterator over ticks
pub struct TickIterator<'a> {
    replayer: &'a ZeroCopyReplayer,
    index: usize,
}

impl Iterator for TickIterator<'_> {
    type Item = TickData;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.replayer.index.len() {
            return None;
        }
        let idx_entry = &self.replayer.index[self.index];
        self.index += 1;
        self.replayer.read_tick_at_offset(idx_entry)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.replayer.index.len() - self.index;
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for TickIterator<'_> {}

// ============================================================================
// Streaming Recorder (for continuous recording without pre-allocation)
// ============================================================================

/// Streaming zero-copy recorder that writes directly to file
pub struct StreamingRecorder {
    /// Session identifier
    session_id: String,
    /// Node name
    node_name: Option<String>,
    /// Base directory
    base_dir: PathBuf,
    /// Data file writer
    writer: BufWriter<File>,
    /// Current write position
    write_pos: u64,
    /// Index entries
    index: Vec<IndexEntry>,
    /// Topic table
    topics: TopicTable,
    /// Current tick
    current_tick: u64,
    /// Current tick offset
    current_tick_offset: u64,
    /// Current tick entries
    current_tick_entries: u16,
    /// Total entries
    total_entries: u64,
    /// Created at
    created_at: chrono::DateTime<chrono::Utc>,
    /// Finalized flag
    finalized: bool,
}

impl StreamingRecorder {
    /// Create a new streaming recorder
    pub fn new(session_id: &str) -> Result<Self> {
        Self::with_node_name(session_id, None)
    }

    /// Create with node name
    pub fn with_node_name(session_id: &str, node_name: Option<&str>) -> Result<Self> {
        let base_dir = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(".horus")
            .join("recordings")
            .join("zero_copy")
            .join(session_id);

        fs::create_dir_all(&base_dir)?;

        let data_path = base_dir.join("data.bin");
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(true)
            .open(&data_path)?;

        let mut writer = BufWriter::with_capacity(64 * 1024, file); // 64KB buffer

        // Write header placeholder
        let header = DataFileHeader {
            magic: *MAGIC,
            version: VERSION,
            flags: 0,
            created_timestamp_ns: 0,
            total_entries: 0,
            total_data_bytes: 0,
            reserved: [0; 32],
        };
        writer.write_all(bytemuck::bytes_of(&header))?;

        let write_pos = std::mem::size_of::<DataFileHeader>() as u64;

        Ok(Self {
            session_id: session_id.to_string(),
            node_name: node_name.map(|s| s.to_string()),
            base_dir,
            writer,
            write_pos,
            index: Vec::with_capacity(10000),
            topics: TopicTable::new(),
            current_tick: 0,
            current_tick_offset: write_pos,
            current_tick_entries: 0,
            total_entries: 0,
            created_at: chrono::Utc::now(),
            finalized: false,
        })
    }

    /// Begin a new tick
    pub fn begin_tick(&mut self, tick: u64) -> Result<()> {
        if self.finalized {
            return Err(ZeroCopyError::AlreadyFinalized);
        }

        if self.current_tick_entries > 0 {
            self.finalize_tick()?;
        }

        self.current_tick = tick;
        self.current_tick_offset = self.write_pos;
        self.current_tick_entries = 0;

        Ok(())
    }

    fn finalize_tick(&mut self) -> Result<()> {
        if self.current_tick_entries > 0 {
            let entry = IndexEntry {
                tick: self.current_tick,
                data_offset: self.current_tick_offset,
                total_size: (self.write_pos - self.current_tick_offset) as u32,
                entry_count: self.current_tick_entries,
                flags: 0,
            };
            self.index.push(entry);
        }
        Ok(())
    }

    /// Record raw bytes
    pub fn record_raw(&mut self, topic: &str, entry_type: u8, data: &[u8]) -> Result<()> {
        if self.finalized {
            return Err(ZeroCopyError::AlreadyFinalized);
        }

        let topic_id = self.topics.intern(topic);

        let header = EntryHeader {
            tick: self.current_tick,
            topic_id,
            entry_type,
            flags: 0,
            _padding: [0; 2],
            timestamp_ns: chrono::Utc::now().timestamp_nanos_opt().unwrap_or(0) as u64,
            data_len: data.len() as u32,
            crc32: 0,
        };

        self.writer.write_all(bytemuck::bytes_of(&header))?;
        self.writer.write_all(data)?;

        self.write_pos += EntryHeader::SIZE as u64 + data.len() as u64;
        self.current_tick_entries += 1;
        self.total_entries += 1;

        Ok(())
    }

    /// Record serializable message
    pub fn record<T: Serialize>(&mut self, topic: &str, entry_type: u8, message: &T) -> Result<()> {
        let data = bincode::serialize(message)
            .map_err(|e| ZeroCopyError::InvalidFormat(format!("Failed to serialize: {}", e)))?;
        self.record_raw(topic, entry_type, &data)
    }

    /// Flush buffer to disk
    pub fn flush(&mut self) -> Result<()> {
        self.writer.flush()?;
        Ok(())
    }

    /// Finalize the recording
    pub fn finalize(mut self) -> Result<ZeroCopyRecording> {
        if self.finalized {
            return Err(ZeroCopyError::AlreadyFinalized);
        }

        self.finalize_tick()?;
        self.writer.flush()?;

        // Rewrite header with final stats
        let inner = self.writer.into_inner().map_err(|e| e.into_error())?;
        let mut file = inner;
        file.seek(SeekFrom::Start(0))?;

        let header = DataFileHeader {
            magic: *MAGIC,
            version: VERSION,
            flags: 1,
            created_timestamp_ns: self.created_at.timestamp_nanos_opt().unwrap_or(0) as u64,
            total_entries: self.total_entries,
            total_data_bytes: self.write_pos,
            reserved: [0; 32],
        };
        file.write_all(bytemuck::bytes_of(&header))?;
        file.sync_all()?;

        // Write index
        let index_path = self.base_dir.join("index.bin");
        let mut index_file = BufWriter::new(File::create(&index_path)?);
        for entry in &self.index {
            index_file.write_all(bytemuck::bytes_of(entry))?;
        }
        index_file.flush()?;

        // Write metadata
        let metadata = RecordingMetadata {
            session_id: self.session_id.clone(),
            node_name: self.node_name.clone(),
            created_at: self.created_at,
            finalized_at: Some(chrono::Utc::now()),
            total_ticks: self.index.len() as u64,
            total_entries: self.total_entries,
            total_bytes: self.write_pos,
            topics: self.topics.clone(),
            custom: HashMap::new(),
        };

        let metadata_path = self.base_dir.join("metadata.json");
        let metadata_file = File::create(&metadata_path)?;
        serde_json::to_writer_pretty(metadata_file, &metadata).map_err(|e| {
            ZeroCopyError::InvalidFormat(format!("Failed to write metadata: {}", e))
        })?;

        self.finalized = true;

        Ok(ZeroCopyRecording {
            path: self.base_dir,
            metadata,
        })
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topic_table() {
        let mut table = TopicTable::new();

        let id1 = table.intern("sensor/imu");
        let id2 = table.intern("sensor/camera");
        let id3 = table.intern("sensor/imu"); // Duplicate

        assert_eq!(id1, 0);
        assert_eq!(id2, 1);
        assert_eq!(id3, 0); // Same as first

        assert_eq!(table.get(0), Some("sensor/imu"));
        assert_eq!(table.get(1), Some("sensor/camera"));
        assert_eq!(table.get(2), None);
    }

    #[test]
    fn test_entry_header_size() {
        // Ensure header is properly aligned
        assert_eq!(EntryHeader::SIZE, 32);
    }

    #[test]
    fn test_zero_copy_recorder() -> Result<()> {
        let session_id = format!("test_zc_{}", uuid::Uuid::new_v4());
        let mut recorder = ZeroCopyRecorder::new(&session_id, 1024 * 1024)?;

        recorder.begin_tick(0)?;
        recorder.record_raw("sensor/imu", 0, &[1, 2, 3, 4])?;
        recorder.record_raw("sensor/camera", 0, &[5, 6, 7, 8, 9, 10])?;

        recorder.begin_tick(1)?;
        recorder.record_raw("sensor/imu", 0, &[11, 12, 13, 14])?;

        let recording = recorder.finalize()?;

        assert_eq!(recording.metadata.total_ticks, 2);
        assert_eq!(recording.metadata.total_entries, 3);

        // Test replay
        let mut replayer = recording.replayer()?;

        let tick0 = replayer.next_tick().unwrap();
        assert_eq!(tick0.tick, 0);
        assert_eq!(tick0.entries.len(), 2);
        assert_eq!(tick0.entries[0].data, vec![1, 2, 3, 4]);

        let tick1 = replayer.next_tick().unwrap();
        assert_eq!(tick1.tick, 1);
        assert_eq!(tick1.entries.len(), 1);

        assert!(replayer.next_tick().is_none());

        // Cleanup
        fs::remove_dir_all(&recording.path).ok();

        Ok(())
    }

    #[test]
    fn test_streaming_recorder() -> Result<()> {
        let session_id = format!("test_stream_{}", uuid::Uuid::new_v4());
        let mut recorder = StreamingRecorder::new(&session_id)?;

        recorder.begin_tick(0)?;
        recorder.record_raw("topic_a", 1, &[100, 101, 102])?;

        recorder.begin_tick(1)?;
        recorder.record_raw("topic_b", 1, &[200, 201])?;

        let recording = recorder.finalize()?;

        let replayer = recording.replayer()?;
        assert_eq!(replayer.total_ticks(), 2);

        // Cleanup
        fs::remove_dir_all(&recording.path).ok();

        Ok(())
    }

    #[test]
    fn test_seek_operations() -> Result<()> {
        let session_id = format!("test_seek_{}", uuid::Uuid::new_v4());
        let mut recorder = ZeroCopyRecorder::new(&session_id, 1024 * 1024)?;

        for tick in 0..10 {
            recorder.begin_tick(tick)?;
            recorder.record_raw("data", 0, &[tick as u8])?;
        }

        let recording = recorder.finalize()?;
        let mut replayer = recording.replayer()?;

        // Seek to tick 5
        replayer.seek_to_tick(5)?;
        let data = replayer.read_current().unwrap();
        assert_eq!(data.tick, 5);
        assert_eq!(data.entries[0].data, vec![5]);

        // Seek backwards
        replayer.seek_to_tick(2)?;
        let data = replayer.read_current().unwrap();
        assert_eq!(data.tick, 2);

        // Cleanup
        fs::remove_dir_all(&recording.path).ok();

        Ok(())
    }
}
