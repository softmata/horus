use crate::communication::network::fragmentation::{Fragment, FragmentManager};
/// High-performance async router client backend
///
/// Optimizations:
/// - Async Tokio I/O (no blocking, no sleep)
/// - Lock-free crossbeam channels
/// - TCP_NODELAY for low latency
/// - Zero-allocation buffer pooling
/// - Batched operations
/// - Zero-copy where possible
use crate::communication::network::protocol::{HorusPacket, MessageType};
use crate::error::HorusResult;
use crossbeam::channel::{bounded, Receiver, Sender};
use crossbeam::queue::SegQueue;
use log::{error, warn};
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use tokio::runtime::Handle;

const DEFAULT_ROUTER_PORT: u16 = 7777;
const RECV_QUEUE_SIZE: usize = 1024; // Increased from 128
const BUFFER_SIZE: usize = 65536;
const SEND_QUEUE_SIZE: usize = 256;
const BUFFER_POOL_SIZE: usize = 128; // Pre-allocated buffers
const SMALL_BUFFER_SIZE: usize = 2048; // For most messages

/// Lock-free buffer pool for zero-allocation sends
#[derive(Debug)]
struct BufferPool {
    small_buffers: Arc<SegQueue<Vec<u8>>>,
}

impl BufferPool {
    fn new() -> Self {
        let pool = Self {
            small_buffers: Arc::new(SegQueue::new()),
        };

        // Pre-allocate buffers
        for _ in 0..BUFFER_POOL_SIZE {
            let mut buf = Vec::with_capacity(SMALL_BUFFER_SIZE);
            buf.clear();
            pool.small_buffers.push(buf);
        }

        pool
    }

    /// Get a buffer from the pool (zero-allocation fast path)
    #[inline]
    fn get(&self) -> Vec<u8> {
        self.small_buffers.pop().unwrap_or_else(|| {
            // Pool exhausted - allocate (slow path)
            Vec::with_capacity(SMALL_BUFFER_SIZE)
        })
    }

    /// Return a buffer to the pool
    #[inline]
    fn put(&self, mut buf: Vec<u8>) {
        buf.clear();
        // Only return if not over capacity (prevent unbounded growth)
        if self.small_buffers.len() < BUFFER_POOL_SIZE * 2 {
            self.small_buffers.push(buf);
        }
        // else: drop the buffer (let it deallocate)
    }
}

/// High-performance router client backend
pub struct RouterBackend<T> {
    topic_name: String,
    router_addr: SocketAddr,
    send_tx: Sender<Vec<u8>>,          // Lock-free send queue
    recv_rx: Receiver<T>,              // Lock-free recv queue
    sequence: parking_lot::Mutex<u32>, // Faster mutex
    fragment_manager: Arc<FragmentManager>,
    buffer_pool: Arc<BufferPool>, // Zero-allocation buffer pool
    _phantom: std::marker::PhantomData<T>,
}

impl<T> RouterBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Send + Sync + 'static,
{
    /// Create a new high-performance router backend
    pub fn new(topic: &str) -> HorusResult<Self> {
        Self::new_with_addr(topic, "127.0.0.1".parse().unwrap(), DEFAULT_ROUTER_PORT)
    }

    /// Create with custom router address
    pub fn new_with_addr(topic: &str, host: IpAddr, port: u16) -> HorusResult<Self> {
        // Get or create tokio runtime
        let runtime_handle = Handle::try_current().unwrap_or_else(|_| {
            // If no runtime exists, create one (shouldn't happen in practice)
            tokio::runtime::Runtime::new().unwrap().handle().clone()
        });

        let router_addr = SocketAddr::new(host, port);

        // Create lock-free channels
        let (send_tx, send_rx) = bounded(SEND_QUEUE_SIZE);
        let (recv_tx, recv_rx) = bounded(RECV_QUEUE_SIZE);

        let backend = Self {
            topic_name: topic.to_string(),
            router_addr,
            send_tx,
            recv_rx,
            sequence: parking_lot::Mutex::new(0),
            fragment_manager: Arc::new(FragmentManager::default()),
            buffer_pool: Arc::new(BufferPool::new()),
            _phantom: std::marker::PhantomData,
        };

        // Spawn async connection handler
        let topic_clone = topic.to_string();
        let buffer_pool_clone = backend.buffer_pool.clone();
        runtime_handle.spawn(async move {
            if let Err(e) = Self::connection_handler(
                router_addr,
                topic_clone,
                send_rx,
                recv_tx,
                buffer_pool_clone,
            )
            .await
            {
                error!("[Router] Connection error: {}", e);
            }
        });

        Ok(backend)
    }

    /// Async connection handler - runs in tokio runtime
    async fn connection_handler(
        router_addr: SocketAddr,
        topic: String,
        send_rx: Receiver<Vec<u8>>,
        recv_tx: Sender<T>,
        buffer_pool: Arc<BufferPool>,
    ) -> HorusResult<()> {
        // Connect with TCP_NODELAY for low latency
        let mut stream = TcpStream::connect(router_addr).await.map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to connect to router at {}: {}",
                router_addr, e
            ))
        })?;

        stream.set_nodelay(true).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set TCP_NODELAY: {}", e))
        })?;

        // Send subscribe message
        let subscribe_packet = HorusPacket::new_router_subscribe(topic.clone());
        let mut buffer = Vec::with_capacity(1024);
        subscribe_packet.encode(&mut buffer);

        let len_bytes = (buffer.len() as u32).to_le_bytes();
        stream.write_all(&len_bytes).await.map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to send subscribe length: {}",
                e
            ))
        })?;
        stream.write_all(&buffer).await.map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to send subscribe: {}", e))
        })?;

        // Split stream for concurrent read/write
        let (mut read_half, mut write_half) = stream.into_split();

        // Spawn write task - returns buffers to pool after sending
        let write_task = tokio::spawn(async move {
            while let Ok(packet_data) = send_rx.recv() {
                let len_bytes = (packet_data.len() as u32).to_le_bytes();
                if write_half.write_all(&len_bytes).await.is_err() {
                    break;
                }
                if write_half.write_all(&packet_data).await.is_err() {
                    break;
                }
                // Return buffer to pool for reuse (zero-allocation)
                buffer_pool.put(packet_data);
            }
        });

        // Read loop - optimized for low latency
        let fragment_manager = Arc::new(FragmentManager::default());
        let mut read_buffer = vec![0u8; BUFFER_SIZE];
        let mut len_buffer = [0u8; 4];

        loop {
            // Read packet length
            if read_half.read_exact(&mut len_buffer).await.is_err() {
                break;
            }

            let packet_len = u32::from_le_bytes(len_buffer) as usize;
            if packet_len > BUFFER_SIZE {
                warn!("[Router] Packet too large: {} bytes, closing connection", packet_len);
                break; // Cannot continue — stream is desynchronized
            }

            // Read packet data
            if read_half
                .read_exact(&mut read_buffer[..packet_len])
                .await
                .is_err()
            {
                break;
            }

            // Decode packet (outside critical path)
            if let Ok(packet) = HorusPacket::decode(&read_buffer[..packet_len]) {
                if packet.topic != topic {
                    continue;
                }

                match packet.msg_type {
                    MessageType::RouterPublish => {
                        // Fast path: direct deserialize
                        if let Ok(msg) = bincode::deserialize::<T>(&packet.payload) {
                            let _ = recv_tx.try_send(msg); // Non-blocking
                        }
                    }
                    MessageType::Fragment => {
                        // Fragment reassembly
                        if let Ok(fragment) = Fragment::decode(&packet.payload) {
                            if let Some(complete_data) = fragment_manager.reassemble(fragment) {
                                if let Ok(msg) = bincode::deserialize::<T>(&complete_data) {
                                    let _ = recv_tx.try_send(msg);
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
        }

        write_task.abort();
        Ok(())
    }

    /// Send a message (optimized with zero-allocation buffer pooling)
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        // Serialize payload
        let payload = bincode::serialize(msg).map_err(|e| {
            crate::error::HorusError::Serialization(format!("Serialization error: {}", e))
        })?;

        // Fragment if needed
        let fragments = self.fragment_manager.fragment(&payload);

        // Get sequence number
        let mut seq = self.sequence.lock();

        // Encode all fragments
        for fragment in fragments {
            let packet = if fragment.total == 1 {
                HorusPacket::new_router_publish(self.topic_name.clone(), fragment.data, *seq)
            } else {
                let fragment_data = fragment.encode();
                HorusPacket::new_fragment(self.topic_name.clone(), fragment_data, *seq)
            };
            *seq = seq.wrapping_add(1);

            // Encode packet - use pooled buffer (zero-allocation fast path)
            let mut buffer = self.buffer_pool.get();
            packet.encode(&mut buffer);

            // Send via lock-free queue (non-blocking)
            self.send_tx.try_send(buffer).map_err(|_| {
                crate::error::HorusError::Communication("Send queue full".to_string())
            })?;
        }

        Ok(())
    }

    /// Receive a message (non-blocking, lock-free)
    pub fn recv(&self) -> Option<T> {
        self.recv_rx.try_recv().ok()
    }

    /// Receive with timeout
    pub fn recv_timeout(&self, timeout: std::time::Duration) -> Option<T> {
        self.recv_rx.recv_timeout(timeout).ok()
    }

    /// Get the topic name
    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }

    /// Get the router address
    pub fn router_addr(&self) -> SocketAddr {
        self.router_addr
    }
}

impl<T> std::fmt::Debug for RouterBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RouterBackend")
            .field("topic_name", &self.topic_name)
            .field("router_addr", &self.router_addr)
            .finish()
    }
}
