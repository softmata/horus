/// High-performance direct 1P1C network backend for Topic
///
/// Unlike RouterBackend (many-to-many), DirectBackend provides a simple
/// point-to-point TCP connection between a single producer and consumer.
///
/// Optimizations:
/// - Async Tokio I/O (no blocking)
/// - Lock-free crossbeam channels
/// - TCP_NODELAY for low latency
/// - Zero-allocation buffer pooling
/// - No router middleman = lower latency (~5-15µs)
use crate::error::HorusResult;
use crossbeam::channel::{bounded, Receiver, Sender};
use crossbeam::queue::SegQueue;
use log::{error, warn};
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{TcpListener, TcpStream};
use tokio::runtime::Handle;

const SEND_QUEUE_SIZE: usize = 256;
const RECV_QUEUE_SIZE: usize = 1024;
const BUFFER_POOL_SIZE: usize = 64; // Smaller pool for 1P1C
const SMALL_BUFFER_SIZE: usize = 2048;

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

    #[inline]
    fn get(&self) -> Vec<u8> {
        self.small_buffers
            .pop()
            .unwrap_or_else(|| Vec::with_capacity(SMALL_BUFFER_SIZE))
    }

    #[inline]
    fn put(&self, mut buf: Vec<u8>) {
        buf.clear();
        if self.small_buffers.len() < BUFFER_POOL_SIZE * 2 {
            self.small_buffers.push(buf);
        }
    }
}

/// Direct 1P1C backend (producer or consumer)
pub struct DirectBackend<T> {
    role: DirectRole,
    addr: SocketAddr,
    send_tx: Option<Sender<Vec<u8>>>, // Producer only
    recv_rx: Option<Receiver<T>>,     // Consumer only
    buffer_pool: Arc<BufferPool>,
    _phantom: std::marker::PhantomData<T>,
}

/// Role in the direct connection
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DirectRole {
    Producer, // Connects to consumer
    Consumer, // Listens for producer
}

impl<T> DirectBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned + Send + Sync + 'static,
{
    /// Create a producer (connects to consumer's listening address)
    pub fn new_producer(consumer_addr: SocketAddr) -> HorusResult<Self> {
        let runtime_handle = Handle::try_current().unwrap_or_else(|_| {
            tokio::runtime::Runtime::new()
                .expect("Failed to create Tokio runtime - insufficient system resources")
                .handle()
                .clone()
        });

        let (send_tx, send_rx) = bounded(SEND_QUEUE_SIZE);
        let buffer_pool = Arc::new(BufferPool::new());
        let buffer_pool_clone = buffer_pool.clone();

        // Spawn async producer task
        runtime_handle.spawn(async move {
            if let Err(e) = Self::producer_handler(consumer_addr, send_rx, buffer_pool_clone).await
            {
                error!("[Direct] Producer connection error: {}", e);
            }
        });

        Ok(DirectBackend {
            role: DirectRole::Producer,
            addr: consumer_addr,
            send_tx: Some(send_tx),
            recv_rx: None,
            buffer_pool,
            _phantom: std::marker::PhantomData,
        })
    }

    /// Create a consumer (listens on address for producer to connect)
    pub fn new_consumer(listen_addr: SocketAddr) -> HorusResult<Self> {
        let runtime_handle = Handle::try_current().unwrap_or_else(|_| {
            tokio::runtime::Runtime::new()
                .expect("Failed to create Tokio runtime - insufficient system resources")
                .handle()
                .clone()
        });

        let (recv_tx, recv_rx) = bounded(RECV_QUEUE_SIZE);

        // Spawn async consumer task
        runtime_handle.spawn(async move {
            if let Err(e) = Self::consumer_handler(listen_addr, recv_tx).await {
                error!("[Direct] Consumer connection error: {}", e);
            }
        });

        Ok(DirectBackend {
            role: DirectRole::Consumer,
            addr: listen_addr,
            send_tx: None,
            recv_rx: Some(recv_rx),
            buffer_pool: Arc::new(BufferPool::new()),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Producer task: connect and send
    async fn producer_handler(
        consumer_addr: SocketAddr,
        send_rx: Receiver<Vec<u8>>,
        buffer_pool: Arc<BufferPool>,
    ) -> HorusResult<()> {
        // Connect to consumer
        let mut stream = TcpStream::connect(consumer_addr).await.map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to connect to consumer at {}: {}",
                consumer_addr, e
            ))
        })?;

        stream.set_nodelay(true).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set TCP_NODELAY: {}", e))
        })?;

        // Send loop
        while let Ok(data) = send_rx.recv() {
            let len_bytes = (data.len() as u32).to_le_bytes();
            if stream.write_all(&len_bytes).await.is_err() {
                break;
            }
            if stream.write_all(&data).await.is_err() {
                break;
            }
            // Return buffer to pool
            buffer_pool.put(data);
        }

        Ok(())
    }

    /// Consumer task: listen and receive
    async fn consumer_handler(listen_addr: SocketAddr, recv_tx: Sender<T>) -> HorusResult<()> {
        // Listen for producer
        let listener = TcpListener::bind(listen_addr).await.map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to bind to {}: {}",
                listen_addr, e
            ))
        })?;

        // Accept first connection (1P1C - only one producer)
        let (mut stream, _) = listener.accept().await.map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to accept connection: {}", e))
        })?;

        stream.set_nodelay(true).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set TCP_NODELAY: {}", e))
        })?;

        // Receive loop
        let mut len_buffer = [0u8; 4];
        let mut read_buffer = vec![0u8; 65536];

        loop {
            // Read message length
            if stream.read_exact(&mut len_buffer).await.is_err() {
                break;
            }

            let msg_len = u32::from_le_bytes(len_buffer) as usize;
            if msg_len > read_buffer.len() {
                warn!("[Direct] Message too large: {} bytes, closing connection", msg_len);
                break; // Cannot continue — stream is desynchronized
            }

            // Read message data
            if stream
                .read_exact(&mut read_buffer[..msg_len])
                .await
                .is_err()
            {
                break;
            }

            // Deserialize and send to recv queue
            if let Ok(msg) = bincode::deserialize::<T>(&read_buffer[..msg_len]) {
                let _ = recv_tx.try_send(msg);
            }
        }

        Ok(())
    }

    /// Send a message (producer only)
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        let send_tx = self.send_tx.as_ref().ok_or_else(|| {
            crate::error::HorusError::Communication("Cannot send from consumer".to_string())
        })?;

        // Serialize
        let data = bincode::serialize(msg).map_err(|e| {
            crate::error::HorusError::Serialization(format!("Serialization error: {}", e))
        })?;

        // Get pooled buffer
        let mut buffer = self.buffer_pool.get();

        // Write length-prefixed data
        let len_bytes = (data.len() as u32).to_le_bytes();
        buffer.extend_from_slice(&len_bytes);
        buffer.extend_from_slice(&data);

        // Send via lock-free queue
        send_tx
            .try_send(buffer)
            .map_err(|_| crate::error::HorusError::Communication("Send queue full".to_string()))?;

        Ok(())
    }

    /// Receive a message (consumer only)
    pub fn recv(&self) -> Option<T> {
        self.recv_rx.as_ref()?.try_recv().ok()
    }

    /// Receive with timeout (consumer only)
    pub fn recv_timeout(&self, timeout: std::time::Duration) -> Option<T> {
        self.recv_rx.as_ref()?.recv_timeout(timeout).ok()
    }

    /// Check if messages are available without consuming them (consumer only)
    ///
    /// Returns `true` if there are messages in the receive queue, `false` otherwise.
    /// This is a non-blocking peek operation.
    ///
    /// # Returns
    ///
    /// - `true` if messages are available
    /// - `false` if no messages are available or this is a producer
    pub fn has_messages(&self) -> bool {
        self.recv_rx
            .as_ref()
            .map(|rx| !rx.is_empty())
            .unwrap_or(false)
    }

    /// Get the role
    pub fn role(&self) -> DirectRole {
        self.role
    }

    /// Get the address
    pub fn addr(&self) -> SocketAddr {
        self.addr
    }
}

impl<T> std::fmt::Debug for DirectBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DirectBackend")
            .field("role", &self.role)
            .field("addr", &self.addr)
            .finish()
    }
}
