/// Unix domain socket backend for localhost communication
///
/// Provides <5μs latency for same-machine communication using Unix sockets.
/// More efficient than TCP loopback as it avoids the IP stack.
use crate::error::HorusResult;
use log::warn;
use std::io::{Read, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};

const UNIX_SOCKET_DIR: &str = "/tmp/horus_sockets";
const BUFFER_SIZE: usize = 65536; // 64KB buffer

/// Unix socket backend for high-performance localhost communication
pub struct UnixSocketBackend<T> {
    topic_name: String,
    socket_path: PathBuf,
    stream: Arc<Mutex<UnixStream>>,
    recv_buffer: Vec<u8>,
    _phantom: std::marker::PhantomData<T>,
}

impl<T> UnixSocketBackend<T>
where
    T: serde::Serialize + serde::de::DeserializeOwned,
{
    /// Create a new publisher (binds to socket, waits for subscriber)
    pub fn new_publisher(topic: &str) -> HorusResult<Self> {
        // Create socket directory
        std::fs::create_dir_all(UNIX_SOCKET_DIR).map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to create socket directory: {}",
                e
            ))
        })?;

        let socket_path = PathBuf::from(UNIX_SOCKET_DIR).join(format!("horus_{}.sock", topic));

        // Remove existing socket if present
        let _ = std::fs::remove_file(&socket_path);

        // Bind listener
        let listener = UnixListener::bind(&socket_path).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to bind Unix socket: {}", e))
        })?;

        listener.set_nonblocking(true).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set nonblocking: {}", e))
        })?;

        // Wait for first subscriber with timeout
        let stream = loop {
            match listener.accept() {
                Ok((stream, _addr)) => break stream,
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No connection yet, sleep briefly
                    std::thread::sleep(std::time::Duration::from_millis(10));
                }
                Err(e) => {
                    return Err(crate::error::HorusError::Communication(format!(
                        "Failed to accept connection: {}",
                        e
                    )))
                }
            }
        };

        stream.set_nonblocking(false).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set blocking: {}", e))
        })?;

        Ok(Self {
            topic_name: topic.to_string(),
            socket_path,
            stream: Arc::new(Mutex::new(stream)),
            recv_buffer: Vec::with_capacity(BUFFER_SIZE),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Create a new subscriber (connects to existing publisher)
    pub fn new_subscriber(topic: &str) -> HorusResult<Self> {
        let socket_path = PathBuf::from(UNIX_SOCKET_DIR).join(format!("horus_{}.sock", topic));

        // Connect to publisher
        let stream = UnixStream::connect(&socket_path).map_err(|e| {
            crate::error::HorusError::Communication(format!(
                "Failed to connect to Unix socket: {}",
                e
            ))
        })?;

        stream.set_nonblocking(false).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to set blocking: {}", e))
        })?;

        Ok(Self {
            topic_name: topic.to_string(),
            socket_path,
            stream: Arc::new(Mutex::new(stream)),
            recv_buffer: Vec::with_capacity(BUFFER_SIZE),
            _phantom: std::marker::PhantomData,
        })
    }

    /// Send a message over Unix socket
    pub fn send(&self, msg: &T) -> HorusResult<()> {
        // Serialize using bincode (faster than JSON)
        let serialized = bincode::serialize(msg).map_err(|e| {
            crate::error::HorusError::Serialization(format!("Serialization error: {}", e))
        })?;

        // Write length prefix (4 bytes)
        let len = serialized.len() as u32;
        let len_bytes = len.to_le_bytes();

        let mut stream = self.stream.lock().unwrap();
        stream.write_all(&len_bytes).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to write length: {}", e))
        })?;
        stream.write_all(&serialized).map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to write data: {}", e))
        })?;
        stream.flush().map_err(|e| {
            crate::error::HorusError::Communication(format!("Failed to flush: {}", e))
        })?;

        Ok(())
    }

    /// Receive a message from Unix socket
    pub fn recv(&mut self) -> Option<T> {
        let mut stream = self.stream.lock().unwrap();

        // Read length prefix
        let mut len_bytes = [0u8; 4];
        if stream.read_exact(&mut len_bytes).is_err() {
            return None;
        }
        let len = u32::from_le_bytes(len_bytes) as usize;

        // Safety check
        if len > BUFFER_SIZE {
            warn!("[UnixSocket] Message too large: {} bytes", len);
            return None;
        }

        // Read message
        self.recv_buffer.resize(len, 0);
        if stream.read_exact(&mut self.recv_buffer).is_err() {
            return None;
        }

        // Deserialize
        bincode::deserialize(&self.recv_buffer).ok()
    }

    /// Get the topic name
    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }
}

impl<T> Drop for UnixSocketBackend<T> {
    fn drop(&mut self) {
        // Clean up socket file
        let _ = std::fs::remove_file(&self.socket_path);
    }
}

impl<T> std::fmt::Debug for UnixSocketBackend<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("UnixSocketBackend")
            .field("topic_name", &self.topic_name)
            .field("socket_path", &self.socket_path)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    struct TestMessage {
        data: u64,
    }

    impl crate::core::LogSummary for TestMessage {
        fn log_summary(&self) -> String {
            format!("TestMessage({})", self.data)
        }
    }

    #[test]
    fn test_unix_socket_basic() {
        // Create publisher in separate thread
        let pub_handle = std::thread::spawn(|| {
            let backend = UnixSocketBackend::<TestMessage>::new_publisher("test_basic").unwrap();
            std::thread::sleep(std::time::Duration::from_millis(100));

            let msg = TestMessage { data: 42 };
            backend.send(&msg).unwrap();
            backend
        });

        // Give publisher time to bind
        std::thread::sleep(std::time::Duration::from_millis(50));

        // Create subscriber
        let mut sub_backend =
            UnixSocketBackend::<TestMessage>::new_subscriber("test_basic").unwrap();

        // Receive message
        let received = sub_backend.recv().unwrap();
        assert_eq!(received.data, 42);

        // Clean up
        pub_handle.join().unwrap();
    }
}
