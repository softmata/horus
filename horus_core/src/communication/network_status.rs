//! Network transport status for monitoring.

/// Network transport status for monitoring
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NetworkStatus {
    node_name: String,
    transport_type: String,
    local_endpoint: Option<String>,
    remote_endpoints: Vec<String>,
    network_topics_pub: Vec<String>,
    network_topics_sub: Vec<String>,
    bytes_sent: u64,
    bytes_received: u64,
    packets_sent: u64,
    packets_received: u64,
    timestamp_secs: u64,
}

impl NetworkStatus {
    /// Create a new network status
    pub fn new(node_name: &str, transport_type: &str) -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        Self {
            node_name: node_name.to_string(),
            transport_type: transport_type.to_string(),
            local_endpoint: None,
            remote_endpoints: Vec::new(),
            network_topics_pub: Vec::new(),
            network_topics_sub: Vec::new(),
            bytes_sent: 0,
            bytes_received: 0,
            packets_sent: 0,
            packets_received: 0,
            timestamp_secs: now,
        }
    }

    /// Check if status is fresh (within last N seconds)
    pub fn is_fresh(&self, max_age_secs: u64) -> bool {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        now.saturating_sub(self.timestamp_secs) <= max_age_secs
    }

    // ── Getters ──

    pub fn node_name(&self) -> &str {
        &self.node_name
    }

    pub fn transport_type(&self) -> &str {
        &self.transport_type
    }

    pub fn local_endpoint(&self) -> Option<&str> {
        self.local_endpoint.as_deref()
    }

    pub fn remote_endpoints(&self) -> &[String] {
        &self.remote_endpoints
    }

    pub fn network_topics_pub(&self) -> &[String] {
        &self.network_topics_pub
    }

    pub fn network_topics_sub(&self) -> &[String] {
        &self.network_topics_sub
    }

    pub fn bytes_sent(&self) -> u64 {
        self.bytes_sent
    }

    pub fn bytes_received(&self) -> u64 {
        self.bytes_received
    }

    pub fn packets_sent(&self) -> u64 {
        self.packets_sent
    }

    pub fn packets_received(&self) -> u64 {
        self.packets_received
    }

    pub fn timestamp_secs(&self) -> u64 {
        self.timestamp_secs
    }

    // ── Builder / mutation methods ──

    pub fn with_local_endpoint(mut self, endpoint: String) -> Self {
        self.local_endpoint = Some(endpoint);
        self
    }

    pub fn add_remote_endpoint(&mut self, endpoint: String) {
        self.remote_endpoints.push(endpoint);
    }

    pub fn add_network_topic_pub(&mut self, topic: String) {
        self.network_topics_pub.push(topic);
    }

    pub fn add_network_topic_sub(&mut self, topic: String) {
        self.network_topics_sub.push(topic);
    }

    pub fn add_bytes_sent(&mut self, n: u64) {
        self.bytes_sent += n;
    }

    pub fn add_bytes_received(&mut self, n: u64) {
        self.bytes_received += n;
    }

    pub fn add_packets_sent(&mut self, n: u64) {
        self.packets_sent += n;
    }

    pub fn add_packets_received(&mut self, n: u64) {
        self.packets_received += n;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_network_status_new() {
        let status = NetworkStatus::new("test_node", "SharedMemory");
        assert_eq!(status.node_name(), "test_node");
        assert_eq!(status.transport_type(), "SharedMemory");
        assert!(status.local_endpoint().is_none());
        assert!(status.remote_endpoints().is_empty());
        assert_eq!(status.bytes_sent(), 0);
        assert_eq!(status.bytes_received(), 0);
    }

    #[test]
    fn test_network_status_is_fresh() {
        let status = NetworkStatus::new("test_node", "Udp");
        // Just created, should be fresh
        assert!(status.is_fresh(5));
    }

    #[test]
    fn test_network_status_clone() {
        let mut status = NetworkStatus::new("test_node", "Udp");
        status.add_bytes_sent(1000);
        status.add_bytes_received(500);
        status.add_remote_endpoint("192.168.1.1:9000".to_string());

        let cloned = status.clone();
        assert_eq!(cloned.bytes_sent(), 1000);
        assert_eq!(cloned.remote_endpoints().len(), 1);
    }
}
