/// Message fragmentation for large payloads
///
/// Automatically splits messages larger than MTU into fragments and reassembles them
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const DEFAULT_MTU: usize = 1400; // Conservative MTU (1500 - headers)
const FRAGMENT_TIMEOUT: Duration = Duration::from_secs(5);
/// Maximum number of concurrent in-flight fragment groups to prevent memory exhaustion
/// from malicious or misbehaving senders flooding unique fragment IDs.
const MAX_PENDING_GROUPS: usize = 256;

/// A single fragment of a larger message
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Fragment {
    pub fragment_id: u64, // Unique ID for this fragmented message
    pub index: u16,       // Fragment index (0-based)
    pub total: u16,       // Total number of fragments
    pub data: Vec<u8>,    // Fragment payload
}

impl Fragment {
    pub fn encode(&self) -> Vec<u8> {
        bincode::serialize(self).unwrap()
    }

    pub fn decode(data: &[u8]) -> Result<Self, Box<bincode::ErrorKind>> {
        bincode::deserialize(data)
    }
}

/// Manages fragmentation and reassembly
pub struct FragmentManager {
    mtu: usize,
    next_fragment_id: AtomicU64,
    pending_fragments: Arc<Mutex<HashMap<u64, FragmentedMessage>>>,
}

impl FragmentManager {
    pub fn new(mtu: Option<usize>) -> Self {
        Self {
            mtu: mtu.unwrap_or(DEFAULT_MTU),
            next_fragment_id: AtomicU64::new(0),
            pending_fragments: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    /// Split data into fragments if needed
    pub fn fragment(&self, data: &[u8]) -> Vec<Fragment> {
        if data.len() <= self.mtu {
            // No fragmentation needed, return single fragment
            return vec![Fragment {
                fragment_id: self.next_fragment_id.fetch_add(1, Ordering::Relaxed),
                index: 0,
                total: 1,
                data: data.to_vec(),
            }];
        }

        // Need fragmentation
        let fragment_id = self.next_fragment_id.fetch_add(1, Ordering::Relaxed);
        let chunks: Vec<&[u8]> = data.chunks(self.mtu).collect();
        if chunks.len() > u16::MAX as usize {
            // Message too large for fragment count to fit in u16.
            // Return single unfragmented chunk — the receiver will handle the oversize data.
            return vec![Fragment {
                fragment_id,
                index: 0,
                total: 1,
                data: data.to_vec(),
            }];
        }
        let total = chunks.len() as u16;

        chunks
            .into_iter()
            .enumerate()
            .map(|(i, chunk)| Fragment {
                fragment_id,
                index: i as u16,
                total,
                data: chunk.to_vec(),
            })
            .collect()
    }

    /// Add a received fragment and return complete message if all fragments received
    pub fn reassemble(&self, fragment: Fragment) -> Option<Vec<u8>> {
        let mut pending = self.pending_fragments.lock().unwrap();

        // Single fragment (not fragmented)
        if fragment.total == 1 {
            return Some(fragment.data);
        }

        // Multi-fragment message — limit pending groups to prevent memory exhaustion
        if !pending.contains_key(&fragment.fragment_id)
            && pending.len() >= MAX_PENDING_GROUPS
        {
            // Drop fragment when too many groups are in flight
            return None;
        }

        let fragmented = pending
            .entry(fragment.fragment_id)
            .or_insert_with(|| FragmentedMessage::new(fragment.total));

        let fragment_id = fragment.fragment_id;
        fragmented.add_fragment(fragment);

        if fragmented.is_complete() {
            // All fragments received, assemble and remove from pending
            Some(pending.remove(&fragment_id).unwrap().assemble())
        } else {
            None
        }
    }

    /// Clean up stale fragments (called periodically)
    pub fn cleanup_stale(&self) {
        let mut pending = self.pending_fragments.lock().unwrap();
        let now = Instant::now();

        pending.retain(|_, fragmented| {
            now.duration_since(fragmented.first_received) < FRAGMENT_TIMEOUT
        });
    }

    pub fn mtu(&self) -> usize {
        self.mtu
    }
}

impl Default for FragmentManager {
    fn default() -> Self {
        Self::new(None)
    }
}

/// Incomplete fragmented message being reassembled
struct FragmentedMessage {
    fragment_id: u64,
    fragments: Vec<Option<Vec<u8>>>,
    received_count: usize,
    first_received: Instant,
}

impl FragmentedMessage {
    fn new(total: u16) -> Self {
        Self {
            fragment_id: 0,
            fragments: vec![None; total as usize],
            received_count: 0,
            first_received: Instant::now(),
        }
    }

    fn add_fragment(&mut self, fragment: Fragment) {
        self.fragment_id = fragment.fragment_id;

        let index = fragment.index as usize;
        if index < self.fragments.len() && self.fragments[index].is_none() {
            self.fragments[index] = Some(fragment.data);
            self.received_count += 1;
        }
    }

    fn is_complete(&self) -> bool {
        self.received_count == self.fragments.len()
    }

    fn assemble(self) -> Vec<u8> {
        let mut result = Vec::new();
        for data in self.fragments.into_iter().flatten() {
            result.extend_from_slice(&data);
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_fragmentation_needed() {
        let manager = FragmentManager::new(Some(1500));
        let data = vec![0u8; 1000]; // Smaller than MTU

        let fragments = manager.fragment(&data);
        assert_eq!(fragments.len(), 1);
        assert_eq!(fragments[0].total, 1);
        assert_eq!(fragments[0].index, 0);
    }

    #[test]
    fn test_fragmentation_needed() {
        let manager = FragmentManager::new(Some(1000));
        let data = vec![0u8; 3500]; // 3.5x MTU

        let fragments = manager.fragment(&data);
        assert_eq!(fragments.len(), 4); // ceil(3500/1000) = 4

        // Check fragment metadata
        for (i, frag) in fragments.iter().enumerate() {
            assert_eq!(frag.index, i as u16);
            assert_eq!(frag.total, 4);
            assert_eq!(frag.fragment_id, 0);
        }
    }

    #[test]
    fn test_reassembly() {
        let manager = FragmentManager::new(Some(1000));
        let original_data = vec![42u8; 2500];

        // Fragment
        let fragments = manager.fragment(&original_data);
        assert_eq!(fragments.len(), 3);

        // Reassemble in order
        for (i, frag) in fragments.iter().enumerate() {
            let result = manager.reassemble(frag.clone());
            if i < 2 {
                assert!(result.is_none()); // Not complete yet
            } else {
                assert!(result.is_some()); // Complete!
                assert_eq!(result.unwrap(), original_data);
            }
        }
    }

    #[test]
    fn test_reassembly_out_of_order() {
        let manager = FragmentManager::new(Some(1000));
        let original_data = vec![42u8; 2500];

        // Fragment
        let mut fragments = manager.fragment(&original_data);

        // Reassemble out of order (2, 0, 1)
        let frag2 = fragments.pop().unwrap();
        let frag0 = fragments.remove(0);
        let frag1 = fragments.pop().unwrap();

        assert!(manager.reassemble(frag2).is_none());
        assert!(manager.reassemble(frag0).is_none());

        let result = manager.reassemble(frag1);
        assert!(result.is_some());
        assert_eq!(result.unwrap(), original_data);
    }

    #[test]
    fn test_single_fragment_reassembly() {
        let manager = FragmentManager::new(Some(1500));
        let data = vec![42u8; 500];

        let fragments = manager.fragment(&data);
        assert_eq!(fragments.len(), 1);

        // Should return immediately (single fragment)
        let result = manager.reassemble(fragments[0].clone());
        assert!(result.is_some());
        assert_eq!(result.unwrap(), data);
    }
}
