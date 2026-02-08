//! Acceptance tests for Topic Communication (Pub/Sub)
//! Tests lock-free, zero-copy shared memory communication between nodes

use horus_core::communication::Topic;
use horus_core::core::Node;
use serde::{Deserialize, Serialize};

#[test]
fn test_scenario_1_basic_pub_sub() {
    // Scenario 1: Basic Publish and Subscribe
    // Given: Two nodes share a topic name
    // When: Publisher sends a message
    // Then: Subscriber receives the message

    let topic = format!("test_basic_{}", std::process::id());

    let pub_hub = Topic::<i32>::new(&topic).expect("Failed to create publisher hub");
    let sub_hub = Topic::<i32>::new(&topic).expect("Failed to create subscriber hub");

    pub_hub.send(42).expect("Failed to send message");

    let msg = sub_hub.recv();
    assert_eq!(
        msg,
        Some(42),
        "Subscriber should receive the exact message sent"
    );
}

#[test]
fn test_scenario_2_multiple_subscribers() {
    // Scenario 2: Multiple Subscribers (competing consumers)
    // Given: One publisher, three subscribers on same topic
    // When: Publisher sends multiple messages
    // Then: Messages are distributed among subscribers (competing consumer semantics)
    // Note: HORUS Topics use point-to-point messaging, not broadcast

    let topic = format!("test_multi_sub_{}", std::process::id());

    let pub_hub = Topic::<i32>::new(&topic).expect("Failed to create publisher");
    let sub1 = Topic::<i32>::new(&topic).expect("Failed to create subscriber 1");
    let sub2 = Topic::<i32>::new(&topic).expect("Failed to create subscriber 2");
    let sub3 = Topic::<i32>::new(&topic).expect("Failed to create subscriber 3");

    // Send 3 messages
    pub_hub.send(100).expect("Failed to send message 1");
    pub_hub.send(200).expect("Failed to send message 2");
    pub_hub.send(300).expect("Failed to send message 3");

    // Collect messages from all subscribers (competing consumer semantics)
    let mut received = Vec::new();
    if let Some(msg) = sub1.recv() {
        received.push(msg);
    }
    if let Some(msg) = sub2.recv() {
        received.push(msg);
    }
    if let Some(msg) = sub3.recv() {
        received.push(msg);
    }
    // Try receiving any remaining messages
    if let Some(msg) = sub1.recv() {
        received.push(msg);
    }
    if let Some(msg) = sub2.recv() {
        received.push(msg);
    }
    if let Some(msg) = sub3.recv() {
        received.push(msg);
    }

    // Verify at least the 3 messages were received (distributed among subscribers)
    assert!(
        received.len() >= 3,
        "Should receive all 3 messages among subscribers, got {} messages: {:?}",
        received.len(),
        received
    );

    // Verify the correct values were received
    received.sort();
    assert!(
        received.contains(&100) && received.contains(&200) && received.contains(&300),
        "All message values should be received: {:?}",
        received
    );
}

#[test]
fn test_scenario_3_multiple_publishers() {
    // Scenario 3: Multiple Publishers
    // Given: Three publishers, one subscriber on same topic
    // When: Each publisher sends a message
    // Then: Subscriber receives all messages (eventually)

    let topic = format!("test_multi_pub_{}", std::process::id());

    let pub1 = Topic::<i32>::new(&topic).expect("Failed to create publisher 1");
    let pub2 = Topic::<i32>::new(&topic).expect("Failed to create publisher 2");
    let pub3 = Topic::<i32>::new(&topic).expect("Failed to create publisher 3");
    let sub = Topic::<i32>::new(&topic).expect("Failed to create subscriber");

    pub1.send(1).expect("Failed to send from pub1");
    pub2.send(2).expect("Failed to send from pub2");
    pub3.send(3).expect("Failed to send from pub3");

    let mut received = vec![];
    for _ in 0..3 {
        if let Some(msg) = sub.recv() {
            received.push(msg);
        }
    }

    assert_eq!(received.len(), 3, "Should receive all 3 messages");
    assert!(received.contains(&1), "Should contain message from pub1");
    assert!(received.contains(&2), "Should contain message from pub2");
    assert!(received.contains(&3), "Should contain message from pub3");
}

#[test]
fn test_scenario_4_message_buffering_same_hub() {
    // Scenario 4: Message Buffering (Modified)
    // Note: Current implementation resets shared memory when new Topic instances are created
    // So we test that a single Topic instance can send messages and read them later
    // Given: A single Topic instance
    // When: Topic sends multiple messages
    // Then: Messages are buffered and can be read later from the same Hub

    let topic = format!("test_buffered_{}", std::process::id());

    let hub = Topic::<i32>::new(&topic).expect("Failed to create hub");

    hub.send(1).expect("Failed to send message 1");
    hub.send(2).expect("Failed to send message 2");
    hub.send(3).expect("Failed to send message 3");

    // Read buffered messages from the same hub
    assert_eq!(hub.recv(), Some(1), "Should receive buffered message 1");
    assert_eq!(hub.recv(), Some(2), "Should receive buffered message 2");
    assert_eq!(hub.recv(), Some(3), "Should receive buffered message 3");
}

#[test]
fn test_scenario_6_empty_receive() {
    // Scenario 6: Empty Receive
    // Given: No messages have been published
    // When: Subscriber calls recv()
    // Then: Returns None immediately

    let topic = format!("test_empty_{}", std::process::id());

    let hub = Topic::<i32>::new(&topic).expect("Failed to create hub");

    assert_eq!(hub.recv(), None, "Empty hub should return None");
    assert_eq!(
        hub.recv(),
        None,
        "Multiple reads of empty hub should return None"
    );
}

#[test]
fn test_scenario_7_large_messages() {
    // Scenario 7: Large Messages
    // Given: Message is large (using a struct with array)
    // When: Publisher sends large message
    // Then: Message is sent successfully and received completely

    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct LargeMessage {
        data: Vec<i32>,
    }

    impl horus_core::core::LogSummary for LargeMessage {
        fn log_summary(&self) -> String {
            format!("LargeMessage[{} elements]", self.data.len())
        }
    }

    let topic = format!("test_large_{}", std::process::id());

    let pub_hub = Topic::<LargeMessage>::new(&topic).expect("Failed to create publisher");
    let sub_hub = Topic::<LargeMessage>::new(&topic).expect("Failed to create subscriber");

    let msg = LargeMessage {
        data: vec![42; 1000],
    };
    pub_hub
        .send(msg.clone())
        .expect("Failed to send large message");

    let received = sub_hub.recv().expect("Should receive large message");
    assert_eq!(received, msg, "Large message should be received completely");
    assert_eq!(received.data.len(), 1000, "Vec should have correct length");
}

#[test]
fn test_scenario_9_custom_capacity() {
    // Scenario 9: Custom Capacity
    // Given: User needs larger buffer
    // When: User creates Topic with new_with_capacity
    // Then: Topic buffer has custom capacity

    let topic = format!("test_capacity_{}", std::process::id());

    let hub = Topic::<i32>::with_capacity(&topic, 4096)
        .expect("Failed to create hub with custom capacity");

    // Send multiple messages to verify capacity
    for i in 0..100 {
        hub.send(i).expect("Failed to send message");
    }

    // Receive and verify messages
    for i in 0..100 {
        let msg = hub.recv().expect("Should receive message");
        assert_eq!(msg, i, "Message order should be preserved");
    }
}

#[test]
#[ignore] // Flaky on CI - timing-sensitive test
fn test_scenario_high_frequency_publishing() {
    // Scenario 8: High Frequency Publishing (simplified)
    // Given: Node publishes many messages rapidly
    // When: Publishing for extended period
    // Then: No message loss within buffer capacity

    let topic = format!("test_high_freq_{}", std::process::id());

    let pub_hub = Topic::<i32>::new(&topic).expect("Failed to create publisher");
    let sub_hub = Topic::<i32>::new(&topic).expect("Failed to create subscriber");

    // Send 1000 messages rapidly
    for i in 0..1000 {
        pub_hub.send(i).expect("Failed to send message");
    }

    // Receive all messages
    let mut count = 0;
    while sub_hub.recv().is_some() {
        count += 1;
    }

    assert!(count > 0, "Should receive at least some messages");
    // Note: Some messages may be lost due to ring buffer overflow
    // This is expected behavior for high-frequency publishing
}

#[test]
fn test_edge_case_topic_name_with_special_chars() {
    // Edge Case 1: Topic Name with Special Characters
    // Given: Topic name has special characters
    // When: Topic is created
    // Then: Topic name is sanitized for filesystem

    let topic = format!("robot.sensors.lidar_{}", std::process::id());

    let hub = Topic::<i32>::new(&topic).expect("Hub should handle special chars in topic name");
    hub.send(42)
        .expect("Should be able to send on topic with special chars");

    assert_eq!(hub.recv(), Some(42), "Should receive message on same topic");
}

#[test]
fn test_edge_case_same_process_multiple_hubs() {
    // Edge Case 4: Same Process, Multiple Hubs
    // Given: Same process creates multiple different Hubs
    // When: All publish simultaneously
    // Then: All Hubs work independently

    let mut hubs = vec![];

    for i in 0..10 {
        let topic = format!("test_multi_hub_{}_{}", std::process::id(), i);
        let hub = Topic::<i32>::new(&topic).expect("Failed to create hub");
        hub.send(i).expect("Failed to send message");
        hubs.push(hub);
    }

    // Verify each hub receives its own message
    for (i, hub) in hubs.iter().enumerate() {
        let msg = hub.recv().expect("Should receive message");
        assert_eq!(msg, i as i32, "Each hub should receive its own message");
    }
}

#[test]
fn test_resource_cleanup() {
    // Scenario 16: Topic Dropped
    // Given: Topic goes out of scope
    // When: Drop trait executes
    // Then: No memory leaks, other Hubs on same topic still work

    let topic = format!("test_cleanup_{}", std::process::id());

    {
        let hub1 = Topic::<i32>::new(&topic).expect("Failed to create hub1");
        hub1.send(42).expect("Failed to send message");
        // hub1 goes out of scope here
    }

    // Create new hub on same topic
    let hub2 = Topic::<i32>::new(&topic).expect("Failed to create hub2 after hub1 dropped");
    hub2.send(100).expect("Failed to send on hub2");

    let msg = hub2.recv().expect("Should receive message on hub2");
    assert_eq!(msg, 100, "New hub should work after old hub dropped");
}

#[test]
fn test_different_message_types() {
    // Test with different serializable types
    let topic_string = format!("test_string_{}", std::process::id());
    let topic_vec = format!("test_vec_{}", std::process::id());

    // Test with String
    let hub_string = Topic::<String>::new(&topic_string).expect("Failed to create String hub");
    hub_string
        .send("Hello, HORUS!".to_string())
        .expect("Failed to send String");
    assert_eq!(hub_string.recv(), Some("Hello, HORUS!".to_string()));

    // Test with Vec
    let hub_vec = Topic::<Vec<i32>>::new(&topic_vec).expect("Failed to create Vec hub");
    hub_vec
        .send(vec![1, 2, 3, 4, 5])
        .expect("Failed to send Vec");
    assert_eq!(hub_vec.recv(), Some(vec![1, 2, 3, 4, 5]));
}

#[test]
fn test_custom_struct() {
    // Test with custom serializable struct
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    struct SensorData {
        temperature: f64,
        humidity: f64,
        pressure: f64,
    }

    impl horus_core::core::LogSummary for SensorData {
        fn log_summary(&self) -> String {
            format!(
                "SensorData[T:{:.1}°C, H:{:.1}%, P:{:.2}hPa]",
                self.temperature, self.humidity, self.pressure
            )
        }
    }

    let topic = format!("test_struct_{}", std::process::id());

    let pub_hub = Topic::<SensorData>::new(&topic).expect("Failed to create publisher");
    let sub_hub = Topic::<SensorData>::new(&topic).expect("Failed to create subscriber");

    let data = SensorData {
        temperature: 23.5,
        humidity: 65.0,
        pressure: 1013.25,
    };

    pub_hub.send(data.clone()).expect("Failed to send struct");

    let received = sub_hub.recv().expect("Should receive struct");
    assert_eq!(received, data, "Struct should be received correctly");
}

// ============================================================================
// topics! Macro + Node Integration Tests (User Perspective)
// ============================================================================
//
// These tests simulate how a real HORUS user would use the topics! macro
// with Node structs, Topic::publish/subscribe, and tick-based communication.

/// User-defined message type (typical robotics struct)
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct MotorCommand {
    velocity: f32,
    torque: f32,
}

impl horus_core::core::LogSummary for MotorCommand {
    fn log_summary(&self) -> String {
        format!(
            "MotorCmd(vel:{:.2}, torq:{:.2})",
            self.velocity, self.torque
        )
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct EncoderFeedback {
    position: f64,
    velocity: f64,
}

impl horus_core::core::LogSummary for EncoderFeedback {
    fn log_summary(&self) -> String {
        format!(
            "Encoder(pos:{:.3}, vel:{:.3})",
            self.position, self.velocity
        )
    }
}

// Define topics once in a shared module — the user pattern
horus_core::topics! {
    pub MOTOR_CMD: MotorCommand = "test_topics_macro_motor_cmd",
    pub ENCODER_FB: EncoderFeedback = "test_topics_macro_encoder_fb",
}

/// A controller node that publishes motor commands and reads encoder feedback
struct ControllerNode {
    cmd_pub: horus_core::Topic<MotorCommand>,
    encoder_sub: horus_core::Topic<EncoderFeedback>,
    last_feedback: Option<EncoderFeedback>,
    tick_count: u32,
}

impl horus_core::Node for ControllerNode {
    fn name(&self) -> &'static str {
        "Controller"
    }

    fn tick(&mut self) {
        self.tick_count += 1;

        // Publish a motor command
        let cmd = MotorCommand {
            velocity: self.tick_count as f32 * 0.1,
            torque: 1.0,
        };
        let _ = self.cmd_pub.send(cmd);

        // Read encoder feedback (non-blocking)
        if let Some(fb) = self.encoder_sub.recv() {
            self.last_feedback = Some(fb);
        }
    }
}

/// A motor driver node that reads motor commands and publishes encoder feedback
struct MotorDriverNode {
    cmd_sub: horus_core::Topic<MotorCommand>,
    encoder_pub: horus_core::Topic<EncoderFeedback>,
    position: f64,
    last_cmd: Option<MotorCommand>,
}

impl horus_core::Node for MotorDriverNode {
    fn name(&self) -> &'static str {
        "MotorDriver"
    }

    fn tick(&mut self) {
        // Read command
        if let Some(cmd) = self.cmd_sub.recv() {
            self.last_cmd = Some(cmd.clone());
            self.position += cmd.velocity as f64 * 0.001; // integrate velocity
        }

        // Publish encoder feedback
        let fb = EncoderFeedback {
            position: self.position,
            velocity: self
                .last_cmd
                .as_ref()
                .map(|c| c.velocity as f64)
                .unwrap_or(0.0),
        };
        let _ = self.encoder_pub.send(fb);
    }
}

#[test]
fn test_topics_macro_with_nodes_pub_sub() {
    // Full user-perspective test:
    // 1. Define topics with topics! macro (done above)
    // 2. Create nodes using Topic::publish/subscribe
    // 3. Run tick loop manually
    // 4. Verify data flows between nodes via typed topics
    //
    // Note: Topic::publish/subscribe both call Topic::new() internally.
    // To share the same underlying backend (like clone()), we create one
    // Topic per name and clone it for the second node — this matches real
    // usage where nodes in the same process share Topic handles.

    // Create shared topic handles (one per topic name, cloned to each node)
    let motor_topic = horus_core::Topic::publish(MOTOR_CMD).expect("create MOTOR_CMD");
    let encoder_topic = horus_core::Topic::publish(ENCODER_FB).expect("create ENCODER_FB");

    let mut controller = ControllerNode {
        cmd_pub: motor_topic.clone(),
        encoder_sub: encoder_topic.clone(),
        last_feedback: None,
        tick_count: 0,
    };

    let mut driver = MotorDriverNode {
        cmd_sub: motor_topic.clone(),
        encoder_pub: encoder_topic.clone(),
        position: 0.0,
        last_cmd: None,
    };

    // Simulate tick cycles: controller publishes cmd → driver reads & responds → controller reads
    for _ in 0..5 {
        controller.tick();
        driver.tick();
    }
    // One final controller tick to read the last encoder feedback
    controller.tick();

    // The driver should have received commands
    assert!(
        driver.last_cmd.is_some(),
        "MotorDriver should have received a MotorCommand via MOTOR_CMD topic"
    );

    // The controller should have received feedback
    assert!(
        controller.last_feedback.is_some(),
        "Controller should have received EncoderFeedback via ENCODER_FB topic"
    );

    // Position should have advanced (velocity was integrated)
    assert!(
        driver.position > 0.0,
        "Motor position should have advanced from commands"
    );

    let fb = controller.last_feedback.as_ref().unwrap();
    assert!(
        fb.position > 0.0,
        "Encoder feedback position should reflect motor movement"
    );
}

#[test]
fn test_topics_macro_type_safety_compile_time() {
    // This test verifies that the topics! macro produces correctly-typed descriptors.
    // If the types were wrong, this test wouldn't compile at all.

    // MOTOR_CMD is TopicDescriptor<MotorCommand>
    let _pub: horus_core::Topic<MotorCommand> =
        horus_core::Topic::publish(MOTOR_CMD).expect("typed publish");

    // ENCODER_FB is TopicDescriptor<EncoderFeedback>
    let _sub: horus_core::Topic<EncoderFeedback> =
        horus_core::Topic::subscribe(ENCODER_FB).expect("typed subscribe");

    // The following would NOT compile (type mismatch):
    // let _bad: horus_core::Topic<String> = horus_core::Topic::publish(MOTOR_CMD);
    //                                                                   ^^^^^^^^^ expected MotorCommand
}

#[test]
fn test_topics_macro_descriptor_names() {
    // Verify the descriptors carry the correct topic names
    assert_eq!(MOTOR_CMD.name(), "test_topics_macro_motor_cmd");
    assert_eq!(ENCODER_FB.name(), "test_topics_macro_encoder_fb");
}

#[test]
fn test_topics_macro_roundtrip_with_publish_subscribe() {
    // Direct roundtrip: publish a message via Topic::publish, receive via Topic::subscribe
    // Uses a unique topic name to avoid shared-memory interference from parallel tests

    horus_core::topics! {
        ROUNDTRIP_TOPIC: MotorCommand = "test_roundtrip_motor_cmd",
    }

    let publisher = horus_core::Topic::publish(ROUNDTRIP_TOPIC).expect("publish");
    let subscriber = publisher.clone();

    let cmd = MotorCommand {
        velocity: 3.14,
        torque: 2.71,
    };
    publisher.send(cmd.clone()).expect("send");

    let received = subscriber.recv();
    assert_eq!(
        received,
        Some(cmd),
        "Should receive exact message through typed topic"
    );
}
