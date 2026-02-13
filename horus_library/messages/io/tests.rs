//! Unit tests for I/O message types

use super::*;

#[test]
fn test_digital_io() {
    let mut dio = DigitalIO::new(8);

    // Test pin operations
    assert!(dio.set_pin(0, true));
    assert!(dio.set_pin(7, true));
    assert!(!dio.set_pin(8, true)); // Out of range

    assert_eq!(dio.get_pin(0), Some(true));
    assert_eq!(dio.get_pin(1), Some(false));
    assert_eq!(dio.count_active(), 2);

    // Test bitmask
    let mask = dio.as_bitmask();
    assert_eq!(mask, 0x81); // Bits 0 and 7 set

    let mut dio2 = DigitalIO::new(8);
    dio2.from_bitmask(0x81);
    assert!(dio2.get_pin(0).unwrap());
    assert!(dio2.get_pin(7).unwrap());
}

#[test]
fn test_analog_io() {
    let mut aio = AnalogIO::new(4);

    assert!(aio.set_channel(0, 5.0));
    assert!(aio.set_channel(3, -2.5));
    assert!(!aio.set_channel(4, 1.0)); // Out of range

    assert_eq!(aio.get_channel(0), Some(5.0));
    assert_eq!(aio.get_channel(1), Some(0.0));

    // Test ADC conversion
    aio.set_channel_range(0, 0.0, 10.0);
    aio.resolution_bits = 12; // 12-bit ADC

    if let Some(raw) = aio.engineering_to_raw(0, 5.0) {
        // 5V should be about half scale (2047 for 12-bit)
        assert!((raw as i32 - 2047).abs() <= 1);
    }
}

#[test]
fn test_modbus_message() {
    let msg = ModbusMessage::read_holding_registers(1, 100, 10);
    assert_eq!(msg.unit_id, 1);
    assert_eq!(
        msg.function_code,
        ModbusMessage::FUNC_READ_HOLDING_REGISTERS
    );
    assert_eq!(msg.start_address, 100);
    assert_eq!(msg.quantity, 10);
    assert!(msg.is_request != 0);

    let write_msg = ModbusMessage::write_single_register(1, 200, 1234);
    assert_eq!(
        write_msg.function_code,
        ModbusMessage::FUNC_WRITE_SINGLE_REGISTER
    );
    assert_eq!(write_msg.data[0], 1234);
}

#[test]
fn test_network_status() {
    let mut status = NetworkStatus::new("eth0");
    status.set_ip_from_string("192.168.1.100").unwrap();

    assert_eq!(status.ip_to_string(), "192.168.1.100");
    assert!(status.set_ip_from_string("invalid").is_err());

    status.tx_packets = 1000;
    status.tx_errors = 10;
    assert_eq!(status.packet_loss_percent(), 1.0);
}

#[test]
fn test_safety_relay() {
    let mut relay = SafetyRelayStatus::new("SR001");
    assert!(relay.is_safe_state()); // Default state should be safe

    relay.fault_present = 1;
    assert!(!relay.is_safe_state());

    relay.fault_present = 0;
    relay.safety_outputs[0] = 1;
    relay.safety_outputs[2] = 1;
    assert_eq!(relay.active_output_count(), 2);
}
