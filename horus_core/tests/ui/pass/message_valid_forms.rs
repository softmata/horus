// Every shape the macro is documented to accept must still compile:
// serde-backed fields (String/Vec/Option), #[fixed] POD messages,
// several messages in one invocation, and doc comments.
use horus_core::message;

message! {
    /// A regular serde-backed message.
    Telemetry {
        label: String,
        samples: Vec<u8>,
        note: Option<String>,
        value: f32,
    }

    #[fixed]
    /// A zero-copy POD message.
    MotorCommand {
        voltage: f64,
        enabled: u8,
    }

    Heartbeat { seq: u64 }
}

fn main() {}
