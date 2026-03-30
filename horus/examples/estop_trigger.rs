/// Trigger an emergency stop by publishing to _horus.estop.
/// Used for E2E testing of launch e-stop propagation.
///
/// Usage: ./estop_trigger
use horus::prelude::*;
use horus_types::EmergencyStop;

fn main() -> Result<()> {
    let topic = Topic::<EmergencyStop>::new("_horus.estop")?;
    println!("Publishing EmergencyStop to _horus.estop...");
    topic.send(EmergencyStop::default());
    // Send a few times to ensure delivery
    std::thread::sleep(std::time::Duration::from_millis(50));
    topic.send(EmergencyStop::default());
    std::thread::sleep(std::time::Duration::from_millis(50));
    topic.send(EmergencyStop::default());
    println!("E-stop sent.");
    Ok(())
}
