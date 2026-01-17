// Snake Game Controller - Backend Logic
// This handles keyboard/joystick input and publishes snake state
// Run snakesim_gui in another terminal to see the visualization

use horus::prelude::*;  // Includes KeyboardInputNode, JoystickInputNode, and all messages


// Snake control node that converts input codes to SnakeState
// Supports both keyboard and joystick input
struct SnakeControlNode {
    keyboard_subscriber: Topic<KeyboardInput>,
    joystick_subscriber: Topic<JoystickInput>,
    snake_publisher: Hub<u32>,
}

impl SnakeControlNode {
    fn new() -> Result<Self> {
        Ok(Self {
            keyboard_subscriber: Topic::new("keyboard_input")?,
            joystick_subscriber: Topic::new("joystick_input")?,
            snake_publisher: Topic::new("snakestate")?,
        })
    }
}

impl Node for SnakeControlNode {
    fn name(&self) -> &'static str {
        "SnakeControlNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        // Process keyboard input
        if let Some(input) = self.keyboard_subscriber.recv(&mut ctx) {
            ctx.log_debug(&format!(
                "Received key code: {}, pressed: {}",
                input.code, input.pressed
            ));

            if input.pressed {
                // Map keyboard codes to snake directions
                // Using standard arrow key codes (37-40) or WASD keys
                let direction = match input.code {
                    38 | 87 => 1, // ArrowUp or W -> Up
                    40 | 83 => 2, // ArrowDown or S -> Down
                    37 | 65 => 3, // ArrowLeft or A -> Left
                    39 | 68 => 4, // ArrowRight or D -> Right
                    _ => return,  // Ignore other keys
                };

                ctx.log_debug(&format!("Publishing direction: {}", direction));
                let _ = self.snake_publisher.send(direction, &mut ctx);
            }
        }

        // Process joystick input
        if let Some(input) = self.joystick_subscriber.recv(&mut ctx) {
            if input.is_button() && input.pressed {
                // Map D-pad buttons to snake directions
                let button_name = input.get_element_name();
                ctx.log_debug(&format!("Received joystick button: {}", button_name));

                let direction = match button_name.as_str() {
                    "DPadUp" => Some(1),    // Up
                    "DPadDown" => Some(2),  // Down
                    "DPadLeft" => Some(3),  // Left
                    "DPadRight" => Some(4), // Right
                    _ => None,
                };

                if let Some(dir) = direction {
                    ctx.log_debug(&format!("Publishing joystick direction: {}", dir));
                    let _ = self.snake_publisher.send(dir, &mut ctx);
                }
            } else if input.is_axis() {
                // Map left stick to snake directions (with threshold)
                let axis_name = input.get_element_name();
                let value = input.value;

                // Only respond to significant axis movements (> 0.5)
                if value.abs() > 0.5 {
                    let direction = match axis_name.as_str() {
                        "LeftStickX" => {
                            if value > 0.5 {
                                Some(4) // Right
                            } else if value < -0.5 {
                                Some(3) // Left
                            } else {
                                None
                            }
                        }
                        "LeftStickY" => {
                            if value > 0.5 {
                                Some(2) // Down
                            } else if value < -0.5 {
                                Some(1) // Up
                            } else {
                                None
                            }
                        }
                        _ => None,
                    };

                    if let Some(dir) = direction {
                        ctx.log_debug(&format!("Publishing joystick axis direction: {}", dir));
                        let _ = self.snake_publisher.send(dir, &mut ctx);
                    }
                }
            }
        }
    }
}

fn main() -> Result<()> {
    eprintln!("=== Snake Game Controller ===");
    eprintln!("Starting snake scheduler with keyboard and joystick input support...");
    eprintln!("\nControls:");
    eprintln!("  Keyboard: Arrow Keys or WASD - Control snake direction");
    eprintln!("  Joystick: D-Pad or Left Stick - Control snake direction");
    eprintln!("  ESC - Quit keyboard capture");
    eprintln!("\nMake sure to run snakesim_gui in another terminal!");
    eprintln!("===============================\n");

    let mut sched = Scheduler::new().name("SnakeScheduler");

    // Create keyboard input node - captures real keyboard input from terminal
    let keyboard_input_node = KeyboardInputNode::new_with_topic("keyboard_input")?;

    // Create joystick input node - captures gamepad input
    let joystick_input_node = JoystickInputNode::new()?; // Uses default "joystick_input" topic

    // Snake control node subscribes to both keyboard_input and joystick_input topics
    let snake_control_node = SnakeControlNode::new()?;

    sched.add(Box::new(keyboard_input_node), 0, Some(true));
    sched.add(Box::new(joystick_input_node), 0, Some(true)); // Same priority as keyboard
    sched.add(Box::new(snake_control_node), 1, Some(true));

    // Run the scheduler loop - continuously ticks all nodes
    sched.tick(&["KeyboardInputNode", "JoystickInputNode", "SnakeControlNode"])
}
