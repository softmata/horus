# HORUS - The Unified Crate

The main entry point for the HORUS robotics framework, providing a clean and unified API.

## Usage

Simply use the unified crate:
```rust
use horus::prelude::*;
```

## What's Included

The `horus` crate re-exports everything you need:

- **Core Framework** (`horus_core`)
  - Node trait and types
  - Scheduler
  - Communication (Hub, Link)
  - Memory management

- **Standard Library** (`horus_library`)
  - Message types (KeyboardInput, JoystickInput, CmdVel, etc.)
  - Core algorithms
  - Reusable nodes

- **Macros** (`horus_macros`)
  - Procedural macros for code generation

- **Common Types**
  - Result and error types
  - Duration, Instant
  - Arc, Mutex
  - Serde traits

## Example

```rust
use horus::prelude::*;

pub struct MyNode {
    counter: u32,
}

impl Node for MyNode {
    fn name(&self) -> &'static str { "MyNode" }

    fn tick(&mut self) {
        self.counter += 1;
        hlog!(info, "Tick #{}", self.counter);
    }
}

fn main() -> Result<()> {
    // Your HORUS application here
    Ok(())
}
```

## Benefits

1. **Cleaner imports** - Single `use horus::prelude::*` statement
2. **Consistent API** - Everything through one crate
3. **Better discoverability** - All types in one place
4. **Simplified dependency** - Add only `horus` to your Cargo.toml