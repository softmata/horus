//! Local driver factory registry.
//!
//! Enables users to register their own driver types by name so that
//! `HardwareSet::local("name")` can instantiate them from config.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//! use horus::drivers::{register_driver, DriverParams};
//!
//! struct MyDriver { /* ... */ }
//!
//! impl MyDriver {
//!     fn from_params(params: &DriverParams) -> HorusResult<Self> {
//!         let port = params.get::<String>("port")?;
//!         Ok(Self { /* ... */ })
//!     }
//! }
//!
//! impl Node for MyDriver {
//!     fn tick(&mut self) { /* ... */ }
//! }
//!
//! // Register so [drivers] config can find it
//! register_driver!(MyDriver, MyDriver::from_params);
//! ```

use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};

use crate::core::Node;
use crate::error::HorusResult;

use super::params::DriverParams;

/// Factory function type for creating driver nodes from config params.
pub type DriverFactory = fn(&DriverParams) -> HorusResult<Box<dyn Node>>;

/// Global registry of local driver factories.
///
/// Populated by [`register_driver!`] at init time.
/// Queried by [`HardwareSet::local()`](super::HardwareSet::local).
static REGISTRY: LazyLock<Mutex<HashMap<String, DriverFactory>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Register a driver factory by name.
///
/// Called by the [`register_driver!`] macro. Can also be called directly.
pub fn register(name: &str, factory: DriverFactory) {
    REGISTRY
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(name.to_string(), factory);
}

/// Look up a registered driver factory by name.
///
/// Returns `None` if no factory is registered with that name.
pub fn lookup(name: &str) -> Option<DriverFactory> {
    REGISTRY
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .get(name)
        .copied()
}

/// List all registered driver names.
pub fn list_registered() -> Vec<String> {
    REGISTRY
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .keys()
        .cloned()
        .collect()
}

/// Register a local driver factory so `[drivers]` config can instantiate it.
///
/// The factory function receives [`DriverParams`] from the `[drivers.NAME]`
/// config table and returns a `Box<dyn Node>`.
///
/// # Usage
///
/// ```rust,ignore
/// use horus::prelude::*;
/// use horus::drivers::{DriverParams, register_driver};
///
/// struct ConveyorDriver { /* ... */ }
///
/// impl ConveyorDriver {
///     fn from_params(params: &DriverParams) -> HorusResult<Self> {
///         let port = params.get::<String>("port")?;
///         Ok(Self { /* ... */ })
///     }
/// }
///
/// impl Node for ConveyorDriver {
///     fn tick(&mut self) { /* ... */ }
/// }
///
/// register_driver!(ConveyorDriver, ConveyorDriver::from_params);
/// ```
///
/// Then in `horus.toml`:
/// ```toml
/// [drivers.conveyor]
/// node = "ConveyorDriver"
/// port = "/dev/ttyACM0"
/// ```
///
/// And in `main.rs`:
/// ```rust,ignore
/// let mut hw = drivers::load()?;
/// sched.add(hw.local("conveyor")?).build()?;
/// ```
#[macro_export]
macro_rules! register_driver {
    ($name:ty, $factory:expr) => {
        const _: () = {
            #[used]
            #[allow(non_upper_case_globals)]
            #[doc(hidden)]
            #[link_section = ".init_array"]
            static __register_driver: extern "C" fn() = {
                extern "C" fn __init() {
                    $crate::drivers::registry::register(
                                stringify!($name),
                                |params: &$crate::drivers::DriverParams|
                                    -> $crate::error::HorusResult<Box<dyn $crate::core::Node>> {
                                    let node = $factory(params)?;
                                    Ok(Box::new(node))
                                },
                            );
                }
                __init
            };
        };
    };
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Node;

    struct TestStubNode {
        value: u32,
    }

    impl TestStubNode {
        fn from_params(params: &DriverParams) -> HorusResult<Self> {
            let value = params.get_or("value", 42u32);
            Ok(Self { value })
        }
    }

    impl Node for TestStubNode {
        fn name(&self) -> &str {
            "test_stub"
        }
        fn tick(&mut self) {}
    }

    #[test]
    fn register_and_lookup() {
        // Register manually (not via macro — macro uses .init_array which
        // requires linker support that may not work in test context)
        register("TestStubNode", |params| {
            Ok(Box::new(TestStubNode::from_params(params)?))
        });

        let factory = lookup("TestStubNode");
        assert!(factory.is_some());

        let params = DriverParams::empty();
        let node = factory.unwrap()(&params).unwrap();
        assert_eq!(node.name(), "test_stub");
    }

    #[test]
    fn lookup_missing_returns_none() {
        assert!(lookup("NonexistentDriver").is_none());
    }

    #[test]
    fn register_overwrites() {
        register("Overwrite", |_| Ok(Box::new(TestStubNode { value: 1 })));
        register("Overwrite", |_| Ok(Box::new(TestStubNode { value: 2 })));

        // Second registration wins
        let factory = lookup("Overwrite").unwrap();
        let _node = factory(&DriverParams::empty()).unwrap();
    }

    #[test]
    fn factory_receives_params() {
        register("ParamTest", |params| {
            let val = params.get::<u32>("value")?;
            Ok(Box::new(TestStubNode { value: val }))
        });

        let mut map = std::collections::HashMap::new();
        map.insert("value".to_string(), toml::Value::Integer(99));
        let params = DriverParams::new(map);

        let factory = lookup("ParamTest").unwrap();
        let _node = factory(&params).unwrap();
    }

    #[test]
    fn list_includes_registered() {
        register("ListTest", |_| Ok(Box::new(TestStubNode { value: 0 })));

        let names = list_registered();
        assert!(names.contains(&"ListTest".to_string()));
    }
}
