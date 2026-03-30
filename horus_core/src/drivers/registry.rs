//! Node factory registry.
//!
//! Enables users to register node types by name so that
//! `hardware::load()` can instantiate them from `[hardware]` config.
//!
//! # Example
//!
//! ```rust,ignore
//! use horus::prelude::*;
//! use horus::hardware::NodeParams;
//!
//! struct MyDriver { /* ... */ }
//!
//! impl MyDriver {
//!     fn from_params(params: &NodeParams) -> HorusResult<Self> {
//!         let port = params.get::<String>("port")?;
//!         Ok(Self { /* ... */ })
//!     }
//! }
//!
//! impl Node for MyDriver {
//!     fn tick(&mut self) { /* ... */ }
//! }
//!
//! // Register so [hardware] config can find it
//! register_driver!(MyDriver, MyDriver::from_params);
//! ```

use std::collections::HashMap;
use std::sync::{LazyLock, Mutex};

use crate::core::Node;
use crate::error::HorusResult;

use super::params::NodeParams;

/// Factory function type for creating nodes from config params.
pub type NodeFactory = fn(&NodeParams) -> HorusResult<Box<dyn Node>>;

/// Backward-compatibility alias.
pub type DriverFactory = NodeFactory;

/// Global registry of node factories.
///
/// Populated by [`register_driver!`] at init time.
/// Queried by [`hardware::load()`](super::load).
static REGISTRY: LazyLock<Mutex<HashMap<String, NodeFactory>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));

/// Register a node factory by name.
///
/// Called by the [`register_driver!`] macro. Can also be called directly.
pub fn register(name: &str, factory: NodeFactory) {
    REGISTRY
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(name.to_string(), factory);
}

/// Look up a registered node factory by name.
///
/// Returns `None` if no factory is registered with that name.
pub fn lookup(name: &str) -> Option<NodeFactory> {
    REGISTRY
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .get(name)
        .copied()
}

/// List all registered node type names.
pub fn list_registered() -> Vec<String> {
    REGISTRY
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .keys()
        .cloned()
        .collect()
}

/// Register a node factory so `[hardware]` config can instantiate it.
///
/// The factory function receives [`NodeParams`] from the `[hardware.NAME]`
/// config table and returns a `Box<dyn Node>`.
///
/// # Usage
///
/// ```rust,ignore
/// use horus::prelude::*;
/// use horus::hardware::NodeParams;
///
/// struct ConveyorDriver { /* ... */ }
///
/// impl ConveyorDriver {
///     fn from_params(params: &NodeParams) -> HorusResult<Self> {
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
/// [hardware.conveyor]
/// use = "ConveyorDriver"
/// port = "/dev/ttyACM0"
/// ```
///
/// And in `main.rs`:
/// ```rust,ignore
/// let nodes = hardware::load()?;
/// for (name, node) in nodes {
///     sched.add(node).build()?;
/// }
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
                                |params: &$crate::drivers::NodeParams|
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
        #[allow(dead_code)]
        value: u32,
    }

    impl TestStubNode {
        fn from_params(params: &NodeParams) -> HorusResult<Self> {
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
        register("TestStubNode", |params| {
            Ok(Box::new(TestStubNode::from_params(params)?))
        });

        let factory = lookup("TestStubNode");
        assert!(factory.is_some());

        let params = NodeParams::empty();
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

        let factory = lookup("Overwrite").unwrap();
        let _node = factory(&NodeParams::empty()).unwrap();
    }

    #[test]
    fn factory_receives_params() {
        register("ParamTest", |params| {
            let val = params.get::<u32>("value")?;
            Ok(Box::new(TestStubNode { value: val }))
        });

        let mut map = std::collections::HashMap::new();
        map.insert("value".to_string(), toml::Value::Integer(99));
        let params = NodeParams::new(map);

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
