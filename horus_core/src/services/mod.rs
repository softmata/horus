//! HORUS Service System
//!
//! Services provide synchronous request/response communication — the HORUS
//! equivalent of ROS2 services.
//!
//! Unlike topics (fire-and-forget pub/sub), a service call blocks the caller
//! until the server returns a response or a timeout elapses.
//!
//! # Quick Start
//!
//! ```rust,ignore
//! use horus_core::{service, services::{ServiceServerBuilder, ServiceClient}};
//! use std::time::Duration;
//!
//! // Define the service
//! service! {
//!     AddTwoInts {
//!         request { a: i64, b: i64 }
//!         response { sum: i64 }
//!     }
//! }
//!
//! // Server side
//! let _server = ServiceServerBuilder::<AddTwoInts>::new()
//!     .on_request(|req| Ok(AddTwoIntsResponse { sum: req.a + req.b }))
//!     .build()?;
//!
//! // Client side
//! let mut client = ServiceClient::<AddTwoInts>::new()?;
//! let resp = client.call(AddTwoIntsRequest { a: 3, b: 4 }, 1_u64.secs())?;
//! println!("sum = {}", resp.sum);
//! ```
//!
//! # Architecture
//!
//! Each service uses two HORUS topics internally:
//!
//! ```text
//! Client                          Server
//!   |                                |
//!   |-- {name}.request -----------> |  (ServiceRequest<Req>)
//!   |                                |
//!   | <---------- {name}.response -- |  (ServiceResponse<Res>)
//!   |                                |
//! ```
//!
//! Multiple clients can call the same service concurrently.  Each request
//! carries a unique `request_id`; the client filters the response topic for
//! the matching ID.
//!
//! # ROS2 Equivalents
//!
//! | ROS2 | HORUS |
//! |------|-------|
//! | `ros2 service list` | `horus service list` |
//! | `ros2 service call /name pkg/Type '{...}'` | `horus service call <name> '{...}'` |
//! | `ros2 service type /name` | `horus service type <name>` |
//! | `ros2 service find <type>` | `horus service find <type>` |
//! | `ServiceServer<Req, Res>` | [`ServiceServerBuilder<S>`] |
//! | `ServiceClient<Req, Res>` | [`ServiceClient<S>`] / [`AsyncServiceClient<S>`] |

#[doc(hidden)]
pub mod client;
#[doc(hidden)]
pub mod macros;
#[doc(hidden)]
pub mod server;
#[doc(hidden)]
pub mod types;

// Public re-exports
#[doc(hidden)]
pub use client::PendingServiceCall;
pub use client::{AsyncServiceClient, ServiceClient};
#[doc(hidden)]
pub use server::RequestHandler;
pub use server::{ServiceServer, ServiceServerBuilder};
#[doc(hidden)]
pub use types::ServiceInfo;
pub use types::{Service, ServiceError, ServiceRequest, ServiceResponse, ServiceResult};

/// Prelude for convenient imports.
///
/// ```rust,ignore
/// use horus_core::services::prelude::*;
/// ```
pub mod prelude {
    pub use super::{
        AsyncServiceClient, Service, ServiceClient, ServiceError, ServiceInfo, ServiceRequest,
        ServiceResponse, ServiceResult, ServiceServer, ServiceServerBuilder,
    };
}
