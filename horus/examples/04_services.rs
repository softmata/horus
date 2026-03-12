//! # Example 4: Services (Request/Response)
//!
//! Services provide synchronous request/response RPC between nodes.
//! The server runs on a background thread; the client sends requests
//! and waits for responses.
//!
//! ```bash
//! cargo run --example 04_services
//! ```

use horus::prelude::*;


// Define a service using the service! macro
service! {
    /// Add two integers
    AddTwoInts {
        request {
            a: i64,
            b: i64,
        }
        response {
            sum: i64,
        }
    }
}

fn main() -> Result<()> {
    println!("=== HORUS Example 4: Services ===\n");

    // Start the server (runs on a background thread)
    let _server = ServiceServerBuilder::<AddTwoInts>::new()
        .on_request(|req| {
            let sum = req.a + req.b;
            println!("[Server] {} + {} = {}", req.a, req.b, sum);
            Ok(AddTwoIntsResponse { sum })
        })
        .build()?;

    println!("[Server] Listening for requests...\n");

    // Create a client
    let mut client = ServiceClient::<AddTwoInts>::new()?;

    // Send some requests
    for i in 1..=5 {
        let a = i * 10;
        let b = i * 3;
        println!("[Client] Requesting: {} + {}", a, b);

        match client.call(AddTwoIntsRequest { a, b }, 1_u64.secs()) {
            Ok(response) => println!("[Client] Got response: sum = {}\n", response.sum),
            Err(e) => println!("[Client] Error: {:?}\n", e),
        }
    }

    println!("Done!");
    Ok(())
}
