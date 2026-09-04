//! What the CLI says when there is no working package registry.
//!
//! The hosted registry at `api.horusrobotics.dev` was suspended and answered
//! every request with HTTP 503 for an extended period. Nothing in the CLI said
//! so. `horus search sensor` printed "No plugins found matching 'sensor'" and
//! exited 0, because discovery collapsed the error into an empty vector:
//!
//! ```ignore
//! let packages = match client.search("", Some("plugin"), None) {
//!     Ok(pkgs) => pkgs,
//!     Err(_) => return Ok(Vec::new()), // Network error — skip
//! };
//! ```
//!
//! That `Err(_)` folded three different answers into one — *offline*, *the
//! registry is broken*, and *nothing matched your query* — and picked the
//! rendering that tells a new user this project has no ecosystem.
//!
//! These tests pin the distinction end-to-end, through the real binary, because
//! the bug was never in `RegistryClient::search` (which returned `Err`
//! correctly) but in what the layers above it did with that error.
//!
//! Tracked in <https://github.com/softmata/horus/issues/173>.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::process::Command;

fn horus() -> &'static str {
    env!("CARGO_BIN_EXE_horus")
}

/// A one-shot HTTP server that answers everything with `status`.
///
/// Deliberately a real socket rather than a mocked client: the regression was
/// about a status code surviving the trip from the transport up to stdout, and
/// a mock at the client boundary would skip exactly the layers that broke.
struct StubRegistry {
    port: u16,
    _thread: std::thread::JoinHandle<()>,
}

impl StubRegistry {
    fn answering(status: u16, reason: &'static str, body: &'static str) -> Self {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind an ephemeral port");
        let port = listener.local_addr().expect("local_addr").port();
        let thread = std::thread::spawn(move || {
            // Serve until the listener is dropped with the struct. Each
            // connection gets one response; the CLI makes at most a couple.
            for stream in listener.incoming() {
                let Ok(mut stream) = stream else { break };
                let _ = drain_request(&mut stream);
                let response = format!(
                    "HTTP/1.1 {status} {reason}\r\n\
                     Content-Type: text/html; charset=utf-8\r\n\
                     Content-Length: {}\r\n\
                     Connection: close\r\n\r\n{body}",
                    body.len()
                );
                let _ = stream.write_all(response.as_bytes());
                let _ = stream.flush();
            }
        });
        Self {
            port,
            _thread: thread,
        }
    }

    fn url(&self) -> String {
        format!("http://127.0.0.1:{}", self.port)
    }
}

fn drain_request(stream: &mut TcpStream) -> std::io::Result<()> {
    // Read just the request head. Enough to let the client finish writing;
    // we never need the body.
    let mut buf = [0u8; 1024];
    stream.set_read_timeout(Some(std::time::Duration::from_millis(500)))?;
    let _ = stream.read(&mut buf);
    Ok(())
}

/// Run `horus <args>` with the registry environment set explicitly.
///
/// `HORUS_REGISTRY_URL` is always set (to `""` for the unconfigured case) so
/// the test never depends on what the developer happens to have exported.
fn run(registry: &str, args: &[&str]) -> (bool, String) {
    let out = Command::new(horus())
        .args(args)
        .env("HORUS_REGISTRY_URL", registry)
        .env("HORUS_PLUGIN_REGISTRY_URL", registry)
        .env("NO_COLOR", "1")
        .output()
        .expect("horus binary runs");
    let mut combined = String::from_utf8_lossy(&out.stdout).into_owned();
    combined.push_str(&String::from_utf8_lossy(&out.stderr));
    (out.status.success(), combined)
}

// ── No registry configured ──────────────────────────────────────────────────

#[test]
fn search_says_no_registry_was_consulted_rather_than_showing_nothing() {
    let (_ok, output) = run("", &["search", "sensor"]);
    assert!(
        output.contains("HORUS_REGISTRY_URL"),
        "`horus search` must say the registry was not consulted and how to \
         point at one. Without this the output is indistinguishable from an \
         empty ecosystem. Got:\n{output}"
    );
    assert!(
        output.contains("issues/173"),
        "the note must link the tracking issue so the state is explicable. \
         Got:\n{output}"
    );
}

#[test]
fn search_json_reports_that_the_registry_was_not_consulted() {
    let (_ok, output) = run("", &["search", "sensor", "--json"]);
    let start = output.find('{').expect("JSON output");
    let end = output.rfind('}').expect("JSON output") + 1;
    let parsed: serde_json::Value =
        serde_json::from_str(&output[start..end]).expect("valid JSON on stdout");

    assert_eq!(
        parsed["registry"]["consulted"],
        serde_json::Value::Bool(false),
        "a script must be able to tell 'nothing matched' from 'nothing was \
         searched'; both render as `results: []`. Got:\n{parsed:#}"
    );
    assert_eq!(
        parsed["registry"]["reason"],
        serde_json::Value::String("not_configured".into()),
        "got:\n{parsed:#}"
    );
}

#[test]
fn publish_refuses_with_the_reason_when_no_registry_is_configured() {
    let (ok, output) = run("", &["publish", "--dry-run"]);
    assert!(
        !ok,
        "`horus publish` cannot succeed with nowhere to publish to. Got:\n{output}"
    );
    // Specifically HORUS_REGISTRY_URL, not merely the word "registry": with the
    // old dead default this command failed at the *authentication* step and
    // printed "Not authenticated with HORUS registry / run horus auth login" --
    // advice that could not be followed, since logging in needs the same dead
    // host. Naming the env var is what makes the message actionable, and
    // asserting on it is what stops the check sliding back behind the auth gate.
    assert!(
        output.contains("HORUS_REGISTRY_URL"),
        "the failure must name the escape hatch, and must be reached before the \
         auth check. Got:\n{output}"
    );
    assert!(
        !output.contains("horus auth login"),
        "telling the user to log in to a registry that does not exist sends them \
         in a circle. Got:\n{output}"
    );
}

// ── Registry configured but broken ──────────────────────────────────────────

#[test]
fn search_reports_a_503_registry_instead_of_an_empty_result() {
    // The exact shape the hosted registry was returning: Render's
    // "This service has been suspended by its owner".
    let stub = StubRegistry::answering(
        503,
        "Service Unavailable",
        "<html><body>This service has been suspended by its owner.</body></html>",
    );
    let (_ok, output) = run(&stub.url(), &["search", "sensor"]);

    assert!(
        output.contains("503") || output.to_lowercase().contains("unreachable"),
        "a registry answering 503 must be reported as a registry problem, not \
         rendered as 'no plugins found'. This is the regression: the status \
         code reached `RegistryClient::search` correctly and was then discarded \
         one layer up. Got:\n{output}"
    );
}

#[test]
fn search_json_distinguishes_a_broken_registry_from_an_empty_one() {
    let stub = StubRegistry::answering(503, "Service Unavailable", "suspended");
    let (_ok, output) = run(&stub.url(), &["search", "sensor", "--json"]);

    let start = output.find('{').expect("JSON output");
    let end = output.rfind('}').expect("JSON output") + 1;
    let parsed: serde_json::Value =
        serde_json::from_str(&output[start..end]).expect("valid JSON on stdout");

    assert_eq!(
        parsed["registry"]["consulted"],
        serde_json::Value::Bool(false),
        "got:\n{parsed:#}"
    );
    assert_eq!(
        parsed["registry"]["reason"],
        serde_json::Value::String("unreachable".into()),
        "a 503 is 'unreachable', distinct from 'not_configured' — the two need \
         different fixes from the user. Got:\n{parsed:#}"
    );
}

/// A configured, working registry must NOT produce a note.
///
/// Without this, "always print the disclaimer" would pass every test above
/// while being wrong.
#[test]
fn a_healthy_registry_produces_no_registry_note() {
    let stub = StubRegistry::answering(200, "OK", r#"{"results":[]}"#);
    let (_ok, output) = run(&stub.url(), &["search", "sensor"]);

    assert!(
        !output.contains("HORUS_REGISTRY_URL"),
        "the registry answered normally; there is nothing to explain and the \
         note must stay out of the way. Got:\n{output}"
    );
    assert!(!output.contains("issues/173"), "got:\n{output}");
}
