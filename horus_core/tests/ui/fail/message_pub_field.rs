// `pub` on a field: the most natural thing a Rust programmer writes.
// Before the terminal @munch arm this reported "recursion limit reached".
use horus_core::message;

message! {
    Foo { pub a: f32, pub b: f32, }
}

fn main() {}
