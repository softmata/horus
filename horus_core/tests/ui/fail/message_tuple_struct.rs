// Tuple structs are not supported; say so rather than recursing.
use horus_core::message;

message! {
    Foo(f32, f32);
}

fn main() {}
