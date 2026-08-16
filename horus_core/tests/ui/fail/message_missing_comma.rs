// A missing separator between fields.
use horus_core::message;

message! {
    Foo {
        a: f32
        b: f32,
    }
}

fn main() {}
