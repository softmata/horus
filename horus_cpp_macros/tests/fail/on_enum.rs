use horus_cpp_macros::horus_api;

#[horus_api]
pub enum Status {
    Running,
    Stopped,
}

fn main() {}
