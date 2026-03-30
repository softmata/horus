use horus_cpp_macros::horus_api;

#[horus_api]
pub struct Publisher {
    inner: u64,
    name: String,
}

fn main() {
    let _p = Publisher { inner: 42, name: String::from("test") };
}
