use horus_cpp_macros::horus_api;

pub struct Foo {
    val: u32,
}

#[horus_api]
impl Foo {
    #[cpp(invalid_hint)]
    pub fn bar(&self) -> u32 {
        self.val
    }
}

fn main() {}
