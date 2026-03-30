use horus_cpp_macros::horus_api;

pub struct LoanedSample {
    data: Vec<u8>,
}

#[horus_api]
impl LoanedSample {
    #[cpp(returns = "unique_ptr")]
    pub fn new(size: usize) -> Self {
        LoanedSample { data: vec![0; size] }
    }

    #[cpp(raii)]
    pub fn release(self) {
        drop(self.data);
    }
}

fn main() {
    let s = LoanedSample::new(1024);
    s.release();
}
