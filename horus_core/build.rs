//! Build script for horus_core

fn main() {
    // Rerun if environment changes
    println!("cargo:rerun-if-env-changed=CUDA_PATH");
    println!("cargo:rerun-if-env-changed=LD_LIBRARY_PATH");
}
