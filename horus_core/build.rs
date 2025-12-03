//! Build script for horus_core
//!
//! This configures CUDA linking when the `cuda` feature is enabled.

fn main() {
    // CUDA linking configuration
    #[cfg(feature = "cuda")]
    {
        // Try common CUDA installation paths
        let cuda_paths = [
            "/usr/lib/x86_64-linux-gnu", // Ubuntu package location (check first)
            "/usr/local/cuda/lib64",
            "/usr/local/cuda-12/lib64",
            "/usr/local/cuda-12.2/lib64",
            "/usr/local/cuda-11/lib64",
            "/opt/cuda/lib64",
        ];

        let mut found_cuda = false;

        // Check CUDA_PATH environment variable first
        if let Ok(cuda_path) = std::env::var("CUDA_PATH") {
            let lib_path = format!("{}/lib64", cuda_path);
            if std::path::Path::new(&lib_path).exists() {
                println!("cargo:rustc-link-search=native={}", lib_path);
                found_cuda = true;
            }
        }

        // Check LD_LIBRARY_PATH
        if !found_cuda {
            if let Ok(ld_path) = std::env::var("LD_LIBRARY_PATH") {
                for path in ld_path.split(':') {
                    let cudart_path = format!("{}/libcudart.so", path);
                    if std::path::Path::new(&cudart_path).exists() {
                        println!("cargo:rustc-link-search=native={}", path);
                        found_cuda = true;
                        break;
                    }
                }
            }
        }

        // Try common installation paths
        if !found_cuda {
            for path in &cuda_paths {
                let cudart_path = format!("{}/libcudart.so", path);
                if std::path::Path::new(&cudart_path).exists() {
                    println!("cargo:rustc-link-search=native={}", path);
                    found_cuda = true;
                    break;
                }
            }
        }

        // Link against CUDA runtime
        if found_cuda {
            println!("cargo:rustc-link-lib=dylib=cudart");
        } else {
            // Warn but don't fail - allows compilation on systems without CUDA
            println!("cargo:warning=CUDA libraries not found. GPU features will fail at runtime.");
            // Still try to link, let it fail gracefully
            println!("cargo:rustc-link-lib=dylib=cudart");
        }
    }

    // Rerun if CUDA paths change
    println!("cargo:rerun-if-env-changed=CUDA_PATH");
    println!("cargo:rerun-if-env-changed=LD_LIBRARY_PATH");
}
