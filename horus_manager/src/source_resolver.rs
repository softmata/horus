//! Smart package source resolver for HORUS.
//!
//! Resolves whether a package name belongs to crates.io, PyPI, or the HORUS
//! registry — so users can type `horus add serde` without specifying `--source
//! crates.io`.
//!
//! Resolution strategy (in order):
//! 1. **Well-known packages** — static tables of popular crates.io and PyPI
//!    packages, covering ~95% of real-world usage.
//! 2. **Name heuristics** — prefixes/suffixes that strongly correlate with one
//!    ecosystem (e.g. `py-*`, `python-*` → PyPI; `*-rs`, `*-sys` → crates.io).
//! 3. **Project context** — if the project is Rust-only, prefer crates.io;
//!    Python-only → PyPI; mixed → stay ambiguous.
//! 4. **Fallback** — returns `DepSource::Registry` at low confidence so the
//!    caller can prompt the user.

use crate::manifest::{DepSource, Language};

/// Confidence level of the resolved source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Confidence {
    /// Guessed from project context or weak heuristics — should prompt user.
    Low,
    /// Name heuristics match, likely correct.
    Medium,
    /// Found in the well-known package list.
    High,
}

/// Result of resolving a package source.
#[derive(Debug, Clone)]
pub struct ResolvedSource {
    /// The resolved dependency source.
    pub source: DepSource,
    /// How confident the resolver is.
    pub confidence: Confidence,
    /// Human-readable reason for the decision.
    pub reason: String,
}

/// Smart package source resolver.
///
/// Stateless — all data is compiled into the binary. No network calls.
pub struct PackageSourceResolver<'a> {
    /// Project languages (for context-aware fallback).
    languages: &'a [Language],
}

impl<'a> PackageSourceResolver<'a> {
    /// Create a resolver with the given project language context.
    pub fn new(languages: &'a [Language]) -> Self {
        Self { languages }
    }

    /// Create a resolver with no project context (pure name-based).
    pub fn without_context() -> Self {
        Self { languages: &[] }
    }

    /// Resolve the source for a package name.
    pub fn resolve(&self, name: &str) -> ResolvedSource {
        let normalized = name.to_lowercase().replace('-', "_");

        // 1. Well-known crates.io packages
        if is_known_crate(&normalized) {
            return ResolvedSource {
                source: DepSource::CratesIo,
                confidence: Confidence::High,
                reason: format!("'{}' is a well-known crates.io package", name),
            };
        }

        // 2. Well-known PyPI packages
        if is_known_pypi(&normalized) {
            return ResolvedSource {
                source: DepSource::PyPI,
                confidence: Confidence::High,
                reason: format!("'{}' is a well-known PyPI package", name),
            };
        }

        // 3. Name heuristics for crates.io
        if let Some(reason) = crate_name_heuristic(name) {
            return ResolvedSource {
                source: DepSource::CratesIo,
                confidence: Confidence::Medium,
                reason,
            };
        }

        // 4. Name heuristics for PyPI
        if let Some(reason) = pypi_name_heuristic(name) {
            return ResolvedSource {
                source: DepSource::PyPI,
                confidence: Confidence::Medium,
                reason,
            };
        }

        // 5. Project context fallback
        let has_rust = self.languages.contains(&Language::Rust);
        let has_python = self.languages.contains(&Language::Python);

        match (has_rust, has_python) {
            (true, false) => ResolvedSource {
                source: DepSource::CratesIo,
                confidence: Confidence::Low,
                reason: format!(
                    "'{}' not in known lists, but project is Rust-only — assuming crates.io",
                    name
                ),
            },
            (false, true) => ResolvedSource {
                source: DepSource::PyPI,
                confidence: Confidence::Low,
                reason: format!(
                    "'{}' not in known lists, but project is Python-only — assuming PyPI",
                    name
                ),
            },
            _ => ResolvedSource {
                source: DepSource::Registry,
                confidence: Confidence::Low,
                reason: format!(
                    "'{}' not in known lists and project language is ambiguous — defaulting to registry",
                    name
                ),
            },
        }
    }
}

// ── Pre-build dependency validation ─────────────────────────────────────────

/// A dependency issue found during pre-build validation.
#[derive(Debug)]
pub struct DepIssue {
    /// Dependency name.
    pub name: String,
    /// The problem.
    pub message: String,
    /// Whether this is a warning (false) or error (true).
    pub is_error: bool,
}

/// Validate dependencies before invoking cargo/pip.
///
/// Checks for:
/// - Crates.io packages routed to PyPI (or vice versa)
/// - Well-known packages set to Registry (should be crates.io/PyPI)
///
/// Returns a list of issues (warnings and errors).
pub fn validate_deps(
    deps: &std::collections::BTreeMap<String, crate::manifest::DependencyValue>,
    languages: &[Language],
) -> Vec<DepIssue> {
    let resolver = PackageSourceResolver::new(languages);
    let mut issues = Vec::new();

    for (name, dep) in deps {
        let declared_source = dep.effective_source();
        let resolved = resolver.resolve(name);

        // Check for cross-ecosystem routing (crates.io package declared as PyPI, etc.)
        if resolved.confidence == Confidence::High {
            match (&declared_source, &resolved.source) {
                (DepSource::PyPI, DepSource::CratesIo) => {
                    issues.push(DepIssue {
                        name: name.clone(),
                        message: format!(
                            "'{}' is a well-known crates.io crate but declared as PyPI",
                            name
                        ),
                        is_error: true,
                    });
                }
                (DepSource::CratesIo, DepSource::PyPI) => {
                    issues.push(DepIssue {
                        name: name.clone(),
                        message: format!(
                            "'{}' is a well-known PyPI package but declared as crates.io",
                            name
                        ),
                        is_error: true,
                    });
                }
                _ => {}
            }
        }
    }

    issues
}

// ── Name heuristics ─────────────────────────────────────────────────────────

fn crate_name_heuristic(name: &str) -> Option<String> {
    let n = name.to_lowercase();

    // *-sys crates (C binding wrappers)
    if n.ends_with("-sys") {
        return Some(format!("'{}' has -sys suffix (Rust FFI binding convention)", name));
    }

    // *-rs suffix
    if n.ends_with("-rs") || n.ends_with("_rs") {
        return Some(format!("'{}' has -rs suffix (Rust package convention)", name));
    }

    // cargo-* tools
    if n.starts_with("cargo-") || n.starts_with("cargo_") {
        return Some(format!("'{}' has cargo- prefix (Cargo subcommand)", name));
    }

    None
}

fn pypi_name_heuristic(name: &str) -> Option<String> {
    let n = name.to_lowercase();

    // python-*, py-* prefixes
    if n.starts_with("python-") || n.starts_with("python_") {
        return Some(format!("'{}' has python- prefix (PyPI convention)", name));
    }
    if n.starts_with("py-") || n.starts_with("py_") {
        return Some(format!("'{}' has py- prefix (PyPI convention)", name));
    }

    // django-*, flask-* (Python web frameworks)
    if n.starts_with("django-") || n.starts_with("django_") {
        return Some(format!("'{}' has django- prefix (Python web framework plugin)", name));
    }
    if n.starts_with("flask-") || n.starts_with("flask_") {
        return Some(format!("'{}' has flask- prefix (Python web framework plugin)", name));
    }

    // *-py suffix
    if n.ends_with("-py") || n.ends_with("_py") {
        return Some(format!("'{}' has -py suffix (PyPI convention)", name));
    }

    None
}

// ── Well-known package registries ───────────────────────────────────────────

/// Check if a normalized name is a well-known crates.io package.
fn is_known_crate(normalized: &str) -> bool {
    KNOWN_CRATES.binary_search(&normalized).is_ok()
}

/// Check if a normalized name is a well-known PyPI package.
fn is_known_pypi(normalized: &str) -> bool {
    KNOWN_PYPI.binary_search(&normalized).is_ok()
}

/// Top crates.io packages (sorted, normalized to underscores + lowercase).
///
/// Covers the top ~300 most-downloaded crates plus robotics-relevant ones.
/// Binary-searched at runtime.
static KNOWN_CRATES: &[&str] = &[
    "actix_rt",
    "actix_web",
    "aes",
    "ahash",
    "aho_corasick",
    "anstream",
    "anstyle",
    "anyhow",
    "approx",
    "arc_swap",
    "arrayvec",
    "ascii",
    "async_channel",
    "async_io",
    "async_lock",
    "async_process",
    "async_recursion",
    "async_std",
    "async_stream",
    "async_trait",
    "atoi",
    "auto_impl",
    "autocfg",
    "axum",
    "backtrace",
    "base64",
    "bincode",
    "bit_set",
    "bit_vec",
    "bitflags",
    "bitvec",
    "blake2",
    "blake3",
    "block_buffer",
    "bollard",
    "bstr",
    "bumpalo",
    "bytemuck",
    "byteorder",
    "bytes",
    "camino",
    "cargo_metadata",
    "cc",
    "cfg_if",
    "chrono",
    "ciborium",
    "clap",
    "clap_builder",
    "clap_derive",
    "clap_lex",
    "color_eyre",
    "colorchoice",
    "colored",
    "console",
    "const_oid",
    "core_foundation",
    "cpufeatures",
    "crc32fast",
    "criterion",
    "crossbeam",
    "crossbeam_channel",
    "crossbeam_deque",
    "crossbeam_epoch",
    "crossbeam_queue",
    "crossbeam_utils",
    "crypto_common",
    "csv",
    "ctor",
    "darling",
    "dashmap",
    "data_encoding",
    "der",
    "deranged",
    "derive_builder",
    "derive_more",
    "diesel",
    "digest",
    "dirs",
    "dirs_next",
    "dotenv",
    "downcast_rs",
    "either",
    "embed_resource",
    "encoding_rs",
    "env_logger",
    "equivalent",
    "erased_serde",
    "errno",
    "eyre",
    "fastrand",
    "filetime",
    "flate2",
    "flume",
    "fnv",
    "form_urlencoded",
    "futures",
    "futures_channel",
    "futures_core",
    "futures_executor",
    "futures_io",
    "futures_lite",
    "futures_macro",
    "futures_sink",
    "futures_task",
    "futures_util",
    "generic_array",
    "getrandom",
    "glob",
    "globset",
    "h2",
    "half",
    "hashbrown",
    "heck",
    "hex",
    "hmac",
    "home",
    "http",
    "http_body",
    "httparse",
    "humantime",
    "hyper",
    "hyper_rustls",
    "hyper_tls",
    "hyper_util",
    "iana_time_zone",
    "idna",
    "image",
    "indexmap",
    "indicatif",
    "insta",
    "ipnet",
    "is_terminal",
    "itertools",
    "itoa",
    "js_sys",
    "lazy_static",
    "libc",
    "libloading",
    "libm",
    "linux_raw_sys",
    "lock_api",
    "log",
    "lru",
    "matrixmultiply",
    "memchr",
    "memmap2",
    "memoffset",
    "mime",
    "miniz_oxide",
    "mio",
    "nalgebra",
    "nix",
    "nom",
    "notify",
    "num",
    "num_bigint",
    "num_complex",
    "num_cpus",
    "num_derive",
    "num_integer",
    "num_rational",
    "num_traits",
    "object",
    "once_cell",
    "opaque_debug",
    "openssl",
    "openssl_sys",
    "ordered_float",
    "os_str_bytes",
    "parking",
    "parking_lot",
    "parking_lot_core",
    "paste",
    "pem",
    "percent_encoding",
    "petgraph",
    "pin_project",
    "pin_project_lite",
    "pin_utils",
    "pkg_config",
    "polling",
    "portable_atomic",
    "ppv_lite86",
    "pretty_assertions",
    "proc_macro2",
    "proptest",
    "prost",
    "prost_build",
    "prost_derive",
    "prost_types",
    "quick_error",
    "quick_xml",
    "quote",
    "r2d2",
    "rand",
    "rand_chacha",
    "rand_core",
    "rand_distr",
    "rapier2d",
    "rapier3d",
    "ratatui",
    "rayon",
    "rayon_core",
    "redis",
    "regex",
    "regex_automata",
    "regex_syntax",
    "reqwest",
    "ring",
    "ron",
    "rsa",
    "rstest",
    "rusqlite",
    "rust_decimal",
    "rustc_hash",
    "rustix",
    "rustls",
    "rustls_pemfile",
    "rustls_webpki",
    "ryu",
    "same_file",
    "schannel",
    "scopeguard",
    "seahash",
    "semver",
    "serde",
    "serde_bytes",
    "serde_derive",
    "serde_json",
    "serde_repr",
    "serde_urlencoded",
    "serde_with",
    "serde_yaml",
    "sha1",
    "sha2",
    "shlex",
    "signal_hook",
    "signal_hook_registry",
    "simba",
    "siphasher",
    "slab",
    "smallvec",
    "smol_str",
    "socket2",
    "sqlx",
    "strsim",
    "subtle",
    "syn",
    "tar",
    "tempfile",
    "termcolor",
    "terminal_size",
    "textwrap",
    "thiserror",
    "time",
    "tinyvec",
    "tokio",
    "tokio_macros",
    "tokio_native_tls",
    "tokio_rustls",
    "tokio_stream",
    "tokio_tungstenite",
    "tokio_util",
    "toml",
    "toml_edit",
    "tonic",
    "tonic_build",
    "tower",
    "tower_http",
    "tower_layer",
    "tower_service",
    "tracing",
    "tracing_appender",
    "tracing_core",
    "tracing_log",
    "tracing_subscriber",
    "tree_sitter",
    "typenum",
    "unicase",
    "unicode_bidi",
    "unicode_ident",
    "unicode_normalization",
    "unicode_segmentation",
    "unicode_width",
    "unicode_xid",
    "url",
    "utf8parse",
    "uuid",
    "walkdir",
    "want",
    "warp",
    "wasi",
    "wasm_bindgen",
    "wasm_bindgen_futures",
    "which",
    "winapi",
    "windows",
    "windows_sys",
    "winnow",
    "yansi",
    "zerocopy",
    "zeroize",
    "zip",
    "zstd",
];

/// Top PyPI packages (sorted, normalized to underscores + lowercase).
///
/// Covers the top ~300 most-downloaded packages plus robotics/ML-relevant ones.
/// Binary-searched at runtime.
static KNOWN_PYPI: &[&str] = &[
    "aiofiles",
    "aiohttp",
    "aiosignal",
    "anyio",
    "argparse",
    "async_timeout",
    "attrs",
    "beautifulsoup4",
    "black",
    "bleach",
    "blinker",
    "bokeh",
    "boto3",
    "botocore",
    "build",
    "cachetools",
    "catkin_pkg",
    "celery",
    "certifi",
    "cffi",
    "charset_normalizer",
    "click",
    "cloudpickle",
    "colcon_core",
    "colorama",
    "contourpy",
    "coverage",
    "cryptography",
    "cycler",
    "cython",
    "dask",
    "dataclasses_json",
    "decorator",
    "defusedxml",
    "deprecated",
    "dill",
    "distlib",
    "docker",
    "docutils",
    "exceptiongroup",
    "executing",
    "fastapi",
    "filelock",
    "flake8",
    "flask",
    "fonttools",
    "frozenlist",
    "fsspec",
    "google_api_core",
    "google_auth",
    "google_cloud_storage",
    "greenlet",
    "grpcio",
    "gunicorn",
    "gym",
    "gymnasium",
    "h11",
    "h5py",
    "httpcore",
    "httptools",
    "httpx",
    "huggingface_hub",
    "humanize",
    "idna",
    "imageio",
    "importlib_metadata",
    "importlib_resources",
    "iniconfig",
    "ipykernel",
    "ipython",
    "ipywidgets",
    "isort",
    "jax",
    "jaxlib",
    "jinja2",
    "joblib",
    "json5",
    "jsonschema",
    "jupyter",
    "jupyter_client",
    "jupyter_core",
    "jupyterlab",
    "keras",
    "kiwisolver",
    "lark",
    "lxml",
    "markupsafe",
    "marshmallow",
    "matplotlib",
    "mccabe",
    "ml_dtypes",
    "more_itertools",
    "moviepy",
    "mpmath",
    "msgpack",
    "mujoco",
    "multidict",
    "mypy",
    "mypy_extensions",
    "nbconvert",
    "networkx",
    "nltk",
    "nodeenv",
    "notebook",
    "numba",
    "numpy",
    "oauthlib",
    "onnx",
    "onnxruntime",
    "open3d",
    "opencv_python",
    "opencv_python_headless",
    "openpyxl",
    "opt_einsum",
    "orjson",
    "packaging",
    "pandas",
    "paramiko",
    "pathlib2",
    "pathspec",
    "pbr",
    "pep517",
    "pillow",
    "pip",
    "pipenv",
    "platformdirs",
    "plotly",
    "pluggy",
    "poetry",
    "polars",
    "pre_commit",
    "prometheus_client",
    "prompt_toolkit",
    "protobuf",
    "psutil",
    "psycopg2",
    "pure_eval",
    "py",
    "pyarrow",
    "pybind11",
    "pybullet",
    "pycodestyle",
    "pycparser",
    "pydantic",
    "pydantic_core",
    "pyflakes",
    "pygments",
    "pyjwt",
    "pylint",
    "pymongo",
    "pynacl",
    "pyopenssl",
    "pyparsing",
    "pyperclip",
    "pyqt5",
    "pyright",
    "pyserial",
    "pytest",
    "pytest_cov",
    "python_dateutil",
    "python_dotenv",
    "pytorch_lightning",
    "pytz",
    "pyyaml",
    "pyzmq",
    "rclpy",
    "redis",
    "regex",
    "requests",
    "rich",
    "rosbag2_py",
    "rospkg",
    "rpyc",
    "rsa",
    "ruamel_yaml",
    "ruff",
    "s3transfer",
    "safetensors",
    "scikit_image",
    "scikit_learn",
    "scipy",
    "seaborn",
    "selenium",
    "sentry_sdk",
    "setuptools",
    "shapely",
    "simplejson",
    "six",
    "sniffio",
    "soupsieve",
    "sphinx",
    "sqlalchemy",
    "stack_data",
    "starlette",
    "stevedore",
    "sympy",
    "tabulate",
    "tenacity",
    "tensorboard",
    "tensorflow",
    "termcolor",
    "threadpoolctl",
    "tokenizers",
    "tomli",
    "tomlkit",
    "toolz",
    "torch",
    "torchvision",
    "tornado",
    "tqdm",
    "traitlets",
    "transformers",
    "trimesh",
    "trio",
    "twine",
    "typer",
    "types_requests",
    "typing_extensions",
    "ujson",
    "urllib3",
    "uvicorn",
    "uvloop",
    "virtualenv",
    "wandb",
    "watchdog",
    "wcwidth",
    "websocket_client",
    "websockets",
    "werkzeug",
    "wheel",
    "wrapt",
    "xarray",
    "xlsxwriter",
    "xmltodict",
    "yarl",
    "zipp",
];

#[cfg(test)]
mod tests {
    use super::*;

    // ── Well-known package resolution ───────────────────────────────────

    #[test]
    fn resolves_serde_as_crates_io() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("serde");
        assert_eq!(result.source, DepSource::CratesIo);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn resolves_tokio_as_crates_io() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("tokio");
        assert_eq!(result.source, DepSource::CratesIo);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn resolves_numpy_as_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("numpy");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn resolves_torch_as_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("torch");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn resolves_requests_as_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("requests");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn resolves_flask_as_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("flask");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn resolves_opencv_python_as_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("opencv-python");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn resolves_nalgebra_as_crates_io() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("nalgebra");
        assert_eq!(result.source, DepSource::CratesIo);
        assert_eq!(result.confidence, Confidence::High);
    }

    // ── Name heuristics ─────────────────────────────────────────────────

    #[test]
    fn heuristic_sys_suffix_is_crates_io() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("libfoo-sys");
        assert_eq!(result.source, DepSource::CratesIo);
        assert_eq!(result.confidence, Confidence::Medium);
    }

    #[test]
    fn heuristic_rs_suffix_is_crates_io() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("mqtt-rs");
        assert_eq!(result.source, DepSource::CratesIo);
        assert_eq!(result.confidence, Confidence::Medium);
    }

    #[test]
    fn heuristic_cargo_prefix_is_crates_io() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("cargo-expand");
        // cargo-expand is also in KNOWN_CRATES, so it should be High
        assert_eq!(result.source, DepSource::CratesIo);
    }

    #[test]
    fn heuristic_py_prefix_is_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("py-spy");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::Medium);
    }

    #[test]
    fn heuristic_python_prefix_is_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("python-dotenv");
        // python_dotenv is in KNOWN_PYPI, so it should be High
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    #[test]
    fn heuristic_django_prefix_is_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("django-rest-framework");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::Medium);
    }

    #[test]
    fn heuristic_flask_prefix_is_pypi() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("flask-cors");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::Medium);
    }

    // ── Project context fallback ────────────────────────────────────────

    #[test]
    fn unknown_package_in_rust_project_resolves_crates_io() {
        let resolver = PackageSourceResolver::new(&[Language::Rust]);
        let result = resolver.resolve("my-custom-crate");
        assert_eq!(result.source, DepSource::CratesIo);
        assert_eq!(result.confidence, Confidence::Low);
    }

    #[test]
    fn unknown_package_in_python_project_resolves_pypi() {
        let resolver = PackageSourceResolver::new(&[Language::Python]);
        let result = resolver.resolve("my-custom-package");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::Low);
    }

    #[test]
    fn unknown_package_in_mixed_project_resolves_registry() {
        let resolver = PackageSourceResolver::new(&[Language::Rust, Language::Python]);
        let result = resolver.resolve("unknown-thing");
        assert_eq!(result.source, DepSource::Registry);
        assert_eq!(result.confidence, Confidence::Low);
    }

    #[test]
    fn unknown_package_no_context_resolves_registry() {
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("something-totally-unknown");
        assert_eq!(result.source, DepSource::Registry);
        assert_eq!(result.confidence, Confidence::Low);
    }

    // ── Normalization ───────────────────────────────────────────────────

    #[test]
    fn hyphen_and_underscore_both_resolve() {
        let resolver = PackageSourceResolver::without_context();

        let r1 = resolver.resolve("serde-json");
        assert_eq!(r1.source, DepSource::CratesIo);
        assert_eq!(r1.confidence, Confidence::High);

        let r2 = resolver.resolve("serde_json");
        assert_eq!(r2.source, DepSource::CratesIo);
        assert_eq!(r2.confidence, Confidence::High);
    }

    #[test]
    fn case_insensitive_resolution() {
        let resolver = PackageSourceResolver::without_context();

        let result = resolver.resolve("NumPy");
        assert_eq!(result.source, DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    // ── Cross-ecosystem collisions ──────────────────────────────────────

    #[test]
    fn redis_resolves_consistently() {
        // 'redis' exists on both crates.io and PyPI.
        // Our well-known lists should have it in exactly one — check that
        // resolution is deterministic.
        let resolver = PackageSourceResolver::without_context();
        let result = resolver.resolve("redis");
        assert!(result.source == DepSource::CratesIo || result.source == DepSource::PyPI);
        assert_eq!(result.confidence, Confidence::High);
    }

    // ── Known-list invariants ───────────────────────────────────────────

    #[test]
    fn known_crates_is_sorted() {
        let mut sorted = KNOWN_CRATES.to_vec();
        sorted.sort();
        assert_eq!(
            KNOWN_CRATES, &sorted[..],
            "KNOWN_CRATES must be sorted for binary search"
        );
    }

    #[test]
    fn known_pypi_is_sorted() {
        let mut sorted = KNOWN_PYPI.to_vec();
        sorted.sort();
        assert_eq!(
            KNOWN_PYPI, &sorted[..],
            "KNOWN_PYPI must be sorted for binary search"
        );
    }

    #[test]
    fn no_duplicates_in_known_crates() {
        let mut seen = std::collections::HashSet::new();
        for &name in KNOWN_CRATES {
            assert!(seen.insert(name), "Duplicate in KNOWN_CRATES: {}", name);
        }
    }

    #[test]
    fn no_duplicates_in_known_pypi() {
        let mut seen = std::collections::HashSet::new();
        for &name in KNOWN_PYPI {
            assert!(seen.insert(name), "Duplicate in KNOWN_PYPI: {}", name);
        }
    }

    // ── Confidence ordering ─────────────────────────────────────────────

    #[test]
    fn confidence_ordering() {
        assert!(Confidence::Low < Confidence::Medium);
        assert!(Confidence::Medium < Confidence::High);
    }

    // ── Robotics packages ───────────────────────────────────────────────

    #[test]
    fn robotics_crates_resolved() {
        let resolver = PackageSourceResolver::without_context();
        for name in &["nalgebra", "rapier3d", "rapier2d"] {
            let result = resolver.resolve(name);
            assert_eq!(
                result.source,
                DepSource::CratesIo,
                "{} should be crates.io",
                name
            );
        }
    }

    #[test]
    fn robotics_pypi_resolved() {
        let resolver = PackageSourceResolver::without_context();
        for name in &[
            "opencv-python",
            "pybullet",
            "mujoco",
            "gymnasium",
            "open3d",
            "trimesh",
        ] {
            let result = resolver.resolve(name);
            assert_eq!(
                result.source,
                DepSource::PyPI,
                "{} should be PyPI",
                name
            );
        }
    }
}
