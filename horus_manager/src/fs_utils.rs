//! Shared filesystem utilities — dir_size, count_files, format_bytes.
//!
//! Consolidates duplicate implementations from doctor.rs, clean.rs, and cache.rs.

use std::path::Path;

/// Calculate total size of a directory recursively (returns 0 on error).
pub fn dir_size(path: &Path) -> u64 {
    walkdir::WalkDir::new(path)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_file())
        .filter_map(|e| e.metadata().ok())
        .map(|m| m.len())
        .sum()
}

/// Count files in a directory recursively (returns 0 on error).
pub fn count_files(path: &Path) -> usize {
    walkdir::WalkDir::new(path)
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| e.file_type().is_file())
        .count()
}

/// Re-export format_bytes from progress module.
pub use crate::progress::format_bytes;

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn dir_size_empty() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(dir_size(tmp.path()), 0);
    }

    #[test]
    fn dir_size_single_file() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("data.bin"), vec![0u8; 256]).unwrap();
        assert_eq!(dir_size(tmp.path()), 256);
    }

    #[test]
    fn dir_size_nested() {
        let tmp = tempfile::tempdir().unwrap();
        let sub = tmp.path().join("a").join("b");
        fs::create_dir_all(&sub).unwrap();
        fs::write(sub.join("file.dat"), vec![1u8; 100]).unwrap();
        fs::write(tmp.path().join("root.dat"), vec![2u8; 50]).unwrap();
        assert_eq!(dir_size(tmp.path()), 150);
    }

    #[test]
    fn dir_size_nonexistent() {
        assert_eq!(dir_size(Path::new("/nonexistent/path/99999")), 0);
    }

    #[test]
    fn count_files_empty() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(count_files(tmp.path()), 0);
    }

    #[test]
    fn count_files_with_files() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("a.txt"), "hello").unwrap();
        fs::write(tmp.path().join("b.txt"), "world").unwrap();
        assert_eq!(count_files(tmp.path()), 2);
    }

    #[test]
    fn count_files_nested() {
        let tmp = tempfile::tempdir().unwrap();
        let sub = tmp.path().join("sub");
        fs::create_dir(&sub).unwrap();
        fs::write(sub.join("file.bin"), "data").unwrap();
        fs::write(tmp.path().join("root.txt"), "data").unwrap();
        assert_eq!(count_files(tmp.path()), 2);
    }

    #[test]
    fn count_files_nonexistent() {
        assert_eq!(count_files(Path::new("/nonexistent/path/99999")), 0);
    }
}
