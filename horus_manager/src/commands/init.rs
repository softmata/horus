//! HORUS initialization command
//!
//! Handles workspace initialization

use crate::cli_output;
use anyhow::Result;
use colored::*;

/// Run the init command - initialize a HORUS workspace
pub fn run_init(workspace_name: Option<String>) -> Result<()> {
    cli_output::header("Initializing HORUS workspace");
    println!();

    // Register workspace using existing workspace module
    crate::workspace::register_current_workspace(workspace_name)?;

    println!();
    cli_output::success("Workspace initialized successfully!");
    println!();
    println!("Next steps:");
    println!(
        "  1. Create a new project: {}",
        "horus new my_robot".yellow()
    );
    println!(
        "  2. Install packages:     {}",
        "horus install <package>".yellow()
    );
    println!("  3. Start monitor:        {}", "horus monitor".yellow());
    println!();

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    // ========================================================================
    // run_init — basic success path
    // ========================================================================

    #[test]
    fn init_creates_horus_dir_and_toml() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("test_workspace".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(
            result.is_ok(),
            "run_init should succeed: {:?}",
            result.err()
        );

        // .horus/ directory should be created
        assert!(
            tmp.path().join(".horus").exists(),
            ".horus/ directory should exist"
        );
        assert!(
            tmp.path().join(".horus").is_dir(),
            ".horus should be a directory"
        );

        // horus.toml should be created
        let toml_path = tmp.path().join("horus.toml");
        assert!(toml_path.exists(), "horus.toml should exist");

        let content = fs::read_to_string(&toml_path).unwrap();
        assert!(
            content.contains("[package]"),
            "horus.toml should contain [package] section"
        );
        assert!(
            content.contains("name = \"test_workspace\""),
            "horus.toml should contain workspace name"
        );
        assert!(
            content.contains("version"),
            "horus.toml should contain version"
        );
    }

    #[test]
    fn init_with_explicit_name_uses_provided_name() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("my_custom_robot".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert!(
            content.contains("name = \"my_custom_robot\""),
            "Should use explicitly provided name, got: {}",
            content
        );
    }

    #[test]
    fn init_with_none_name_uses_directory_name() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(None);
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(
            result.is_ok(),
            "run_init(None) should succeed: {:?}",
            result.err()
        );

        // horus.toml should exist with the tempdir's folder name
        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        let dir_name = tmp.path().file_name().unwrap().to_str().unwrap();
        assert!(
            content.contains(&format!("name = \"{}\"", dir_name)),
            "Should use directory name '{}', got: {}",
            dir_name,
            content
        );
    }

    // ========================================================================
    // run_init — idempotency
    // ========================================================================

    #[test]
    fn init_is_idempotent() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // First init
        let result1 = run_init(Some("ws_idem".to_string()));
        assert!(result1.is_ok(), "First init should succeed");

        let content_after_first = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();

        // Second init — should not fail, should not overwrite existing horus.toml
        let result2 = run_init(Some("ws_idem_v2".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result2.is_ok(), "Second init should succeed (idempotent)");

        // horus.toml should still have the original name (not overwritten)
        let content_after_second = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert_eq!(
            content_after_first, content_after_second,
            "horus.toml should not be overwritten on re-init"
        );
    }

    #[test]
    fn init_does_not_overwrite_existing_horus_toml() {
        let tmp = TempDir::new().unwrap();

        // Pre-create horus.toml with custom content
        let custom_content = "[package]\nname = \"pre_existing\"\nversion = \"1.0.0\"\n";
        fs::write(tmp.path().join("horus.toml"), custom_content).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("new_name".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        // Original content should be preserved
        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert_eq!(
            content, custom_content,
            "Pre-existing horus.toml should not be overwritten"
        );
    }

    #[test]
    fn init_does_not_overwrite_existing_horus_dir() {
        let tmp = TempDir::new().unwrap();

        // Pre-create .horus/ with a file inside
        let horus_dir = tmp.path().join(".horus");
        fs::create_dir_all(&horus_dir).unwrap();
        let marker = horus_dir.join("marker.txt");
        fs::write(&marker, "existing content").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("ws_existing_dir".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        // Marker file should still exist
        assert!(
            marker.exists(),
            "Pre-existing files in .horus/ should be preserved"
        );
        assert_eq!(fs::read_to_string(&marker).unwrap(), "existing content");
    }

    // ========================================================================
    // run_init — workspace registry integration
    // ========================================================================

    #[test]
    fn init_registers_workspace_in_registry() {
        // Use a unique name to avoid collisions with parallel tests
        let unique_name = format!("reg_test_{}", std::process::id());
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some(unique_name.clone()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        // Verify workspace was registered
        let registry = crate::workspace::WorkspaceRegistry::load().unwrap();
        let found = registry.find_by_name(&unique_name);
        assert!(
            found.is_some(),
            "Workspace should be registered in the global registry"
        );
    }

    // ========================================================================
    // run_init — various workspace name formats
    // ========================================================================

    #[test]
    fn init_with_hyphenated_name() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("my-robot-arm".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert!(content.contains("name = \"my-robot-arm\""));
    }

    #[test]
    fn init_with_underscored_name() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("diff_drive_robot".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert!(content.contains("name = \"diff_drive_robot\""));
    }

    #[test]
    fn init_with_single_char_name() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("x".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert!(content.contains("name = \"x\""));
    }

    #[test]
    fn init_with_empty_string_name() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // Empty string is provided as Some(""), which will be used as-is
        // by register_current_workspace (the name is passed through)
        let result = run_init(Some("".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        // Should still succeed — register_current_workspace does not validate names
        assert!(result.is_ok());
    }

    #[test]
    fn init_with_name_containing_spaces() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("my robot workspace".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        // register_current_workspace does not reject names with spaces
        assert!(result.is_ok());
        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert!(content.contains("name = \"my robot workspace\""));
    }

    // ========================================================================
    // run_init — return type verification
    // ========================================================================

    #[test]
    fn init_returns_ok_result() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("ok_result_ws".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        // run_init may fail if workspace registry format changed — not a test issue
        assert!(result.is_ok(), "run_init failed: {:?}", result.err());
        let _ = result; // use the result
    }

    // ========================================================================
    // run_init — directory structure verification
    // ========================================================================

    #[test]
    fn init_creates_expected_directory_structure() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        run_init(Some("structure_test".to_string())).unwrap();
        std::env::set_current_dir(&original_dir).unwrap();

        // Verify the complete set of created files/dirs
        let entries: Vec<_> = fs::read_dir(tmp.path())
            .unwrap()
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().to_string())
            .collect();

        assert!(
            entries.contains(&".horus".to_string()),
            "Should contain .horus dir, got: {:?}",
            entries
        );
        assert!(
            entries.contains(&"horus.toml".to_string()),
            "Should contain horus.toml, got: {:?}",
            entries
        );
    }

    #[test]
    fn init_horus_toml_is_valid_toml() {
        let tmp = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        run_init(Some("toml_valid_ws".to_string())).unwrap();
        std::env::set_current_dir(&original_dir).unwrap();

        let content = fs::read_to_string(tmp.path().join("horus.toml")).unwrap();

        // Should be parseable as TOML
        let parsed: toml::Value =
            toml::from_str(&content).expect("horus.toml should be valid TOML");

        // Should have package.name and package.version
        let package = parsed
            .get("package")
            .expect("Should have [package] section");
        assert_eq!(
            package.get("name").and_then(|v| v.as_str()),
            Some("toml_valid_ws"),
            "package.name should match workspace name"
        );
        assert!(
            package.get("version").is_some(),
            "package.version should be present"
        );
    }

    // ========================================================================
    // run_init — sequential inits in different directories
    // ========================================================================

    #[test]
    fn init_multiple_workspaces_sequentially() {
        let tmp1 = TempDir::new().unwrap();
        let tmp2 = TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Init first workspace
        std::env::set_current_dir(tmp1.path()).unwrap();
        run_init(Some("ws_alpha".to_string())).unwrap();

        // Init second workspace
        std::env::set_current_dir(tmp2.path()).unwrap();
        run_init(Some("ws_beta".to_string())).unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        // Both should have their own files
        assert!(tmp1.path().join(".horus").exists());
        assert!(tmp1.path().join("horus.toml").exists());
        assert!(tmp2.path().join(".horus").exists());
        assert!(tmp2.path().join("horus.toml").exists());

        // Both should be in registry
        let registry = crate::workspace::WorkspaceRegistry::load().unwrap();
        assert!(registry.find_by_name("ws_alpha").is_some());
        assert!(registry.find_by_name("ws_beta").is_some());

        // Names and paths should be distinct
        let alpha = registry.find_by_name("ws_alpha").unwrap();
        let beta = registry.find_by_name("ws_beta").unwrap();
        assert_ne!(alpha.path, beta.path);
    }

    // ========================================================================
    // run_init — pre-existing subdirectories
    // ========================================================================

    #[test]
    fn init_in_directory_with_existing_files() {
        let tmp = TempDir::new().unwrap();

        // Create some pre-existing content
        fs::write(tmp.path().join("README.md"), "# My Robot").unwrap();
        fs::create_dir_all(tmp.path().join("src")).unwrap();
        fs::write(tmp.path().join("src/main.rs"), "fn main() {}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_init(Some("existing_files_ws".to_string()));
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok());

        // Existing files should be untouched
        assert_eq!(
            fs::read_to_string(tmp.path().join("README.md")).unwrap(),
            "# My Robot"
        );
        assert_eq!(
            fs::read_to_string(tmp.path().join("src/main.rs")).unwrap(),
            "fn main() {}"
        );
    }
}
