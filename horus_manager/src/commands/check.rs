//! Check command - validate horus.yaml, source files, or entire workspace

use crate::cli_output;
use crate::commands::run::{check_hardware_requirements, parse_horus_yaml_dependencies_v2};
use crate::config::{CARGO_TOML, HORUS_YAML};
use crate::dependency_resolver::DependencySource;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use horus_core::memory::has_native_shm;
#[cfg(not(target_os = "linux"))]
use horus_core::memory::shm_base_dir;
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

/// Run the check command on a path (directory or file)
pub fn run_check(path: Option<PathBuf>, quiet: bool, json: bool) -> HorusResult<()> {
    let target_path = path.unwrap_or_else(|| PathBuf::from("."));
    log::debug!("checking path: {:?}", target_path);

    if !target_path.exists() {
        if json {
            let output = serde_json::json!({
                "path": target_path.display().to_string(),
                "valid": false,
                "error": "Path not found",
            });
            println!("{}", serde_json::to_string_pretty(&output).unwrap());
            return Ok(());
        }
        println!(
            "{} Path not found: {}",
            cli_output::ICON_ERROR.red(),
            target_path.display()
        );
        return Err(HorusError::Config("Path not found".to_string()));
    }

    // For JSON mode, capture check result and output as JSON only
    if json {
        // Run the check and only care about its Ok/Err result for JSON output
        let result = if target_path.is_dir() {
            check_workspace(&target_path, true)
        } else {
            check_single_file(&target_path, true)
        };
        let valid = result.is_ok();
        let error_msg = result.as_ref().err().map(|e| e.to_string());
        // Use eprintln-free JSON output
        let output = serde_json::json!({
            "path": target_path.display().to_string(),
            "valid": valid,
            "error": error_msg,
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
        return if valid { Ok(()) } else { result };
    }

    if target_path.is_dir() {
        check_workspace(&target_path, quiet)
    } else {
        check_single_file(&target_path, quiet)
    }
}

/// Scan an entire workspace directory
fn check_workspace(target_path: &Path, quiet: bool) -> HorusResult<()> {
    println!(
        "{} Scanning workspace: {}\n",
        cli_output::ICON_INFO.cyan().bold(),
        target_path
            .canonicalize()
            .unwrap_or(target_path.to_path_buf())
            .display()
    );

    let mut total_errors = 0;
    let mut total_warnings = 0;
    let mut files_checked = 0;
    let mut horus_yamls: Vec<PathBuf> = Vec::new();
    let mut rust_files: Vec<PathBuf> = Vec::new();
    let mut python_files: Vec<PathBuf> = Vec::new();

    // Collect all files to check
    for entry in WalkDir::new(target_path)
        .into_iter()
        .filter_entry(|e| {
            let name = e.file_name().to_string_lossy();
            !name.starts_with('.')
                && name != "target"
                && name != "node_modules"
                && name != "__pycache__"
                && name != ".horus"
        })
        .filter_map(|e| e.ok())
    {
        let path = entry.path();
        if path.is_file() {
            let filename = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
            let ext = path.extension().and_then(|e| e.to_str());

            if filename == HORUS_YAML {
                horus_yamls.push(path.to_path_buf());
            } else if ext == Some("rs") && filename != "build.rs" {
                rust_files.push(path.to_path_buf());
            } else if ext == Some("py") {
                python_files.push(path.to_path_buf());
            }
        }
    }

    println!("  Found {} horus.yaml file(s)", horus_yamls.len());
    println!("  Found {} Rust file(s)", rust_files.len());
    println!("  Found {} Python file(s)\n", python_files.len());

    // Find Cargo.toml directories for deep Rust checking
    let mut cargo_dirs: HashSet<PathBuf> = HashSet::new();
    for entry in WalkDir::new(target_path)
        .into_iter()
        .filter_entry(|e| {
            let name = e.file_name().to_string_lossy();
            !name.starts_with('.') && name != "target" && name != "node_modules"
        })
        .filter_map(|e| e.ok())
    {
        if entry.file_name() == CARGO_TOML {
            if let Some(parent) = entry.path().parent() {
                cargo_dirs.insert(parent.to_path_buf());
            }
        }
    }

    // ═══════════════════════════════════════════════════════════
    // PHASE 1: Validate horus.yaml manifests
    // ═══════════════════════════════════════════════════════════
    if !horus_yamls.is_empty() {
        println!("{}", "━".repeat(60).dimmed());
        println!(
            "{} Phase 1: Validating horus.yaml manifests...\n",
            cli_output::ICON_INFO.cyan().bold()
        );

        for yaml_path in &horus_yamls {
            let rel_path = yaml_path.strip_prefix(target_path).unwrap_or(yaml_path);
            println!("  {} {}", "▸".cyan(), rel_path.display());

            match fs::read_to_string(yaml_path) {
                Ok(content) => {
                    match serde_yaml::from_str::<serde_yaml::Value>(&content) {
                        Ok(yaml) => {
                            let mut file_errors: Vec<String> = Vec::new();
                            let base_dir = yaml_path.parent().unwrap_or(Path::new("."));

                            // Required fields
                            if yaml.get("name").is_none() {
                                file_errors.push("missing 'name' field".to_string());
                            }
                            let language = yaml.get("language").and_then(|l| l.as_str());
                            if language.is_none() {
                                file_errors.push("missing 'language' field".to_string());
                            }

                            // Check main file exists
                            if let Some(lang) = language {
                                let main_exists = match lang {
                                    "rust" => {
                                        base_dir.join("main.rs").exists()
                                            || base_dir.join("src/main.rs").exists()
                                            || base_dir.join(CARGO_TOML).exists()
                                    }
                                    "python" => base_dir.join("main.py").exists(),
                                    _ => true,
                                };
                                if !main_exists {
                                    file_errors.push(format!("main file not found for '{}'", lang));
                                }
                            }

                            // Validate path dependencies exist
                            if let Ok(deps) =
                                parse_horus_yaml_dependencies_v2(yaml_path.to_str().unwrap_or(""))
                            {
                                for dep in &deps {
                                    if let DependencySource::Path(path_str) = &dep.source {
                                        let dep_path = if Path::new(path_str).is_absolute() {
                                            PathBuf::from(path_str)
                                        } else {
                                            base_dir.join(path_str)
                                        };
                                        if !dep_path.exists() {
                                            file_errors.push(format!(
                                                "dependency '{}' path not found: {}",
                                                dep.name,
                                                dep_path.display()
                                            ));
                                        }
                                    }
                                }
                            }

                            if file_errors.is_empty() {
                                println!("      {} manifest valid", cli_output::ICON_SUCCESS.green());
                            } else {
                                for err in &file_errors {
                                    println!("      {} {}", cli_output::ICON_ERROR.red(), err);
                                }
                                total_errors += file_errors.len();
                            }
                        }
                        Err(e) => {
                            println!("      {} YAML parse error: {}", cli_output::ICON_ERROR.red(), e);
                            total_errors += 1;
                        }
                    }
                }
                Err(e) => {
                    println!("      {} Read error: {}", cli_output::ICON_ERROR.red(), e);
                    total_errors += 1;
                }
            }
            files_checked += 1;
        }
    }

    // ═══════════════════════════════════════════════════════════
    // PHASE 2: Deep Rust compilation check (cargo check)
    // ═══════════════════════════════════════════════════════════
    if !cargo_dirs.is_empty() {
        println!("\n{}", "━".repeat(60).dimmed());
        println!(
            "{} Phase 2: Deep Rust check (cargo check)...\n",
            cli_output::ICON_INFO.cyan().bold()
        );

        for cargo_dir in &cargo_dirs {
            let rel_path = cargo_dir.strip_prefix(target_path).unwrap_or(cargo_dir);
            let display_path = if rel_path.as_os_str().is_empty() {
                "."
            } else {
                rel_path.to_str().unwrap_or(".")
            };
            print!("  {} {} ... ", "▸".cyan(), display_path);
            std::io::Write::flush(&mut std::io::stdout()).ok();

            let output = std::process::Command::new("cargo")
                .arg("check")
                .arg("--message-format=short")
                .current_dir(cargo_dir)
                .output();

            match output {
                Ok(result) if result.status.success() => {
                    println!("{}", cli_output::ICON_SUCCESS.green());
                }
                Ok(result) => {
                    println!("{}", cli_output::ICON_ERROR.red());
                    let stderr = String::from_utf8_lossy(&result.stderr);
                    for line in stderr.lines().take(5) {
                        if line.contains("error") {
                            println!("      {} {}", cli_output::ICON_ERROR.red(), line.trim());
                        }
                    }
                    total_errors += 1;
                }
                Err(e) => {
                    println!("{} cargo error: {}", cli_output::ICON_WARN.yellow(), e);
                    total_warnings += 1;
                }
            }
            files_checked += 1;
        }
    }

    // ═══════════════════════════════════════════════════════════
    // PHASE 3: Python validation (syntax + imports)
    // ═══════════════════════════════════════════════════════════
    if !python_files.is_empty() {
        println!("\n{}", "━".repeat(60).dimmed());
        println!(
            "{} Phase 3: Python validation (syntax + imports)...\n",
            cli_output::ICON_INFO.cyan().bold()
        );

        for py_path in &python_files {
            print!(
                "  {} {} ",
                "▸".cyan(),
                py_path
                    .strip_prefix(target_path)
                    .unwrap_or(py_path)
                    .display()
            );

            // Syntax check
            let syntax_check = std::process::Command::new("python3")
                .arg("-m")
                .arg("py_compile")
                .arg(py_path)
                .output();

            match syntax_check {
                Ok(result) if result.status.success() => {
                    // Syntax OK - now check imports
                    let import_script = format!(
                        r#"
import ast, sys
try:
    with open('{}') as f:
        tree = ast.parse(f.read())
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                imports.add(alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module.split('.')[0])
    for imp in imports:
        if imp not in ('__future__',):
            __import__(imp)
except ModuleNotFoundError as e:
    print(f'ModuleNotFoundError: {{e.name}}', file=sys.stderr)
    sys.exit(1)
except ImportError as e:
    print(f'ImportError: {{e}}', file=sys.stderr)
    sys.exit(1)
"#,
                        py_path.display()
                    );

                    let import_check = std::process::Command::new("python3")
                        .arg("-c")
                        .arg(&import_script)
                        .output();

                    match import_check {
                        Ok(r) if r.status.success() => {
                            println!("{}", cli_output::ICON_SUCCESS.green());
                        }
                        Ok(r) => {
                            println!("{}", cli_output::ICON_WARN.yellow());
                            let err = String::from_utf8_lossy(&r.stderr);
                            if !err.is_empty() {
                                println!(
                                    "      {} {}",
                                    cli_output::ICON_WARN.yellow(),
                                    err.lines().next().unwrap_or("").trim()
                                );
                            }
                            total_warnings += 1;
                        }
                        Err(_) => println!("{}", cli_output::ICON_SUCCESS.green()),
                    }
                }
                Ok(result) => {
                    println!("{}", cli_output::ICON_ERROR.red());
                    let error = String::from_utf8_lossy(&result.stderr);
                    if !error.is_empty() {
                        println!(
                            "      {} {}",
                            cli_output::ICON_ERROR.red(),
                            error.lines().next().unwrap_or("").trim()
                        );
                    }
                    total_errors += 1;
                }
                Err(_) => {
                    println!("{}", "⊘".dimmed());
                    if !quiet {
                        total_warnings += 1;
                    }
                }
            }
            files_checked += 1;
        }
    }

    // Summary
    println!("\n{}", "━".repeat(60).dimmed());
    println!("{} Workspace Check Summary\n", cli_output::ICON_INFO.cyan().bold());
    println!("  Files checked: {}", files_checked);

    if total_errors == 0 && total_warnings == 0 {
        println!("  Status: {} All checks passed!", cli_output::ICON_SUCCESS.green());
    } else {
        if total_errors > 0 {
            println!("  Errors: {} {}", cli_output::ICON_ERROR.red(), total_errors);
        }
        if total_warnings > 0 && !quiet {
            println!("  Warnings: {} {}", cli_output::ICON_WARN.yellow(), total_warnings);
        }
    }
    println!();

    if total_errors > 0 {
        return Err(HorusError::Config(format!(
            "{} error(s) found",
            total_errors
        )));
    }
    Ok(())
}

/// Check a single file (horus.yaml, .rs, or .py)
fn check_single_file(horus_yaml_path: &Path, quiet: bool) -> HorusResult<()> {
    let extension = horus_yaml_path.extension().and_then(|s| s.to_str());

    match extension {
        Some("rs") => check_rust_file(horus_yaml_path),
        Some("py") => check_python_file(horus_yaml_path),
        _ => check_yaml_file(horus_yaml_path, quiet),
    }
}

/// Check a single Rust file
fn check_rust_file(path: &Path) -> HorusResult<()> {
    println!("{} Checking Rust file: {}\n", cli_output::ICON_INFO.cyan(), path.display());

    print!("  {} Parsing Rust syntax... ", "▸".cyan());
    let content = fs::read_to_string(path)?;

    match syn::parse_file(&content) {
        Ok(_) => {
            println!("{}", cli_output::ICON_SUCCESS.green());
            println!("\n{} Syntax check passed!", cli_output::ICON_SUCCESS.green().bold());
        }
        Err(e) => {
            println!("{}", cli_output::ICON_ERROR.red());
            println!("\n{} Syntax error:", cli_output::ICON_ERROR.red().bold());
            println!("  {}", e);
            return Err(HorusError::Config(format!("Rust syntax error: {}", e)));
        }
    }

    if let Err(e) = check_hardware_requirements(path, "rust") {
        log::warn!("Hardware check error: {}", e);
        eprintln!("\n{} Hardware check error: {}", "[WARNING]".yellow(), e);
    }

    Ok(())
}

/// Check a single Python file
fn check_python_file(path: &Path) -> HorusResult<()> {
    println!("{} Checking Python file: {}\n", cli_output::ICON_INFO.cyan(), path.display());

    print!("  {} Parsing Python syntax... ", "▸".cyan());

    let output = std::process::Command::new("python3")
        .arg("-m")
        .arg("py_compile")
        .arg(path)
        .output();

    match output {
        Ok(result) if result.status.success() => {
            println!("{}", cli_output::ICON_SUCCESS.green());
            println!("\n{} Syntax check passed!", cli_output::ICON_SUCCESS.green().bold());
        }
        Ok(result) => {
            println!("{}", cli_output::ICON_ERROR.red());
            let error = String::from_utf8_lossy(&result.stderr);
            println!("\n{} Syntax error:", cli_output::ICON_ERROR.red().bold());
            println!("  {}", error);
            return Err(HorusError::Config(format!(
                "Python syntax error: {}",
                error
            )));
        }
        Err(e) => {
            println!("{}", "[WARNING]".yellow());
            println!(
                "\n{} Could not check Python syntax (python3 not found): {}",
                "[WARNING]".yellow(),
                e
            );
        }
    }
    Ok(())
}

/// Check a horus.yaml manifest file
fn check_yaml_file(horus_yaml_path: &Path, quiet: bool) -> HorusResult<()> {
    println!("{} Checking {}...\n", cli_output::ICON_INFO.cyan(), horus_yaml_path.display());

    let mut errors = Vec::new();
    let mut warn_msgs = Vec::new();
    let base_dir = horus_yaml_path.parent().unwrap_or(Path::new("."));

    // 1. YAML Syntax Validation
    print!("  {} Validating YAML syntax... ", "▸".cyan());
    let yaml_content = match fs::read_to_string(horus_yaml_path) {
        Ok(content) => {
            println!("{}", cli_output::ICON_SUCCESS.green());
            content
        }
        Err(e) => {
            println!("{}", cli_output::ICON_ERROR.red());
            errors.push(format!("Cannot read file: {}", e));
            String::new()
        }
    };

    let yaml_value: Option<serde_yaml::Value> = if !yaml_content.is_empty() {
        match serde_yaml::from_str(&yaml_content) {
            Ok(val) => Some(val),
            Err(e) => {
                errors.push(format!("Invalid YAML syntax: {}", e));
                None
            }
        }
    } else {
        errors.push(
            "Empty horus.yaml file — must contain at least 'name' and 'version' fields".to_string(),
        );
        None
    };

    // 2. Required Fields Check
    if let Some(ref yaml) = yaml_value {
        print!("  {} Checking required fields... ", "▸".cyan());
        let mut missing_fields = Vec::new();

        if yaml.get("name").is_none() {
            missing_fields.push("name");
        }
        if yaml.get("version").is_none() {
            missing_fields.push("version");
        }

        if missing_fields.is_empty() {
            println!("{}", cli_output::ICON_SUCCESS.green());
        } else {
            println!("{}", cli_output::ICON_ERROR.red());
            errors.push(format!(
                "Missing required fields: {}",
                missing_fields.join(", ")
            ));
        }

        // Optional fields warning
        if !quiet {
            if yaml.get("description").is_none() {
                warn_msgs.push("Optional field missing: description".to_string());
            }
            if yaml.get("author").is_none() {
                warn_msgs.push("Optional field missing: author".to_string());
            }
        }

        // License warning
        print!("  {} Checking license field... ", "▸".cyan());
        let missing_license_warning = "No license specified. Consider adding a license field (e.g., Apache-2.0, BSD-3-Clause).";
        if let Some(license) = yaml.get("license").and_then(|l| l.as_str()) {
            if license.trim().is_empty() {
                println!("{}", "[WARNING]".yellow());
                warn_msgs.push(missing_license_warning.to_string());
            } else {
                println!("{} ({})", cli_output::ICON_SUCCESS.green(), license.dimmed());
            }
        } else {
            println!("{}", "[WARNING]".yellow());
            warn_msgs.push(missing_license_warning.to_string());
        }

        // Language validation
        print!("  {} Validating language field... ", "▸".cyan());
        if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
            if language == "rust" || language == "python" {
                println!("{}", cli_output::ICON_SUCCESS.green());
            } else {
                println!("{}", cli_output::ICON_ERROR.red());
                errors.push(format!(
                    "Invalid language '{}' - must be: rust or python",
                    language
                ));
            }
        } else {
            println!("{}", cli_output::ICON_ERROR.red());
            errors
                .push("Missing or invalid 'language' field - must be: rust or python".to_string());
        }

        // Version format validation
        print!("  {} Validating version format... ", "▸".cyan());
        if let Some(version_str) = yaml.get("version").and_then(|v| v.as_str()) {
            use semver::Version;
            match Version::parse(version_str) {
                Ok(_) => println!("{}", cli_output::ICON_SUCCESS.green()),
                Err(e) => {
                    println!("{}", cli_output::ICON_ERROR.red());
                    errors.push(format!(
                        "Invalid version format '{}': {} (must be valid semver like 0.1.0)",
                        version_str, e
                    ));
                }
            }
        } else if yaml.get("version").is_some() {
            println!("{}", cli_output::ICON_ERROR.red());
            errors.push("Version field must be a string".to_string());
        }

        // Project name validation
        print!("  {} Validating project name... ", "▸".cyan());
        if let Some(name) = yaml.get("name").and_then(|n| n.as_str()) {
            let mut name_issues = Vec::new();

            if name.is_empty() {
                name_issues.push("name cannot be empty");
            }
            if name.contains(' ') {
                name_issues.push("name cannot contain spaces");
            }
            if name
                .chars()
                .any(|c| !c.is_ascii_alphanumeric() && c != '_' && c != '-')
            {
                name_issues
                    .push("name can only contain letters, numbers, hyphens, and underscores");
            }

            if name_issues.is_empty() {
                println!("{}", cli_output::ICON_SUCCESS.green());
                if !quiet && name.chars().any(|c| c.is_uppercase()) {
                    warn_msgs.push(format!(
                        "Project name '{}' contains uppercase - consider using lowercase",
                        name
                    ));
                }
            } else {
                println!("{}", cli_output::ICON_ERROR.red());
                for issue in name_issues {
                    errors.push(format!("Invalid project name: {}", issue));
                }
            }
        }

        // Main file existence check
        print!("  {} Checking for main file... ", "▸".cyan());
        if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
            let main_files = match language {
                "rust" => vec!["main.rs", "src/main.rs"],
                "python" => vec!["main.py"],
                _ => vec![],
            };

            let mut found = false;
            for main_file in &main_files {
                let path = base_dir.join(main_file);
                if path.exists() {
                    println!("{}", cli_output::ICON_SUCCESS.green());
                    found = true;
                    break;
                }
            }

            if !found && !main_files.is_empty() {
                println!("{}", cli_output::ICON_WARN.yellow());
                if !quiet {
                    warn_msgs.push(format!(
                        "No main file found - expected one of: {}",
                        main_files.join(", ")
                    ));
                }
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    }

    // 3. Parse Dependencies
    print!("  {} Parsing dependencies... ", "▸".cyan());
    let dep_specs = match parse_horus_yaml_dependencies_v2(&horus_yaml_path.to_string_lossy()) {
        Ok(specs) => {
            println!("{}", cli_output::ICON_SUCCESS.green());
            specs
        }
        Err(e) => {
            println!("{}", cli_output::ICON_ERROR.red());
            errors.push(format!("Failed to parse dependencies: {}", e));
            Vec::new()
        }
    };

    // 4. Check for Duplicates
    if !dep_specs.is_empty() {
        print!("  {} Checking for duplicates... ", "▸".cyan());
        let mut seen = HashSet::new();
        let mut duplicates = Vec::new();

        for spec in &dep_specs {
            if !seen.insert(&spec.name) {
                duplicates.push(spec.name.clone());
            }
        }

        if duplicates.is_empty() {
            println!("{}", cli_output::ICON_SUCCESS.green());
        } else {
            println!("{}", cli_output::ICON_ERROR.red());
            errors.push(format!("Duplicate dependencies: {}", duplicates.join(", ")));
        }
    }

    // 5. Validate Path Dependencies
    println!("\n  {} Checking path dependencies...", "▸".cyan());
    let mut path_deps_found = false;

    for spec in &dep_specs {
        if let DependencySource::Path(ref path) = spec.source {
            path_deps_found = true;
            let resolved_path = if path.is_absolute() {
                path.clone()
            } else {
                base_dir.join(path)
            };

            if resolved_path.exists() {
                if resolved_path.is_dir() {
                    println!("    {} {} ({})", cli_output::ICON_SUCCESS.green(), spec.name, path.display());
                } else {
                    println!(
                        "    {} {} ({}) - Not a directory",
                        cli_output::ICON_ERROR.red(),
                        spec.name,
                        path.display()
                    );
                    errors.push(format!(
                        "Path dependency '{}' is not a directory: {}",
                        spec.name,
                        path.display()
                    ));
                }
            } else {
                println!(
                    "    {} {} ({}) - Path not found",
                    cli_output::ICON_ERROR.red(),
                    spec.name,
                    path.display()
                );
                errors.push(format!(
                    "Path dependency '{}' not found: {}",
                    spec.name,
                    path.display()
                ));
            }
        }
    }

    if !path_deps_found {
        println!("    {} No path dependencies", "⊘".dimmed());
    }

    // 6. Circular Dependency Detection
    println!("\n  {} Checking for circular dependencies...", "▸".cyan());
    let mut circular_found = false;

    for spec in &dep_specs {
        if let DependencySource::Path(ref path) = spec.source {
            let resolved_path = if path.is_absolute() {
                path.clone()
            } else {
                base_dir.join(path)
            };

            let target_yaml = resolved_path.join(HORUS_YAML);
            if target_yaml.exists() {
                if let Ok(target_deps) =
                    parse_horus_yaml_dependencies_v2(&target_yaml.to_string_lossy())
                {
                    let our_name = yaml_value
                        .as_ref()
                        .and_then(|y| y.get("name"))
                        .and_then(|n| n.as_str())
                        .unwrap_or("");

                    for target_dep in target_deps {
                        if target_dep.name == our_name {
                            if let DependencySource::Path(_) = target_dep.source {
                                circular_found = true;
                                errors.push(format!(
                                    "Circular dependency detected: {} -> {} -> {}",
                                    our_name, spec.name, our_name
                                ));
                                println!(
                                    "    {} Circular: {} <-> {}",
                                    cli_output::ICON_ERROR.red(),
                                    our_name,
                                    spec.name
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    if !circular_found {
        println!("    {} No circular dependencies", cli_output::ICON_SUCCESS.green());
    }

    // 7. Version Constraint Validation
    print!("\n  {} Validating version constraints... ", "▸".cyan());

    for spec in &dep_specs {
        if spec.requirement.to_string() == "*" && !quiet {
            warn_msgs.push(format!(
                "Dependency '{}' uses wildcard version (*) - consider pinning to a specific version",
                spec.name
            ));
        }
    }

    println!("{}", cli_output::ICON_SUCCESS.green());

    // 8. Workspace Structure Check
    print!("\n  {} Checking workspace structure... ", "▸".cyan());
    let base_dir = horus_yaml_path.parent().unwrap_or_else(|| Path::new("."));
    let horus_dir = base_dir.join(".horus");

    if horus_dir.exists() && horus_dir.is_dir() {
        println!("{}", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", cli_output::ICON_WARN.yellow());
        if !quiet {
            warn_msgs.push(
                "No .horus/ workspace directory found - will be created on first run".to_string(),
            );
        }
    }

    // 9. Dependency Installation Check
    print!("  {} Checking installed dependencies... ", "▸".cyan());
    if horus_dir.exists() {
        let packages_dir = horus_dir.join("packages");
        if packages_dir.exists() {
            let mut missing_deps = Vec::new();

            for spec in &dep_specs {
                match &spec.source {
                    DependencySource::Registry => {
                        let package_dir = packages_dir.join(&spec.name);
                        if !package_dir.exists() {
                            missing_deps.push(spec.name.clone());
                        }
                    }
                    DependencySource::Path(_) => {}
                    DependencySource::Git { .. } => {}
                    DependencySource::Pip { package_name } => {
                        let _ = package_name;
                    }
                }
            }

            if missing_deps.is_empty() {
                println!("{}", cli_output::ICON_SUCCESS.green());
            } else {
                println!("{}", cli_output::ICON_WARN.yellow());
                if !missing_deps.is_empty() && !quiet {
                    warn_msgs.push(format!(
                        "Missing dependencies: {} (run 'horus run' to install)",
                        missing_deps.join(", ")
                    ));
                }
            }
        } else {
            println!("{}", cli_output::ICON_WARN.yellow());
            if !quiet {
                warn_msgs
                    .push("No packages directory - dependencies not installed yet".to_string());
            }
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // 10. Toolchain Check
    print!("  {} Checking toolchain... ", "▸".cyan());
    if let Some(ref yaml) = yaml_value {
        if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
            let toolchain_available = match language {
                "rust" => std::process::Command::new("rustc")
                    .arg("--version")
                    .output()
                    .map(|o| o.status.success())
                    .unwrap_or(false),
                "python" => std::process::Command::new("python3")
                    .arg("--version")
                    .output()
                    .map(|o| o.status.success())
                    .unwrap_or(false),
                _ => false,
            };

            if toolchain_available {
                println!("{}", cli_output::ICON_SUCCESS.green());
            } else {
                println!("{}", cli_output::ICON_ERROR.red());
                errors.push(format!(
                    "Required toolchain for '{}' not found in PATH",
                    language
                ));
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // 11. Code Validation
    print!("  {} Validating code syntax... ", "▸".cyan());
    if let Some(ref yaml) = yaml_value {
        if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
            match language {
                "rust" => {
                    let has_cargo = base_dir.join(CARGO_TOML).exists();
                    let has_main =
                        base_dir.join("main.rs").exists() || base_dir.join("src/main.rs").exists();

                    if has_cargo || has_main {
                        let check_result = std::process::Command::new("cargo")
                            .arg("build")
                            .arg("--quiet")
                            .current_dir(base_dir)
                            .output();

                        match check_result {
                            Ok(output) if output.status.success() => {
                                println!("{}", cli_output::ICON_SUCCESS.green());
                            }
                            Ok(_) => {
                                println!("{}", cli_output::ICON_ERROR.red());
                                errors.push(
                                    "Rust code has compilation errors (run 'cargo build' for details)"
                                        .to_string(),
                                );
                            }
                            Err(_) => {
                                println!("{}", cli_output::ICON_WARN.yellow());
                                if !quiet {
                                    warn_msgs.push(
                                        "Could not run 'cargo build' - skipping code validation"
                                            .to_string(),
                                    );
                                }
                            }
                        }
                    } else {
                        println!("{}", "⊘".dimmed());
                    }
                }
                "python" => {
                    let main_py = base_dir.join("main.py");
                    if main_py.exists() {
                        let check_result = std::process::Command::new("python3")
                            .arg("-m")
                            .arg("py_compile")
                            .arg(&main_py)
                            .output();

                        match check_result {
                            Ok(output) if output.status.success() => {
                                println!("{}", cli_output::ICON_SUCCESS.green());
                            }
                            Ok(_) => {
                                println!("{}", cli_output::ICON_ERROR.red());
                                errors.push("Python code has syntax errors".to_string());
                            }
                            Err(_) => {
                                println!("{}", cli_output::ICON_WARN.yellow());
                                if !quiet {
                                    warn_msgs.push("Could not validate Python syntax".to_string());
                                }
                            }
                        }
                    } else {
                        println!("{}", "⊘".dimmed());
                    }
                }
                _ => {
                    println!("{}", "⊘".dimmed());
                }
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // 12. HORUS System Check
    print!("\n  {} Checking HORUS installation... ", "▸".cyan());
    let horus_version = env!("CARGO_PKG_VERSION");
    println!("v{}", horus_version.dimmed());

    // 13. Registry Connectivity
    print!("  {} Checking registry connectivity... ", "▸".cyan());
    let registry_available = std::process::Command::new("ping")
        .arg("-c")
        .arg("1")
        .arg("-W")
        .arg("1")
        .arg("registry.horus.rs")
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false);

    if registry_available {
        println!("{}", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", "⊘".dimmed());
        if !quiet {
            warn_msgs.push("Registry not reachable - package installation may fail".to_string());
        }
    }

    // 14. System Requirements Check
    print!("  {} Checking system requirements... ", "▸".cyan());
    let mut sys_issues = Vec::new();

    if has_native_shm() {
        #[cfg(target_os = "linux")]
        {
            let dev_shm = std::path::Path::new("/dev/shm");
            if !dev_shm.exists() {
                sys_issues.push("/dev/shm not available");
            } else if let Ok(metadata) = std::fs::metadata(dev_shm) {
                use std::os::unix::fs::PermissionsExt;
                let mode = metadata.permissions().mode();
                if mode & 0o777 != 0o777 {
                    sys_issues.push("/dev/shm permissions restrictive");
                }
            }
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        let shm_path = shm_base_dir();
        if std::fs::create_dir_all(&shm_path).is_err() {
            sys_issues.push("Cannot create shared memory directory");
        }
    }

    if sys_issues.is_empty() {
        println!("{}", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", cli_output::ICON_WARN.yellow());
        for issue in sys_issues {
            if !quiet {
                warn_msgs.push(format!("System issue: {}", issue));
            }
        }
    }

    // Disk Space Check
    print!("  {} Checking available disk space... ", "▸".cyan());
    #[cfg(target_os = "linux")]
    {
        use std::process::Command;

        if let Ok(output) = Command::new("df").arg("-BM").arg(base_dir).output() {
            if output.status.success() {
                let output_str = String::from_utf8_lossy(&output.stdout);
                if let Some(line) = output_str.lines().nth(1) {
                    let parts: Vec<&str> = line.split_whitespace().collect();
                    if parts.len() >= 4 {
                        if let Some(available) = parts[3].strip_suffix('M') {
                            if let Ok(available_mb) = available.parse::<u64>() {
                                if available_mb < 500 {
                                    println!("{} ({}MB free)", cli_output::ICON_WARN.yellow(), available_mb);
                                    if !quiet {
                                        warn_msgs.push(format!(
                                            "Low disk space: only {}MB available (recommended: 500MB+)",
                                            available_mb
                                        ));
                                    }
                                } else if available_mb < 100 {
                                    println!("{} ({}MB free)", cli_output::ICON_ERROR.red(), available_mb);
                                    errors.push(format!(
                                        "Critically low disk space: only {}MB available",
                                        available_mb
                                    ));
                                } else {
                                    println!("{} ({}MB free)", cli_output::ICON_SUCCESS.green(), available_mb);
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }
                        } else {
                            println!("{}", "⊘".dimmed());
                        }
                    } else {
                        println!("{}", "⊘".dimmed());
                    }
                } else {
                    println!("{}", "⊘".dimmed());
                }
            } else {
                println!("{}", "⊘".dimmed());
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        println!("{}", "⊘".dimmed());
    }

    // 15. API Usage Check
    print!("  {} Checking API usage... ", "▸".cyan());
    if let Some(ref yaml) = yaml_value {
        if let Some(language) = yaml.get("language").and_then(|l| l.as_str()) {
            match language {
                "rust" => {
                    let uses_horus = dep_specs
                        .iter()
                        .any(|spec| spec.name == "horus" || spec.name == "horus_macros");

                    if uses_horus {
                        let main_paths =
                            vec![base_dir.join("main.rs"), base_dir.join("src/main.rs")];

                        let mut has_scheduler = false;
                        for main_path in main_paths {
                            if main_path.exists() {
                                if let Ok(content) = std::fs::read_to_string(&main_path) {
                                    has_scheduler = content.contains("Scheduler::new")
                                        || content.contains("scheduler.register");
                                    if has_scheduler {
                                        break;
                                    }
                                }
                            }
                        }

                        if has_scheduler {
                            println!("{}", cli_output::ICON_SUCCESS.green());
                        } else {
                            println!("{}", cli_output::ICON_WARN.yellow());
                            if !quiet {
                                warn_msgs.push(
                                    "HORUS dependency found but no Scheduler usage detected"
                                        .to_string(),
                                );
                            }
                        }
                    } else {
                        println!("{}", "⊘".dimmed());
                    }
                }
                "python" => {
                    let uses_horus = dep_specs.iter().any(|spec| spec.name == "horus_py");

                    if uses_horus {
                        let main_py = base_dir.join("main.py");
                        if main_py.exists() {
                            if let Ok(content) = std::fs::read_to_string(&main_py) {
                                if content.contains("import horus")
                                    || content.contains("from horus")
                                {
                                    println!("{}", cli_output::ICON_SUCCESS.green());
                                } else {
                                    println!("{}", cli_output::ICON_WARN.yellow());
                                    if !quiet {
                                        warn_msgs.push(
                                            "horus_py dependency but no 'import horus' found"
                                                .to_string(),
                                        );
                                    }
                                }
                            } else {
                                println!("{}", "⊘".dimmed());
                            }
                        } else {
                            println!("{}", "⊘".dimmed());
                        }
                    } else {
                        println!("{}", "⊘".dimmed());
                    }

                    // Check external Python dependencies
                    print!("  {} Checking Python external dependencies... ", "▸".cyan());
                    let main_py = base_dir.join("main.py");
                    if main_py.exists() {
                        match parse_python_imports(&main_py) {
                            Ok(imports) if !imports.is_empty() => {
                                let mut missing_packages = Vec::new();
                                for package in &imports {
                                    if !check_system_package_exists(package) {
                                        missing_packages.push(package.clone());
                                    }
                                }

                                if missing_packages.is_empty() {
                                    println!("{} ({})", cli_output::ICON_SUCCESS.green(), imports.len());
                                } else {
                                    println!("{}", cli_output::ICON_ERROR.red());
                                    errors.push(format!(
                                        "Missing Python packages: {} (install with: pip install {})",
                                        missing_packages.join(", "),
                                        missing_packages.join(" ")
                                    ));
                                }
                            }
                            Ok(_) => {
                                println!("{}", "⊘".dimmed());
                            }
                            Err(e) => {
                                println!("{}", cli_output::ICON_WARN.yellow());
                                if !quiet {
                                    warn_msgs
                                        .push(format!("Could not parse Python imports: {}", e));
                                }
                            }
                        }
                    } else {
                        println!("{}", "⊘".dimmed());
                    }
                }
                _ => {
                    println!("{}", "⊘".dimmed());
                }
            }
        } else {
            println!("{}", "⊘".dimmed());
        }
    } else {
        println!("{}", "⊘".dimmed());
    }

    // Print Summary
    println!();
    if !quiet {
        if warn_msgs.is_empty() {
            println!("{} No warnings detected.", cli_output::ICON_SUCCESS.green());
        } else {
            println!("{} {} warning(s):", "[WARNING]".yellow(), warn_msgs.len());
            for warn in &warn_msgs {
                println!("  - {}", warn);
            }
        }
        println!();
    }

    if errors.is_empty() {
        println!("{} All checks passed!", cli_output::ICON_SUCCESS.green().bold());
        Ok(())
    } else {
        println!(
            "{} {} error(s) found:\n",
            cli_output::ICON_ERROR.red().bold(),
            errors.len()
        );
        for (i, err) in errors.iter().enumerate() {
            println!("  {}. {}", i + 1, err);
        }
        println!();
        Err(HorusError::Config("Validation failed".to_string()))
    }
}

// Helper functions for system package detection

pub fn check_system_package_exists(package_name: &str) -> bool {
    use std::process::Command;

    let py_check = Command::new("python3")
        .args(["-m", "pip", "show", package_name])
        .output();

    if let Ok(output) = py_check {
        if output.status.success() {
            return true;
        }
    }

    if let Some(home) = dirs::home_dir() {
        let cargo_bin = home.join(".cargo/bin").join(package_name);
        if cargo_bin.exists() {
            return true;
        }
    }

    false
}

/// Parse Python imports from a file
pub(crate) fn parse_python_imports(python_file: &Path) -> Result<Vec<String>, std::io::Error> {
    let content = std::fs::read_to_string(python_file)?;
    let mut imports = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();

        if trimmed.starts_with('#') || trimmed.is_empty() {
            continue;
        }

        if let Some(rest) = trimmed.strip_prefix("import ") {
            let module = rest
                .split_whitespace()
                .next()
                .unwrap_or("")
                .split('.')
                .next()
                .unwrap_or("")
                .split(',')
                .next()
                .unwrap_or("")
                .trim();

            if !module.is_empty() && !imports.contains(&module.to_string()) {
                imports.push(module.to_string());
            }
        }

        if let Some(rest) = trimmed.strip_prefix("from ") {
            if let Some(module) = rest.split_whitespace().next() {
                let base_module = module.split('.').next().unwrap_or("");
                if !base_module.is_empty() && !imports.contains(&base_module.to_string()) {
                    imports.push(base_module.to_string());
                }
            }
        }
    }

    let stdlib_modules = vec![
        "os",
        "sys",
        "re",
        "json",
        "math",
        "time",
        "datetime",
        "collections",
        "itertools",
        "functools",
        "pathlib",
        "typing",
        "abc",
        "io",
        "logging",
        "argparse",
        "subprocess",
        "threading",
        "multiprocessing",
        "queue",
        "socket",
        "http",
        "urllib",
        "email",
        "xml",
        "html",
        "random",
        "string",
        "unittest",
        "pytest",
        "asyncio",
        "concurrent",
        "pickle",
        "copy",
        "enum",
        "dataclasses",
        "contextlib",
        "warnings",
        "traceback",
        "pdb",
        "timeit",
    ];

    imports.retain(|module| !stdlib_modules.contains(&module.as_str()) && module != "horus");

    Ok(imports)
}

#[derive(Debug, Clone, PartialEq)]
pub enum MissingSystemChoice {
    InstallGlobal,
    InstallLocal,
    Skip,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use tempfile::TempDir;

    /// Helper: write a horus.yaml with given content into a temp dir and return the path
    fn write_yaml(dir: &TempDir, content: &str) -> PathBuf {
        let yaml_path = dir.path().join("horus.yaml");
        fs::write(&yaml_path, content).unwrap();
        yaml_path
    }

    // ─── YAML Syntax Tests ───

    #[test]
    fn valid_yaml_passes_check() {
        let dir = TempDir::new().unwrap();
        // Create a valid horus.yaml with all required fields
        // Don't create Cargo.toml — code validation step simply skips
        write_yaml(&dir, "name: my-robot\nversion: \"0.1.0\"\nlanguage: rust\n");
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(
            result.is_ok(),
            "Valid horus.yaml should pass: {:?}",
            result.err()
        );
    }

    #[test]
    fn malformed_yaml_returns_error() {
        let dir = TempDir::new().unwrap();
        // Deliberately malformed YAML (tabs in wrong place, invalid mapping)
        write_yaml(&dir, "name: [\ninvalid: yaml: content:\n  - broken");
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Malformed YAML should fail validation");
    }

    #[test]
    fn empty_yaml_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(&dir, "");
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        // Empty file has no required fields
        assert!(result.is_err(), "Empty YAML should fail");
    }

    // ─── Required Fields Tests ───

    #[test]
    fn missing_name_field_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(&dir, "version: \"0.1.0\"\nlanguage: rust\n");
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Missing 'name' should fail");
    }

    #[test]
    fn missing_version_field_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(&dir, "name: my-robot\nlanguage: rust\n");
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Missing 'version' should fail");
    }

    #[test]
    fn missing_language_field_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(&dir, "name: my-robot\nversion: \"0.1.0\"\n");
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Missing 'language' should fail");
    }

    // ─── Version Format Tests ───

    #[test]
    fn invalid_version_format_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(
            &dir,
            "name: my-robot\nversion: not_a_version\nlanguage: rust\n",
        );
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Invalid semver version should fail");
    }

    #[test]
    fn valid_semver_version_passes() {
        let dir = TempDir::new().unwrap();
        write_yaml(&dir, "name: my-robot\nversion: \"1.2.3\"\nlanguage: rust\n");
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(
            result.is_ok(),
            "Valid semver should pass: {:?}",
            result.err()
        );
    }

    // ─── Language Validation Tests ───

    #[test]
    fn invalid_language_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(
            &dir,
            "name: my-robot\nversion: \"0.1.0\"\nlanguage: javascript\n",
        );
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Invalid language 'javascript' should fail");
    }

    // ─── Project Name Validation Tests ───

    #[test]
    fn name_with_spaces_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(
            &dir,
            "name: \"my robot\"\nversion: \"0.1.0\"\nlanguage: rust\n",
        );
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Name with spaces should fail");
    }

    #[test]
    fn name_with_special_chars_returns_error() {
        let dir = TempDir::new().unwrap();
        write_yaml(
            &dir,
            "name: \"my@robot!\"\nversion: \"0.1.0\"\nlanguage: rust\n",
        );
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(result.is_err(), "Name with special chars should fail");
    }

    #[test]
    fn name_with_hyphens_and_underscores_passes() {
        let dir = TempDir::new().unwrap();
        write_yaml(
            &dir,
            "name: my-robot_v2\nversion: \"0.1.0\"\nlanguage: rust\n",
        );
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(
            result.is_ok(),
            "Hyphens and underscores in name should pass: {:?}",
            result.err()
        );
    }

    // ─── Path Not Found Tests ───

    #[test]
    fn nonexistent_path_returns_error() {
        let result = run_check(
            Some(PathBuf::from("/tmp/nonexistent_horus_path_12345")),
            true,
            false,
        );
        assert!(result.is_err(), "Non-existent path should fail");
    }

    // ─── Circular Dependency Detection ───

    #[test]
    fn circular_dependency_detected() {
        let dir = TempDir::new().unwrap();

        // Create package A that depends on B
        let pkg_a = dir.path().join("pkg_a");
        let pkg_b = dir.path().join("pkg_b");
        fs::create_dir_all(&pkg_a).unwrap();
        fs::create_dir_all(&pkg_b).unwrap();

        // A depends on B via path (map format: dependencies: { pkg-b: { path: ... } })
        fs::write(
            pkg_a.join("horus.yaml"),
            format!(
                "name: pkg-a\nversion: \"0.1.0\"\nlanguage: rust\ndependencies:\n  pkg-b:\n    path: \"{}\"\n",
                pkg_b.display()
            ),
        )
        .unwrap();

        // B depends on A via path (circular!)
        fs::write(
            pkg_b.join("horus.yaml"),
            format!(
                "name: pkg-b\nversion: \"0.1.0\"\nlanguage: rust\ndependencies:\n  pkg-a:\n    path: \"{}\"\n",
                pkg_a.display()
            ),
        )
        .unwrap();

        // Check package A — should detect the circular dependency
        let result = run_check(Some(pkg_a.join("horus.yaml")), true, false);
        assert!(result.is_err(), "Circular dependency A→B→A should fail");
    }

    // ─── Path Dependency to Non-existent Directory ───

    #[test]
    fn path_dependency_nonexistent_returns_error() {
        let dir = TempDir::new().unwrap();
        // Use map format: dependencies: { missing-dep: { path: ./nonexistent_dir } }
        write_yaml(
            &dir,
            "name: my-robot\nversion: \"0.1.0\"\nlanguage: rust\ndependencies:\n  missing-dep:\n    path: ./nonexistent_dir\n",
        );
        let result = run_check(Some(dir.path().join("horus.yaml")), true, false);
        assert!(
            result.is_err(),
            "Path dependency to non-existent dir should fail"
        );
    }

    // ─── parse_python_imports Tests ───

    #[test]
    fn parse_python_imports_basic() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import numpy\nimport pandas\nfrom torch import nn\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(imports.contains(&"numpy".to_string()));
        assert!(imports.contains(&"pandas".to_string()));
        assert!(imports.contains(&"torch".to_string()));
    }

    #[test]
    fn parse_python_imports_filters_stdlib() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import os\nimport sys\nimport json\nimport numpy\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        // os, sys, json are stdlib — should be filtered
        assert!(!imports.contains(&"os".to_string()));
        assert!(!imports.contains(&"sys".to_string()));
        assert!(!imports.contains(&"json".to_string()));
        // numpy is NOT stdlib — should be included
        assert!(imports.contains(&"numpy".to_string()));
    }

    #[test]
    fn parse_python_imports_handles_comments_and_empty() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "# This is a comment\n\nimport numpy\n# import pandas\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(imports.contains(&"numpy".to_string()));
        assert!(
            !imports.contains(&"pandas".to_string()),
            "Commented imports should be ignored"
        );
    }

    #[test]
    fn parse_python_imports_from_submodule() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(&py_file, "from scipy.spatial import KDTree\n").unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(
            imports.contains(&"scipy".to_string()),
            "Should extract base module from 'from' imports"
        );
    }

    #[test]
    fn parse_python_imports_no_duplicates() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import numpy\nimport numpy\nfrom numpy import array\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        let numpy_count = imports.iter().filter(|i| *i == "numpy").count();
        assert_eq!(numpy_count, 1, "Should not have duplicate imports");
    }

    #[test]
    fn parse_python_imports_empty_file() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(&py_file, "").unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        assert!(imports.is_empty(), "Empty file should have no imports");
    }

    #[test]
    fn parse_python_imports_mixed_stdlib_and_external() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import os\nimport numpy\nimport sys\nimport custom_pkg\nfrom json import loads\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        // stdlib filtered
        assert!(!imports.contains(&"os".to_string()));
        assert!(!imports.contains(&"sys".to_string()));
        assert!(!imports.contains(&"json".to_string()));
        // external kept
        assert!(imports.contains(&"numpy".to_string()));
        assert!(imports.contains(&"custom_pkg".to_string()));
        assert_eq!(imports.len(), 2);
    }

    #[test]
    fn parse_python_imports_horus_filtered() {
        let dir = TempDir::new().unwrap();
        let py_file = dir.path().join("test.py");
        fs::write(
            &py_file,
            "import horus\nfrom horus import Scheduler\nimport numpy\n",
        )
        .unwrap();

        let imports = parse_python_imports(&py_file).unwrap();
        // horus is explicitly filtered
        assert!(!imports.contains(&"horus".to_string()));
        assert!(imports.contains(&"numpy".to_string()));
    }

    // ─── System Package Existence Tests ───

    #[test]
    fn check_nonexistent_package_returns_false() {
        assert!(
            !check_system_package_exists("nonexistent_pkg_xyz_12345"),
            "Non-existent package should return false"
        );
    }

    // ─── SHM Availability Tests ───

    #[test]
    fn shm_is_available_on_linux() {
        // HORUS requires /dev/shm on Linux for zero-copy IPC
        assert!(
            horus_core::memory::has_native_shm(),
            "HORUS requires native SHM support"
        );

        #[cfg(target_os = "linux")]
        {
            let dev_shm = std::path::Path::new("/dev/shm");
            assert!(dev_shm.exists(), "/dev/shm must exist for HORUS IPC");
            assert!(dev_shm.is_dir(), "/dev/shm must be a directory");
        }
    }

    // ─── MissingSystemChoice Tests ───

    #[test]
    fn missing_system_choice_variants() {
        // Verify the enum variants exist and are distinguishable
        let install_global = MissingSystemChoice::InstallGlobal;
        let install_local = MissingSystemChoice::InstallLocal;
        let skip = MissingSystemChoice::Skip;

        assert_eq!(install_global, MissingSystemChoice::InstallGlobal);
        assert_eq!(install_local, MissingSystemChoice::InstallLocal);
        assert_eq!(skip, MissingSystemChoice::Skip);
        assert_ne!(install_global, skip);
    }
}

pub fn prompt_missing_system_package(
    package_name: &str,
) -> Result<MissingSystemChoice, HorusError> {
    use std::io::{self, Write};

    println!(
        "\n  System package '{}' was expected but not found.",
        package_name
    );
    println!("  What would you like to do?");
    println!("    [1] Install to HORUS global cache (shared across projects)");
    println!("    [2] Install to HORUS local (this project only)");
    println!("    [3] Skip (you will install it manually later)");

    print!("\n  Choice [1-3]: ");
    io::stdout()
        .flush()
        .map_err(|e| HorusError::Config(e.to_string()))?;

    let mut input = String::new();
    io::stdin()
        .read_line(&mut input)
        .map_err(|e| HorusError::Config(e.to_string()))?;

    match input.trim() {
        "1" => Ok(MissingSystemChoice::InstallGlobal),
        "2" => Ok(MissingSystemChoice::InstallLocal),
        "3" => Ok(MissingSystemChoice::Skip),
        _ => {
            println!("  Invalid choice, defaulting to Skip");
            Ok(MissingSystemChoice::Skip)
        }
    }
}
