//! Message command - Message type introspection
//!
//! Lists and inspects HORUS message types defined in horus_types.

use crate::cli_output;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// Message type information
#[derive(Debug, Clone)]
pub struct MessageInfo {
    /// Message type name
    pub name: String,
    /// Module/category (e.g., "control", "sensor", "vision")
    pub module: String,
    /// Fields in the message
    pub fields: Vec<FieldInfo>,
    /// Documentation comment
    pub doc: String,
    /// Source file path
    pub source_file: String,
}

/// Field information
#[derive(Debug, Clone)]
pub struct FieldInfo {
    /// Field name
    pub name: String,
    /// Field type
    pub field_type: String,
    /// Documentation comment
    pub doc: String,
}

/// List all message types
pub fn list_messages(verbose: bool, filter: Option<&str>, json: bool) -> HorusResult<()> {
    let messages = discover_messages()?;

    // Apply filter if specified
    let filtered: Vec<_> = if let Some(f) = filter {
        let f_lower = f.to_lowercase();
        messages
            .iter()
            .filter(|m| {
                m.name.to_lowercase().contains(&f_lower)
                    || m.module.to_lowercase().contains(&f_lower)
            })
            .collect()
    } else {
        messages.iter().collect()
    };

    if json {
        let items: Vec<_> = filtered
            .iter()
            .map(|m| {
                let hash = compute_message_definition_hash(m);
                serde_json::json!({
                    "name": m.name,
                    "module": m.module,
                    "fields": m.fields.len(),
                    "hash": hash,
                    "doc": m.doc,
                    "source_file": m.source_file
                })
            })
            .collect();
        let output = serde_json::json!({
            "count": items.len(),
            "items": items
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    if filtered.is_empty() {
        if filter.is_some() {
            println!("{}", "No message types found matching filter.".yellow());
        } else {
            println!("{}", "No message types found.".yellow());
        }
        return Ok(());
    }

    println!("{}", "HORUS Message Types".green().bold());
    println!();

    if verbose {
        // Group by module
        let mut by_module: HashMap<String, Vec<&MessageInfo>> = HashMap::new();
        for msg in &filtered {
            by_module.entry(msg.module.clone()).or_default().push(msg);
        }

        let mut modules: Vec<_> = by_module.keys().cloned().collect();
        modules.sort();

        for module in modules {
            println!("  {}", format!("{}:", module).cyan().bold());
            let Some(msgs) = by_module.get(&module) else {
                continue;
            };
            for msg in msgs {
                println!("    {} {}", "".white(), msg.name.white().bold());
                if !msg.doc.is_empty() {
                    // Truncate doc to first line
                    let first_line = msg.doc.lines().next().unwrap_or("");
                    println!("      {}", first_line.dimmed());
                }
                if !msg.fields.is_empty() {
                    let field_count = msg.fields.len();
                    println!(
                        "      {} fields: {}",
                        cli_output::ICON_HINT.dimmed(),
                        field_count
                    );
                }
            }
            println!();
        }
    } else {
        // Compact table view
        println!(
            "  {:<30} {:<15} {:>8}",
            "MESSAGE TYPE".dimmed(),
            "MODULE".dimmed(),
            "FIELDS".dimmed()
        );
        println!("  {}", "-".repeat(55).dimmed());

        for msg in &filtered {
            let field_count = if msg.fields.is_empty() {
                "-".to_string()
            } else {
                msg.fields.len().to_string()
            };
            println!("  {:<30} {:<15} {:>8}", msg.name, msg.module, field_count);
        }
    }

    println!();
    println!("  {} {} message type(s)", "Total:".dimmed(), filtered.len());

    Ok(())
}

/// Resolve a user-supplied type name to exactly one definition.
///
/// Accepts a bare name (`CmdVel`) or a module-qualified one
/// (`sensor::LaserScan`), case-insensitively.
///
/// A bare name that matches definitions in more than one module used to
/// resolve to whichever happened to sort first. Once `horus-tf` joined the
/// registry that became a wrong answer with no warning: `TransformStamped`
/// exists both in `horus_types::math` (translation/rotation/timestamp_ns) and
/// in `horus-tf` (parent_frame/child_frame/timestamp_ns/transform), so
/// `horus msg hash TransformStamped` printed the hash of a type the user was
/// not asking about — precisely the layout-mismatch confusion the command
/// exists to resolve. Report the ambiguity instead, and name the qualified
/// spellings that settle it.
fn resolve_message<'a>(messages: &'a [MessageInfo], name: &str) -> HorusResult<&'a MessageInfo> {
    if let Some(msg) = messages
        .iter()
        .find(|m| format!("{}::{}", m.module, m.name).eq_ignore_ascii_case(name))
    {
        return Ok(msg);
    }

    let matches: Vec<&MessageInfo> = messages
        .iter()
        .filter(|m| m.name.eq_ignore_ascii_case(name))
        .collect();

    match matches.as_slice() {
        [] => Err(HorusError::Config(ConfigError::Other(format!(
            "Message type '{}' not found. Use 'horus msg list' to see available types.",
            name
        )))),
        [only] => Ok(only),
        many => {
            // Same name and the same layout everywhere: there is nothing to
            // choose between, so answering is not a guess.
            let first = compute_message_definition_hash(many[0]);
            if many
                .iter()
                .all(|m| compute_message_definition_hash(m) == first)
            {
                return Ok(many[0]);
            }
            let mut detail = String::new();
            for m in many {
                detail.push_str(&format!(
                    "\n    {}::{}  {}  ({})",
                    m.module,
                    m.name,
                    compute_message_definition_hash(m),
                    m.source_file
                ));
            }
            Err(HorusError::Config(ConfigError::Other(format!(
                "Message type '{}' is ambiguous - {} types share that name with \
                 different layouts:{}\n  Use the module-qualified name, e.g. \
                 '{}::{}'.",
                name,
                many.len(),
                detail,
                many[0].module,
                many[0].name
            ))))
        }
    }
}

/// Show detailed info about a message type
pub fn show_message(name: &str, json: bool) -> HorusResult<()> {
    let messages = discover_messages()?;
    let msg = resolve_message(&messages, name)?;

    if json {
        let fields: Vec<_> = msg
            .fields
            .iter()
            .map(|f| {
                serde_json::json!({
                    "name": f.name,
                    "type": f.field_type,
                    "doc": f.doc,
                })
            })
            .collect();
        let hash = compute_message_definition_hash(msg);
        let output = serde_json::json!({
            "name": msg.name,
            "module": msg.module,
            "source_file": msg.source_file,
            "doc": msg.doc,
            "fields": fields,
            "hash": hash,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    println!("{}", "Message Type Definition".green().bold());
    println!();
    println!("  {} {}", "Type:".cyan(), msg.name.white().bold());
    println!("  {} {}", "Module:".cyan(), msg.module);
    println!("  {} {}", "Source:".cyan(), msg.source_file.dimmed());

    if !msg.doc.is_empty() {
        println!();
        println!("  {}", "Description:".cyan());
        for line in msg.doc.lines() {
            println!("    {}", line);
        }
    }

    println!();
    println!("  {}", "Fields:".cyan());
    if msg.fields.is_empty() {
        println!("    {}", "(no public fields or unit struct)".dimmed());
    } else {
        for field in &msg.fields {
            println!(
                "    {} {}: {}",
                "".white(),
                field.name.white(),
                field.field_type.yellow()
            );
            if !field.doc.is_empty() {
                println!("      {}", field.doc.dimmed());
            }
        }
    }

    let hash = compute_message_definition_hash(msg);
    println!();
    println!("  {} {}", "Hash:".cyan(), hash.dimmed());

    Ok(())
}

/// Show message definition hash
pub fn message_hash(name: &str, json: bool) -> HorusResult<()> {
    let messages = discover_messages()?;
    let msg = resolve_message(&messages, name)?;
    let hash = compute_message_definition_hash(msg);

    if json {
        let output = serde_json::json!({
            "name": msg.name,
            "module": msg.module,
            "hash": hash,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
    } else {
        println!("{}", hash);
    }

    Ok(())
}

/// Discover all message types from source files
/// Every crate that ships HORUS message definitions, resolved from the
/// dependency graph rather than from a hard-coded crate name.
///
/// The universal IPC types live in `horus_types/src`, but that is only part of
/// the registry. The standard robotics messages (CmdVel, Imu, LaserScan,
/// Odometry, ...) live in `horus-robotics` and the transform messages
/// (TFMessage, StaticTransformStamped, TransformStamped) live in `horus-tf` —
/// both separate git dependencies, checked out by cargo under
/// `$CARGO_HOME/git/checkouts/<crate>-<hash>/<rev>/`.
///
/// This used to key on the literal directory prefix `horus-robotics-` *and* on
/// finding a `src/messages` **directory**, which missed two things at once:
/// any other linked message crate, and `horus-tf`, which ships the whole module
/// as a single `src/messages.rs` file. So `horus msg info TFMessage` said the
/// type did not exist, and `horus msg hash TransformStamped` silently printed
/// the hash of the *unrelated* `horus_types::math::TransformStamped`.
///
/// Discovery is now driven by the lockfile of the HORUS source tree, so it
/// spans whatever message crates are actually linked, and recognises a message
/// module by its shape (`src/messages/` **or** `src/messages.rs`).
///
/// Returns `(file, module)` pairs. A missing directory is simply skipped, so
/// this never turns an available message set into an error.
fn linked_message_files() -> Vec<(std::path::PathBuf, String)> {
    // An explicit HORUS_SOURCE_DIR is an override: resolve the linked crates
    // from *that* tree and nothing else, so a synthetic fixture root stays
    // exactly as synthetic as the caller made it. It used to disable this whole
    // function, which meant pointing the variable at a real HORUS checkout —
    // the documented use ("path to a HORUS checkout, used to resolve message
    // types") — reproduced the original bug verbatim:
    //
    //     $ HORUS_SOURCE_DIR=/path/to/horus horus msg info CmdVel
    //     Error: Message type 'CmdVel' not found
    //
    // A real checkout has a Cargo.lock naming its message crates; a fixture
    // directory does not, and so still contributes nothing.
    let explicit = std::env::var_os("HORUS_SOURCE_DIR").map(std::path::PathBuf::from);
    let source_root = match &explicit {
        Some(p) => Some(p.clone()),
        None => crate::commands::run::find_horus_source_dir().ok(),
    };
    linked_message_files_for(source_root.as_deref(), explicit.is_some())
}

/// The body of [`linked_message_files`], with the environment read out of it so
/// both branches are testable without mutating process state.
///
/// `explicit` is true when the caller pinned the tree with `HORUS_SOURCE_DIR`;
/// that suppresses the machine-wide fallback scan but — unlike before — not the
/// lockfile-driven discovery, which is the whole point of pointing the variable
/// at a real checkout.
fn linked_message_files_for(
    source_root: Option<&Path>,
    explicit: bool,
) -> Vec<(std::path::PathBuf, String)> {
    let mut out: Vec<(std::path::PathBuf, String)> = Vec::new();

    let mut crate_roots: Vec<std::path::PathBuf> = Vec::new();
    if let Some(root) = &source_root {
        for (name, rev) in git_packages_in_lock(&root.join("Cargo.lock")) {
            // A sibling development checkout wins over the cargo cache, for
            // anyone developing the two repos together.
            let dir = root
                .parent()
                .map(|p| p.join(&name))
                .filter(|p| p.join("src").is_dir())
                .or_else(|| git_checkout_dir(&name, &rev));
            let Some(dir) = dir else { continue };
            if crate_roots.contains(&dir) {
                continue;
            }
            crate_roots.push(dir.clone());
            out.extend(message_files_in_crate(&dir, &name));
        }
    }

    // Fallback for an install with no lockfile to consult: recognise a message
    // crate by its shape instead of by its name, so a user's own message crate
    // is as visible as HORUS's own.
    if out.is_empty() && !explicit {
        if let Some(checkouts) = cargo_home().map(|h| h.join("git").join("checkouts")) {
            let mut entries: Vec<std::path::PathBuf> = fs::read_dir(&checkouts)
                .into_iter()
                .flatten()
                .flatten()
                .map(|e| e.path())
                .collect();
            entries.sort();
            for entry in entries {
                let dir_name = entry
                    .file_name()
                    .and_then(|s| s.to_str())
                    .unwrap_or_default()
                    .to_string();
                // cargo names the directory "<crate>-<hash-of-url>".
                let crate_name = dir_name
                    .rsplit_once('-')
                    .map(|(n, _)| n.to_string())
                    .unwrap_or(dir_name);
                let mut revs: Vec<std::path::PathBuf> = fs::read_dir(&entry)
                    .into_iter()
                    .flatten()
                    .flatten()
                    .map(|e| e.path())
                    .collect();
                revs.sort();
                for rev in revs {
                    out.extend(message_files_in_crate(&rev, &crate_name));
                }
            }
        }
    }

    out
}

/// The `.rs` files that make up a crate's message module, with the module label
/// each one should be reported under.
///
/// Two shapes are accepted, because both are in use by crates HORUS links:
/// `src/messages/<module>.rs` (horus-robotics) and a single flat
/// `src/messages.rs` (horus-tf). Only the first was recognised, which is why
/// every horus-tf message type was missing from the registry.
fn message_files_in_crate(root: &Path, crate_name: &str) -> Vec<(std::path::PathBuf, String)> {
    let mut out = Vec::new();

    let dir = root.join("src").join("messages");
    if dir.is_dir() {
        let mut files: Vec<std::path::PathBuf> = fs::read_dir(&dir)
            .into_iter()
            .flatten()
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|e| e == "rs"))
            .collect();
        files.sort();
        for file in files {
            let stem = file
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or_default()
                .to_string();
            if stem == "mod" {
                continue;
            }
            out.push((file, stem));
        }
    }

    let single = root.join("src").join("messages.rs");
    if single.is_file() {
        out.push((single, crate_module_label(crate_name)));
    }

    out
}

/// Module label for a crate that ships one flat `messages.rs`.
///
/// The file stem would be the useless label "messages" for every such crate, so
/// the crate names the module instead: `horus-tf` -> `tf`, which is also what
/// disambiguates `tf::TransformStamped` from `math::TransformStamped`.
fn crate_module_label(crate_name: &str) -> String {
    let base = crate_name
        .strip_prefix("horus-")
        .or_else(|| crate_name.strip_prefix("horus_"))
        .filter(|s| !s.is_empty())
        .unwrap_or(crate_name);
    base.replace('-', "_")
}

/// Git dependencies recorded in a `Cargo.lock`, as `(crate name, revision)`.
fn git_packages_in_lock(lock: &Path) -> Vec<(String, String)> {
    let Ok(text) = fs::read_to_string(lock) else {
        return Vec::new();
    };

    let mut out = Vec::new();
    let mut name: Option<String> = None;
    for line in text.lines() {
        let line = line.trim();
        if line == "[[package]]" {
            name = None;
        } else if let Some(rest) = line.strip_prefix("name = ") {
            name = Some(rest.trim().trim_matches('"').to_string());
        } else if let Some(rest) = line.strip_prefix("source = ") {
            // "git+https://host/repo.git?rev=<rev>#<resolved sha>"
            let src = rest.trim().trim_matches('"');
            if !src.starts_with("git+") {
                continue;
            }
            let Some((_, sha)) = src.split_once('#') else {
                continue;
            };
            if let Some(name) = name.clone() {
                out.push((name, sha.to_string()));
            }
        }
    }
    out
}

/// The cargo checkout directory holding `<name>` at `<rev>`, if cargo has one.
fn git_checkout_dir(name: &str, rev: &str) -> Option<std::path::PathBuf> {
    let checkouts = cargo_home()?.join("git").join("checkouts");
    let prefix = format!("{name}-");
    for entry in fs::read_dir(&checkouts).ok()?.flatten() {
        let dir_name = entry.file_name();
        let dir_name = dir_name.to_string_lossy();
        if !dir_name.starts_with(&prefix) {
            continue;
        }
        // Each checkout holds one directory per revision, named with the
        // abbreviated sha.
        let Ok(revs) = fs::read_dir(entry.path()) else {
            continue;
        };
        for rev_dir in revs.flatten() {
            let short = rev_dir.file_name();
            let short = short.to_string_lossy();
            if !short.is_empty() && rev.starts_with(short.as_ref()) && rev_dir.path().is_dir() {
                return Some(rev_dir.path());
            }
        }
    }
    None
}

/// Cargo's home directory, without pulling in a dependency for one lookup.
fn cargo_home() -> Option<std::path::PathBuf> {
    std::env::var_os("CARGO_HOME")
        .map(std::path::PathBuf::from)
        .or_else(|| dirs_next_home().map(|h| h.join(".cargo")))
}

/// Home directory, without pulling in a dependency for one lookup.
fn dirs_next_home() -> Option<std::path::PathBuf> {
    std::env::var_os("HOME").map(std::path::PathBuf::from)
}

/// Messages declared in this project's `msgs/*.hmsg`, if we are in a project.
///
/// Silent when there is no project or no `msgs/` directory — `horus msg list`
/// is used outside projects and must keep working. Parse failures are silent
/// here too, because this is the *listing* path; `horus msg gen` is where a
/// broken definition gets a diagnostic, and reporting it twice from two
/// commands with different wording helps nobody.
fn local_hmsg_messages() -> Vec<MessageInfo> {
    let Ok((manifest, root)) = crate::manifest::HorusManifest::find_and_load() else {
        return Vec::new();
    };
    let msgs_dir = root.join("msgs");
    if !msgs_dir.is_dir() {
        return Vec::new();
    }

    let mut files: Vec<std::path::PathBuf> = std::fs::read_dir(&msgs_dir)
        .into_iter()
        .flatten()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "hmsg"))
        .collect();
    files.sort();

    // Every file, then one resolve over the lot. Resolving file by file would
    // be wrong in the same way it was wrong in `generate_messages`: a
    // reference is only resolvable once the whole package is known, and an
    // unresolved reference renders as the empty string. `horus msg hash
    // WeatherData` then printed the hash of
    // `WeatherData|temperature:f32|wind:|ts:u64` — a number the runtime never
    // computes, for the one command whose purpose is to be compared against a
    // layout-mismatch error.
    let mut messages = Vec::new();
    for file in files {
        let Ok(text) = std::fs::read_to_string(&file) else {
            continue;
        };
        let Ok(defs) = crate::msgspec::parse::parse_file(&text, &file) else {
            continue;
        };
        messages.extend(defs);
    }

    let mut pkg = crate::msgspec::Package {
        name: manifest.package.name.clone(),
        messages,
    };
    // Still the listing path, so a definition that does not resolve is skipped
    // rather than reported — `horus msg gen` is where it gets a diagnostic.
    // Skipping is not the same as listing it with a truncated hash.
    if crate::msgspec::resolve::resolve(&mut pkg).is_err() {
        return Vec::new();
    }

    pkg.messages
        .iter()
        .map(|d| MessageInfo {
            name: d.name.clone(),
            module: manifest.package.name.clone(),
            fields: d
                .fields
                .iter()
                .map(|f| FieldInfo {
                    name: f.name.clone(),
                    field_type: crate::msgspec::canonical::render_rust(&f.ty),
                    doc: f.doc.join(" "),
                })
                .collect(),
            doc: d.doc.join(" "),
            source_file: d.src.display().to_string(),
        })
        .collect()
}

pub(crate) fn discover_messages() -> HorusResult<Vec<MessageInfo>> {
    // Find the message type definitions.
    //
    // These used to live in `horus_library/messages`, but horus_library was
    // removed (commit 7b430279, "decomposed into horus_types + horus-tf +
    // horus-robotics"). Every search path here pointed at that deleted
    // directory, so `horus msg list/info/hash` could not succeed on any
    // install — and the error told the user to set HORUS_SOURCE_DIR, which
    // could not help either. The universal IPC types now live in
    // `horus_types/src`.
    let mut search_paths: Vec<std::path::PathBuf> = Vec::new();

    // 1. HORUS_SOURCE_DIR env var (explicit override, kept for compatibility)
    if let Ok(source_dir) = std::env::var("HORUS_SOURCE_DIR") {
        search_paths.push(Path::new(&source_dir).join("horus_types/src"));
    }

    // 2. The canonical source-tree resolution used by the rest of the CLI.
    //    This is what makes the command work on a normal install, where the
    //    source is cached at ~/.horus/cache/horus@<version> (it also honors
    //    $HORUS_SOURCE and the well-known checkout locations).
    if let Ok(source_root) = crate::commands::run::find_horus_source_dir() {
        search_paths.push(source_root.join("horus_types/src"));
    }

    // 3. Relative to the executable, for running out of a build tree.
    if let Ok(exe) = std::env::current_exe() {
        if let Some(exe_dir) = exe.parent() {
            for ancestor in [exe_dir, exe_dir.parent().unwrap_or(exe_dir)] {
                search_paths.push(ancestor.join("horus_types/src"));
                search_paths.push(ancestor.join("../horus_types/src"));
                search_paths.push(ancestor.join("../../horus_types/src"));
            }
        }
    }

    // The universal types are only half the picture. Every standard robotics
    // message — CmdVel, Imu, LaserScan, Odometry — lives in `horus-robotics`
    // and every transform message in `horus-tf`, both separate git
    // dependencies, and this scan never looked there. So while a live topic
    // reported its type correctly:
    //
    //     $ horus topic info cmd_vel      ->  Message Type: CmdVel
    //     $ horus msg info CmdVel         ->  Message type 'CmdVel' not found
    //
    // the introspection command could not describe the very types users
    // publish. `linked_message_files()` collects every linked message crate.

    // The project's own messages, when there are any.
    //
    // Parsed from `msgs/*.hmsg` with the strict parser rather than scraped out
    // of the generated Rust: the `.hmsg` file is the definition, and going
    // through the same parser means `horus msg hash` and the generated
    // artifacts cannot compute the value two different ways.
    let mut messages: Vec<MessageInfo> = local_hmsg_messages();

    // `search_paths` are alternative locations for the *same* directory
    // (horus_types/src), so they stay first-match-wins. The linked message
    // crates are a genuinely different source and are additive.
    let mut message_files: Vec<(std::path::PathBuf, String)> = Vec::new();
    // Whether any message *source location* was found. Distinct from
    // `message_files.is_empty()`: a located-but-empty directory is a project
    // with no messages, not a broken installation.
    let mut located_a_source = false;
    if let Some(found) = search_paths.iter().find(|p| p.is_dir()) {
        located_a_source = true;
        let mut files: Vec<std::path::PathBuf> = fs::read_dir(found)
            .map_err(HorusError::Io)?
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().map(|e| e == "rs").unwrap_or(false))
            .collect();
        files.sort();
        for file in files {
            let stem = file
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("unknown")
                .to_string();
            if stem == "mod" {
                continue;
            }
            message_files.push((file, stem));
        }
    }
    let linked = linked_message_files();
    located_a_source |= !linked.is_empty();
    message_files.extend(linked);
    message_files.dedup();

    if !located_a_source {
        return Err(HorusError::Config(ConfigError::Other(
            "Could not find the HORUS message definitions (horus_types/src).\n  \
             Re-run the installer, or point HORUS_SOURCE at your HORUS source tree,\n  \
             e.g.: export HORUS_SOURCE=/path/to/horus"
                .to_string(),
        )));
    }

    for (path, module) in &message_files {
        if let Ok(content) = fs::read_to_string(path) {
            let file_messages =
                parse_messages_from_source(&content, module, path.to_string_lossy().to_string());
            messages.extend(file_messages);
        }
    }

    // Sort by module and name
    messages.sort_by(|a, b| match a.module.cmp(&b.module) {
        std::cmp::Ordering::Equal => a.name.cmp(&b.name),
        other => other,
    });
    // A type can appear in more than one source tree (a local checkout and the
    // cargo git cache, say); report it once.
    messages.dedup_by(|a, b| a.module == b.module && a.name == b.name);

    Ok(messages)
}

/// Parse message types from source code
fn parse_messages_from_source(source: &str, module: &str, source_file: String) -> Vec<MessageInfo> {
    let mut messages = Vec::new();
    let lines: Vec<&str> = source.lines().collect();
    let mut i = 0;

    while i < lines.len() {
        let line = lines[i].trim();

        // Look for pub struct definitions
        if line.starts_with("pub struct ") || line.starts_with("#[derive") {
            // Collect doc comments before the struct
            let mut doc_lines = Vec::new();
            let mut j = i;

            // Go back to find doc comments
            while j > 0 {
                let prev_line = lines[j - 1].trim();
                if prev_line.starts_with("///") {
                    doc_lines.insert(0, prev_line.trim_start_matches("///").trim());
                    j -= 1;
                } else if prev_line.is_empty() || prev_line.starts_with("#[") {
                    j -= 1;
                } else {
                    break;
                }
            }

            // Skip derive attributes to find struct line
            let mut struct_line_idx = i;
            while struct_line_idx < lines.len()
                && !lines[struct_line_idx].trim().starts_with("pub struct ")
            {
                struct_line_idx += 1;
            }

            if struct_line_idx >= lines.len() {
                i += 1;
                continue;
            }

            let struct_line = lines[struct_line_idx].trim();

            // Extract struct name
            if let Some(name) = extract_struct_name(struct_line) {
                // Parse fields
                let mut fields = Vec::new();
                let mut field_idx = struct_line_idx + 1;
                let mut in_struct = struct_line.contains('{');
                let mut brace_count = if in_struct { 1 } else { 0 };

                // Check if it's a unit struct or tuple struct
                if struct_line.ends_with(';') || struct_line.contains('(') {
                    // Unit struct or tuple struct - no named fields
                } else {
                    // Named struct - parse fields
                    while field_idx < lines.len() && (in_struct || brace_count == 0) {
                        let field_line = lines[field_idx].trim();

                        if field_line.contains('{') {
                            in_struct = true;
                            brace_count += field_line.matches('{').count();
                        }
                        if field_line.contains('}') {
                            brace_count -= field_line.matches('}').count();
                            if brace_count == 0 {
                                break;
                            }
                        }

                        // Parse field
                        if let Some(field) = parse_field(field_line) {
                            // Get field doc
                            let mut field_doc = String::new();
                            if field_idx > 0 {
                                let prev = lines[field_idx - 1].trim();
                                if prev.starts_with("///") {
                                    field_doc = prev.trim_start_matches("///").trim().to_string();
                                }
                            }
                            fields.push(FieldInfo {
                                name: field.0,
                                field_type: field.1,
                                doc: field_doc,
                            });
                        }

                        field_idx += 1;
                    }
                }

                messages.push(MessageInfo {
                    name: name.to_string(),
                    module: module.to_string(),
                    fields,
                    doc: doc_lines.join("\n"),
                    source_file: source_file.clone(),
                });

                i = field_idx;
                continue;
            }
        }

        i += 1;
    }

    messages
}

/// Extract struct name from "pub struct Foo" or "pub struct Foo {"
fn extract_struct_name(line: &str) -> Option<&str> {
    let line = line.trim_start_matches("pub struct ").trim();
    // Handle generics like "Foo<T>", "Foo {", "Foo(f64);" and the unit struct
    // "Foo;". The terminator of a unit struct used to be missing from this
    // list, so `pub struct PauseRequest;` was registered under the name
    // "PauseRequest;" — `horus msg list` printed the semicolon, `horus msg
    // info PauseRequest` reported the type did not exist, and only the
    // unspellable `horus msg info 'PauseRequest;'` worked.
    let name = line
        .split(|c: char| c == '<' || c == '{' || c == '(' || c == ';' || c.is_whitespace())
        .next()?;
    if name.is_empty() || !name.chars().next()?.is_uppercase() {
        return None;
    }
    Some(name)
}

/// Parse a field line like "pub name: Type," or "name: Type,"
fn parse_field(line: &str) -> Option<(String, String)> {
    let line = line.trim();

    // Skip non-field lines
    if line.is_empty()
        || line.starts_with("//")
        || line.starts_with("#[")
        || line == "{"
        || line == "}"
        || line.starts_with("pub fn")
        || line.starts_with("fn ")
        || line.starts_with("impl")
    {
        return None;
    }

    // Handle "pub name: Type," or "name: Type,"
    let line = line.trim_start_matches("pub ");

    if !line.contains(':') {
        return None;
    }

    let parts: Vec<&str> = line.splitn(2, ':').collect();
    if parts.len() != 2 {
        return None;
    }

    let name = parts[0].trim().to_string();
    // Cut a trailing line comment before the comma. Without this,
    //
    //     pub linear: f32,  // m/s forward velocity
    //
    // yielded the *type* "f32,  // m/s forward velocity", which then went into
    // the layout hash — so `horus msg hash CmdVel` printed 0xe9574bd4 while the
    // runtime computes 0x3836b786 over `CmdVel|timestamp_ns:u64|linear:f32|angular:f32`.
    // The command exists to be compared against a layout-mismatch error, and for
    // every commented message type it printed a number that error never shows.
    //
    // A `//` cannot appear inside a real Rust field type, so splitting on it is
    // safe here.
    let field_type = parts[1]
        .split("//")
        .next()
        .unwrap_or("")
        .trim()
        .trim_end_matches(',')
        .trim()
        .to_string();

    // Skip if it looks like a method signature
    if field_type.contains("->") || field_type.contains("fn(") {
        return None;
    }

    // Padding is not decoration, it is layout.
    //
    // This used to `return None` for any field whose name begins with `_`,
    // called "skip padding fields". The canonical form this feeds is the layout
    // identity, so dropping `_pad: [u8; 6]` from `Clock` made
    // `horus msg hash Clock` print the hash of a struct that does not exist —
    // and the `message!` macro, which stringifies every field it is given,
    // computes the hash *with* the padding. Two numbers for one type, from the
    // two places a developer compares, which is the defect this whole canonical
    // form was introduced to end.
    //
    // It also erased a real distinction: `A { x: u8, _pad: [u8; 7], y: u64 }`
    // and `A { x: u8, y: u64 }` hashed identically while having different
    // layouts, so a peer built against one would be accepted by the other.
    //
    // `horus msg info` now lists padding fields too. That is the point: a
    // developer checking a Rust struct against a C++ one needs to see them.

    Some((name, field_type))
}

/// Canonical string a message's layout hash is computed over.
///
/// Must stay byte-identical to what the `message!` macro concatenates for
/// `LAYOUT_HASH`, or the two hashes describe the same type and disagree.
///
/// Shape: `Name|field:Type|field:Type`
fn canonical_message_form(name: &str, fields: &[(String, String)]) -> String {
    let mut canonical = String::from(name);
    for (field_name, field_type) in fields {
        canonical.push('|');
        canonical.push_str(field_name);
        canonical.push(':');
        canonical.push_str(field_type);
    }
    canonical
}

/// FNV-1a, matching `horus_core::communication::topic::const_fnv1a`.
fn fnv1a(bytes: &[u8]) -> u32 {
    let mut hash: u32 = 2166136261;
    for &byte in bytes {
        hash ^= byte as u32;
        hash = hash.wrapping_mul(16777619);
    }
    hash
}

/// Compute a definition hash for a message type.
///
/// Two things were wrong with the previous implementation.
///
/// It used `DefaultHasher`, whose output std explicitly does not guarantee
/// between Rust releases: "the internal algorithm is not specified, and so it
/// and its hashes should not be relied upon over releases." A definition hash
/// that changes when you upgrade your toolchain reports every message as
/// modified, which is the one thing it exists to detect.
///
/// And it hashed a different canonical form from the one the runtime uses, so
/// `horus msg hash Pose` and the layout-mismatch error raised by
/// `Topic::new_checked` printed different numbers for the same type — actively
/// misleading for the one task the command is for.
///
/// Now: FNV-1a over `Name|field:Type|…`, byte-identical to the `LAYOUT_HASH`
/// the `message!` macro emits, so the number the CLI prints is the number the
/// runtime compares.
fn compute_message_definition_hash(msg: &MessageInfo) -> String {
    let fields: Vec<(String, String)> = msg
        .fields
        .iter()
        .map(|f| (f.name.clone(), f.field_type.clone()))
        .collect();

    format!(
        "{:#010x}",
        fnv1a(canonical_message_form(&msg.name, &fields).as_bytes())
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_struct_name_simple() {
        assert_eq!(extract_struct_name("pub struct Twist {"), Some("Twist"));
    }

    #[test]
    fn extract_struct_name_generic() {
        assert_eq!(extract_struct_name("pub struct Vec3<T> {"), Some("Vec3"));
    }

    /// A unit struct's name must not carry its terminating semicolon.
    ///
    /// This test used to assert `starts_with("Empty")` and *document* the
    /// semicolon as expected, so it passed while the parser produced the name
    /// "Empty;". That is a name no user can type: `horus msg list` printed
    /// "PauseRequest;", `horus msg info PauseRequest` answered "Message type
    /// 'PauseRequest' not found", and only `horus msg info 'PauseRequest;'`
    /// worked. horus-robotics ships four such types (PauseRequest,
    /// ResumeRequest, ResetRequest, GetStateRequest).
    #[test]
    fn extract_struct_name_unit() {
        assert_eq!(extract_struct_name("pub struct Empty;"), Some("Empty"));
    }

    /// The real witnesses, verbatim from horus-robotics
    /// `src/messages/simulation.rs`.
    #[test]
    fn unit_structs_from_robotics_sources_parse_to_typeable_names() {
        let source = "\
/// Pause the simulation
#[derive(Debug, Clone, Copy)]
pub struct PauseRequest;

/// Resume the simulation
#[derive(Debug, Clone, Copy)]
pub struct ResumeRequest;
";
        let parsed = parse_messages_from_source(source, "simulation", "simulation.rs".into());
        let names: Vec<&str> = parsed.iter().map(|m| m.name.as_str()).collect();
        assert_eq!(names, vec!["PauseRequest", "ResumeRequest"]);
    }

    #[test]
    fn extract_struct_name_tuple() {
        assert_eq!(
            extract_struct_name("pub struct Wrapper(f64);"),
            Some("Wrapper")
        );
    }

    #[test]
    fn extract_struct_name_lowercase_rejected() {
        assert_eq!(extract_struct_name("pub struct lowercase {"), None);
    }

    #[test]
    fn parse_field_basic() {
        let result = parse_field("pub x: f64,");
        assert_eq!(result, Some(("x".into(), "f64".into())));
    }

    #[test]
    fn parse_field_no_pub() {
        let result = parse_field("y: f32,");
        assert_eq!(result, Some(("y".into(), "f32".into())));
    }

    #[test]
    fn parse_field_complex_type() {
        let result = parse_field("pub data: Vec<u8>,");
        assert_eq!(result, Some(("data".into(), "Vec<u8>".into())));
    }

    #[test]
    fn parse_field_skips_comment() {
        assert!(parse_field("// this is a comment").is_none());
    }

    #[test]
    fn parse_field_skips_attribute() {
        assert!(parse_field("#[serde(skip)]").is_none());
    }

    #[test]
    fn parse_field_skips_method() {
        assert!(parse_field("pub fn new() -> Self {").is_none());
    }

    /// This test used to assert the opposite, and it was wrong.
    ///
    /// Skipping `_`-prefixed fields was called "skip padding fields", and it
    /// silently changed what the canonical form describes: `horus msg hash
    /// Clock` printed the hash of a `Clock` without its `_pad: [u8; 6]`, while
    /// the `message!` macro — which stringifies every field it is handed —
    /// hashes the padding in. Two numbers for one type, from the two places a
    /// developer holds up against each other.
    #[test]
    fn parse_field_keeps_padding() {
        assert_eq!(
            parse_field("_pad: [u8; 3],"),
            Some(("_pad".into(), "[u8; 3]".into())),
            "padding occupies bytes on the wire, so it is part of the layout"
        );
    }

    #[test]
    fn parse_field_skips_fn_type() {
        assert!(parse_field("pub callback: fn(u32) -> bool,").is_none());
    }

    #[test]
    fn parse_messages_from_source_simple() {
        let source = r#"
/// A 3D vector
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
"#;
        let messages = parse_messages_from_source(source, "math", "math.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Vec3");
        assert_eq!(messages[0].module, "math");
        assert_eq!(messages[0].fields.len(), 3);
        assert_eq!(messages[0].fields[0].name, "x");
        assert!(messages[0].doc.contains("3D vector"));
    }

    #[test]
    fn parse_messages_from_source_unit_struct() {
        let source = "pub struct Empty;\n";
        let messages = parse_messages_from_source(source, "test", "test.rs".into());
        assert_eq!(messages.len(), 1);
        // extract_struct_name returns "Empty;" for unit structs (';' not in split chars)
        assert!(messages[0].name.starts_with("Empty"));
        assert!(messages[0].fields.is_empty());
    }

    #[test]
    fn parse_messages_multiple_structs() {
        let source = r#"
pub struct A {
    pub x: f64,
}

pub struct B {
    pub y: i32,
    pub z: i32,
}
"#;
        let messages = parse_messages_from_source(source, "mod", "mod.rs".into());
        assert_eq!(messages.len(), 2);
        assert_eq!(messages[0].name, "A");
        assert_eq!(messages[1].name, "B");
        assert_eq!(messages[1].fields.len(), 2);
    }

    #[test]
    fn compute_hash_deterministic() {
        let msg = MessageInfo {
            name: "Twist".into(),
            module: "control".into(),
            fields: vec![
                FieldInfo {
                    name: "linear".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
                FieldInfo {
                    name: "angular".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
            ],
            doc: String::new(),
            source_file: String::new(),
        };
        let h1 = compute_message_definition_hash(&msg);
        let h2 = compute_message_definition_hash(&msg);
        assert_eq!(h1, h2, "hash must be deterministic");
        assert_eq!(h1.len(), 10, "hash should be 0x + 8 hex chars");
    }

    #[test]
    fn compute_hash_changes_with_field() {
        let msg1 = MessageInfo {
            name: "Foo".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "a".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        let msg2 = MessageInfo {
            name: "Foo".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "b".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&msg1),
            compute_message_definition_hash(&msg2),
            "different fields should produce different hashes"
        );
    }

    // =========================================================================
    // extract_struct_name — additional edge cases
    // =========================================================================

    #[test]
    fn extract_struct_name_empty_input() {
        // After stripping "pub struct ", the remainder is empty
        assert_eq!(extract_struct_name("pub struct "), None);
    }

    #[test]
    fn extract_struct_name_whitespace_after_prefix() {
        // Leading whitespace in input line; function strips "pub struct " literally
        // so "  pub struct Foo {" won't match because it starts with spaces
        let result = extract_struct_name("  pub struct Foo {");
        // The function does trim_start_matches("pub struct ") which is char-level,
        // so "  pub struct Foo {" -> "  Foo {" after stripping matching prefix chars.
        // Actually trim_start_matches matches the full pattern — if line doesn't start with
        // "pub struct " it returns the original. Let's verify.
        // Actually, the function receives the already-trimmed line from the caller,
        // but we test the function directly here.
        assert!(result.is_none() || result.is_some());
        // The key point: function should not panic
    }

    #[test]
    fn extract_struct_name_with_lifetime() {
        assert_eq!(extract_struct_name("pub struct Ref<'a> {"), Some("Ref"));
    }

    #[test]
    fn extract_struct_name_with_multiple_generics() {
        assert_eq!(extract_struct_name("pub struct Map<K, V> {"), Some("Map"));
    }

    #[test]
    fn extract_struct_name_trailing_space() {
        assert_eq!(extract_struct_name("pub struct Point  {"), Some("Point"));
    }

    #[test]
    fn extract_struct_name_where_clause() {
        // struct with where clause
        assert_eq!(
            extract_struct_name("pub struct Container<T> where T: Clone {"),
            Some("Container")
        );
    }

    #[test]
    fn extract_struct_name_numeric_start_rejected() {
        // Name starting with a digit is not uppercase
        assert_eq!(extract_struct_name("pub struct 3DPoint {"), None);
    }

    #[test]
    fn extract_struct_name_underscore_start_rejected() {
        // Name starting with underscore — first char is '_', not uppercase
        assert_eq!(extract_struct_name("pub struct _Internal {"), None);
    }

    #[test]
    fn extract_struct_name_single_char() {
        assert_eq!(extract_struct_name("pub struct X {"), Some("X"));
    }

    // =========================================================================
    // parse_field — additional edge cases
    // =========================================================================

    #[test]
    fn parse_field_option_type() {
        let result = parse_field("pub name: Option<String>,");
        assert_eq!(result, Some(("name".into(), "Option<String>".into())));
    }

    #[test]
    fn parse_field_hashmap_type() {
        let result = parse_field("pub metadata: HashMap<String, String>,");
        assert_eq!(
            result,
            Some(("metadata".into(), "HashMap<String, String>".into()))
        );
    }

    #[test]
    fn parse_field_nested_generic() {
        let result = parse_field("pub data: Vec<Option<f64>>,");
        assert_eq!(result, Some(("data".into(), "Vec<Option<f64>>".into())));
    }

    #[test]
    fn parse_field_array_type() {
        let result = parse_field("pub buf: [u8; 256],");
        assert_eq!(result, Some(("buf".into(), "[u8; 256]".into())));
    }

    #[test]
    fn parse_field_no_trailing_comma() {
        // Last field in struct may not have trailing comma
        let result = parse_field("pub z: f64");
        assert_eq!(result, Some(("z".into(), "f64".into())));
    }

    #[test]
    fn parse_field_empty_line() {
        assert!(parse_field("").is_none());
    }

    #[test]
    fn parse_field_whitespace_only() {
        assert!(parse_field("   ").is_none());
    }

    #[test]
    fn parse_field_open_brace_only() {
        assert!(parse_field("{").is_none());
    }

    #[test]
    fn parse_field_close_brace_only() {
        assert!(parse_field("}").is_none());
    }

    #[test]
    fn parse_field_doc_comment() {
        assert!(parse_field("/// field documentation").is_none());
    }

    #[test]
    fn parse_field_impl_line() {
        assert!(parse_field("impl Foo {").is_none());
    }

    #[test]
    fn parse_field_fn_line() {
        assert!(parse_field("fn helper() -> u32 {").is_none());
    }

    #[test]
    fn parse_field_no_colon() {
        assert!(parse_field("pub some_thing").is_none());
    }

    #[test]
    fn parse_field_arrow_in_type() {
        // "data: Box<dyn Fn() -> bool>" contains "->" so it's skipped
        assert!(parse_field("pub handler: Box<dyn Fn() -> bool>,").is_none());
    }

    #[test]
    fn parse_field_string_type() {
        let result = parse_field("pub label: String,");
        assert_eq!(result, Some(("label".into(), "String".into())));
    }

    #[test]
    fn parse_field_bool_type() {
        let result = parse_field("pub enabled: bool,");
        assert_eq!(result, Some(("enabled".into(), "bool".into())));
    }

    #[test]
    fn parse_field_tuple_type() {
        let result = parse_field("pub pos: (f64, f64, f64),");
        assert_eq!(result, Some(("pos".into(), "(f64, f64, f64)".into())));
    }

    /// A public reserved field is no more skippable than a private one: both
    /// occupy bytes, and a layout identity that leaves them out cannot tell
    /// `A { x: u8, _reserved: u8 }` from `A { x: u8 }`.
    #[test]
    fn parse_field_keeps_a_public_reserved_field() {
        assert_eq!(
            parse_field("pub _reserved: u8,"),
            Some(("_reserved".into(), "u8".into()))
        );
    }

    #[test]
    fn parse_field_with_leading_whitespace() {
        let result = parse_field("    pub x: f64,");
        assert_eq!(result, Some(("x".into(), "f64".into())));
    }

    // =========================================================================
    // parse_messages_from_source — additional edge cases
    // =========================================================================

    #[test]
    fn parse_messages_empty_source() {
        let messages = parse_messages_from_source("", "empty", "empty.rs".into());
        assert!(messages.is_empty());
    }

    #[test]
    fn parse_messages_no_structs() {
        let source = r#"
fn helper() -> u32 { 42 }
const VALUE: f64 = 2.75;
"#;
        let messages = parse_messages_from_source(source, "util", "util.rs".into());
        assert!(messages.is_empty());
    }

    #[test]
    fn parse_messages_struct_with_derive() {
        let source = r#"
#[derive(Debug, Clone)]
pub struct Velocity {
    pub linear: f64,
    pub angular: f64,
}
"#;
        let messages = parse_messages_from_source(source, "control", "control.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Velocity");
        assert_eq!(messages[0].fields.len(), 2);
        assert_eq!(messages[0].fields[0].name, "linear");
        assert_eq!(messages[0].fields[1].name, "angular");
    }

    #[test]
    fn parse_messages_struct_with_multiple_derives() {
        let source = r#"
#[derive(Debug)]
#[derive(Clone, PartialEq)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}
"#;
        let messages = parse_messages_from_source(source, "nav", "nav.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Pose");
        assert_eq!(messages[0].fields.len(), 3);
    }

    #[test]
    fn parse_messages_multiline_doc() {
        let source = r#"
/// First line of doc.
/// Second line of doc.
/// Third line.
pub struct Documented {
    pub val: u32,
}
"#;
        let messages = parse_messages_from_source(source, "docs", "docs.rs".into());
        assert_eq!(messages.len(), 1);
        assert!(messages[0].doc.contains("First line"));
        assert!(messages[0].doc.contains("Second line"));
        assert!(messages[0].doc.contains("Third line"));
    }

    #[test]
    fn parse_messages_field_docs() {
        let source = r#"
pub struct Stamped {
    /// Timestamp in nanoseconds
    pub timestamp: u64,
    /// Frame identifier
    pub frame_id: String,
}
"#;
        let messages = parse_messages_from_source(source, "core", "core.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].fields.len(), 2);
        assert!(messages[0].fields[0].doc.contains("Timestamp"));
        assert!(messages[0].fields[1].doc.contains("Frame identifier"));
    }

    #[test]
    fn parse_messages_tuple_struct() {
        let source = "pub struct Wrapper(pub f64);\n";
        let messages = parse_messages_from_source(source, "wrap", "wrap.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Wrapper");
        // Tuple struct has no named fields
        assert!(messages[0].fields.is_empty());
    }

    #[test]
    fn parse_messages_struct_with_comments_between_fields() {
        let source = r#"
pub struct Mixed {
    pub a: f64,
    // internal comment
    pub b: f64,
}
"#;
        let messages = parse_messages_from_source(source, "mix", "mix.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].fields.len(), 2);
    }

    #[test]
    fn parse_messages_struct_with_attributes_on_fields() {
        let source = r#"
pub struct Config {
    #[serde(default)]
    pub enabled: bool,
    pub rate: f64,
}
"#;
        let messages = parse_messages_from_source(source, "cfg", "cfg.rs".into());
        assert_eq!(messages.len(), 1);
        // The #[serde(default)] line is skipped by parse_field
        // Only actual field lines are parsed
        assert_eq!(messages[0].fields.len(), 2);
    }

    #[test]
    fn parse_messages_preserves_source_file() {
        let source = "pub struct Foo { pub x: i32, }\n";
        let messages = parse_messages_from_source(source, "test", "/some/path/test.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].source_file, "/some/path/test.rs");
    }

    #[test]
    fn parse_messages_preserves_module() {
        let source = "pub struct Bar { pub y: u8, }\n";
        let messages = parse_messages_from_source(source, "sensors", "sensors.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].module, "sensors");
    }

    #[test]
    fn parse_messages_struct_with_no_fields() {
        let source = r#"
pub struct Marker {
}
"#;
        let messages = parse_messages_from_source(source, "tag", "tag.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Marker");
        assert!(messages[0].fields.is_empty());
    }

    #[test]
    fn parse_messages_ignores_private_struct() {
        let source = r#"
struct Private {
    x: f64,
}
pub struct Public {
    pub y: f64,
}
"#;
        let messages = parse_messages_from_source(source, "vis", "vis.rs".into());
        // Only "pub struct" is recognized
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Public");
    }

    #[test]
    fn parse_messages_complex_field_types() {
        let source = r#"
pub struct Complex {
    pub data: Vec<u8>,
    pub lookup: HashMap<String, Vec<f64>>,
    pub optional: Option<String>,
    pub nested: Vec<Option<(f64, f64)>>,
}
"#;
        let messages = parse_messages_from_source(source, "complex", "complex.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].fields.len(), 4);
        assert_eq!(messages[0].fields[0].field_type, "Vec<u8>");
        assert_eq!(
            messages[0].fields[1].field_type,
            "HashMap<String, Vec<f64>>"
        );
        assert_eq!(messages[0].fields[2].field_type, "Option<String>");
        assert_eq!(messages[0].fields[3].field_type, "Vec<Option<(f64, f64)>>");
    }

    /// Padding appears in the field list, in its declared position.
    ///
    /// The position matters as much as the presence: the canonical form is
    /// ordered, so a `_pad` recorded in the wrong place would describe a
    /// different layout just as surely as one left out.
    #[test]
    fn parse_messages_struct_keeps_padding_fields_in_place() {
        let source = r#"
pub struct Padded {
    pub value: f64,
    _pad: [u8; 4],
    pub flag: bool,
}
"#;
        let messages = parse_messages_from_source(source, "pad", "pad.rs".into());
        assert_eq!(messages.len(), 1);
        let names: Vec<&str> = messages[0].fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, vec!["value", "_pad", "flag"]);
        assert_eq!(messages[0].fields[1].field_type, "[u8; 4]");
    }

    #[test]
    fn parse_messages_doc_with_blank_lines_and_attributes() {
        let source = r#"
/// Doc line one

#[derive(Debug)]
pub struct WithGap {
    pub x: f64,
}
"#;
        let messages = parse_messages_from_source(source, "gap", "gap.rs".into());
        assert_eq!(messages.len(), 1);
        // The parser scans backwards skipping blank and attribute lines
        assert!(messages[0].doc.contains("Doc line one"));
    }

    #[test]
    fn parse_messages_struct_on_same_line_as_brace() {
        let source = "pub struct Inline { pub a: i32, pub b: i32, }\n";
        let messages = parse_messages_from_source(source, "inl", "inl.rs".into());
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].name, "Inline");
        // All on one line — "pub a: i32, pub b: i32, }" is the next portion
        // The parser sees '{' and '}' on the same line, so brace_count goes to 1 then 0
        // Fields on the struct line itself won't be parsed because field_idx starts at
        // struct_line_idx + 1. So 0 fields for single-line struct bodies.
    }

    // =========================================================================
    // compute_message_definition_hash — additional edge cases
    // =========================================================================

    #[test]
    fn compute_hash_no_fields() {
        let msg = MessageInfo {
            name: "Empty".into(),
            module: "test".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        let hash = compute_message_definition_hash(&msg);
        // "0x" + 8 hex digits: a 32-bit FNV-1a, the same width and encoding the
        // runtime prints in a layout-mismatch error, so the two can be compared
        // by eye.
        assert_eq!(hash.len(), 10, "expected 0x + 8 hex digits, got {hash}");
        assert!(hash.starts_with("0x"));
        assert!(hash[2..].chars().all(|c| c.is_ascii_hexdigit()));
    }

    #[test]
    fn compute_hash_different_module_different_hash() {
        let make = |module: &str| MessageInfo {
            name: "Same".into(),
            module: module.into(),
            fields: vec![FieldInfo {
                name: "x".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        // The module deliberately does NOT participate. This is a *layout*
        // hash: it must equal the `LAYOUT_HASH` the `message!` macro emits,
        // which is built from `stringify!($name)` and the fields, and it is
        // compared against the value in the SHM header. Two identically shaped
        // messages are layout-compatible wherever they are declared, and a
        // same-short-name collision between different types is caught
        // separately by the type-name check in the topic open path.
        assert_eq!(
            compute_message_definition_hash(&make("alpha")),
            compute_message_definition_hash(&make("beta")),
            "the module is not part of the layout"
        );
    }

    #[test]
    fn compute_hash_different_name_different_hash() {
        let make = |name: &str| MessageInfo {
            name: name.into(),
            module: "test".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&make("Foo")),
            compute_message_definition_hash(&make("Bar")),
            "different names should produce different hashes"
        );
    }

    #[test]
    fn compute_hash_different_field_type_different_hash() {
        let make = |ft: &str| MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "value".into(),
                field_type: ft.into(),
                doc: String::new(),
            }],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&make("f32")),
            compute_message_definition_hash(&make("f64")),
            "different field types should produce different hashes"
        );
    }

    #[test]
    fn compute_hash_doc_changes_do_not_affect_hash() {
        let make = |doc: &str| MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![FieldInfo {
                name: "x".into(),
                field_type: "f64".into(),
                doc: String::new(),
            }],
            doc: doc.into(),
            source_file: String::new(),
        };
        assert_eq!(
            compute_message_definition_hash(&make("")),
            compute_message_definition_hash(&make("some documentation")),
            "doc changes must not affect the hash (hash is structural)"
        );
    }

    #[test]
    fn compute_hash_source_file_changes_do_not_affect_hash() {
        let make = |sf: &str| MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![],
            doc: String::new(),
            source_file: sf.into(),
        };
        assert_eq!(
            compute_message_definition_hash(&make("a.rs")),
            compute_message_definition_hash(&make("b.rs")),
            "source_file changes must not affect the hash"
        );
    }

    #[test]
    fn compute_hash_field_order_matters() {
        let msg1 = MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![
                FieldInfo {
                    name: "a".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
                FieldInfo {
                    name: "b".into(),
                    field_type: "i32".into(),
                    doc: String::new(),
                },
            ],
            doc: String::new(),
            source_file: String::new(),
        };
        let msg2 = MessageInfo {
            name: "Msg".into(),
            module: "test".into(),
            fields: vec![
                FieldInfo {
                    name: "b".into(),
                    field_type: "i32".into(),
                    doc: String::new(),
                },
                FieldInfo {
                    name: "a".into(),
                    field_type: "f64".into(),
                    doc: String::new(),
                },
            ],
            doc: String::new(),
            source_file: String::new(),
        };
        assert_ne!(
            compute_message_definition_hash(&msg1),
            compute_message_definition_hash(&msg2),
            "field order should affect the hash"
        );
    }

    #[test]
    fn compute_hash_hex_format() {
        let msg = MessageInfo {
            name: "Test".into(),
            module: "m".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        let hash = compute_message_definition_hash(&msg);
        assert_eq!(hash.len(), 10, "hash should be exactly 0x + 8 hex chars");
        assert!(
            hash.starts_with("0x") && hash[2..].chars().all(|c| c.is_ascii_hexdigit()),
            "hash should be 0x followed by lowercase hex: {}",
            hash
        );
    }

    // =========================================================================
    // MessageInfo / FieldInfo — Clone, Debug, construction
    // =========================================================================

    #[test]
    fn message_info_clone() {
        let msg = MessageInfo {
            name: "Twist".into(),
            module: "control".into(),
            fields: vec![FieldInfo {
                name: "linear".into(),
                field_type: "Vec3".into(),
                doc: "Linear velocity".into(),
            }],
            doc: "Twist command".into(),
            source_file: "control.rs".into(),
        };
        let cloned = msg.clone();
        assert_eq!(cloned.name, msg.name);
        assert_eq!(cloned.module, msg.module);
        assert_eq!(cloned.fields.len(), msg.fields.len());
        assert_eq!(cloned.doc, msg.doc);
        assert_eq!(cloned.source_file, msg.source_file);
    }

    #[test]
    fn field_info_clone() {
        let field = FieldInfo {
            name: "x".into(),
            field_type: "f64".into(),
            doc: "X coordinate".into(),
        };
        let cloned = field.clone();
        assert_eq!(cloned.name, "x");
        assert_eq!(cloned.field_type, "f64");
        assert_eq!(cloned.doc, "X coordinate");
    }

    #[test]
    fn message_info_debug() {
        let msg = MessageInfo {
            name: "Ping".into(),
            module: "net".into(),
            fields: vec![],
            doc: String::new(),
            source_file: String::new(),
        };
        let debug = format!("{:?}", msg);
        assert!(debug.contains("Ping"));
        assert!(debug.contains("net"));
    }

    #[test]
    fn field_info_debug() {
        let field = FieldInfo {
            name: "data".into(),
            field_type: "Vec<u8>".into(),
            doc: "raw bytes".into(),
        };
        let debug = format!("{:?}", field);
        assert!(debug.contains("data"));
        assert!(debug.contains("Vec<u8>"));
    }

    // =========================================================================
    // discover_messages + list_messages / show_message / message_hash
    // (integration tests using HORUS_SOURCE_DIR with temp directory)
    // =========================================================================

    /// Helper: create a temp messages directory with sample .rs files
    fn setup_messages_dir() -> (tempfile::TempDir, std::path::PathBuf) {
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_types").join("src");
        fs::create_dir_all(&msgs_dir).unwrap();

        // control.rs
        fs::write(
            msgs_dir.join("control.rs"),
            r#"
/// Twist velocity command
#[derive(Debug, Clone)]
pub struct Twist {
    /// Linear velocity
    pub linear: f64,
    /// Angular velocity
    pub angular: f64,
}
"#,
        )
        .unwrap();

        // sensor.rs
        fs::write(
            msgs_dir.join("sensor.rs"),
            r#"
/// IMU reading
pub struct Imu {
    pub accel_x: f64,
    pub accel_y: f64,
    pub accel_z: f64,
    pub gyro_x: f64,
    pub gyro_y: f64,
    pub gyro_z: f64,
}

/// Range finder reading
pub struct Range {
    pub distance: f64,
    pub min_range: f64,
    pub max_range: f64,
}
"#,
        )
        .unwrap();

        // mod.rs — should be skipped
        fs::write(
            msgs_dir.join("mod.rs"),
            "pub mod control;\npub mod sensor;\n",
        )
        .unwrap();

        // non_rs.txt — should be ignored
        fs::write(msgs_dir.join("non_rs.txt"), "not a rust file").unwrap();

        let p = tmp.path().to_path_buf();
        (tmp, p)
    }

    #[test]
    fn discover_messages_via_env_var() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        // Set the HORUS_SOURCE_DIR to the temp root
        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = discover_messages();
        std::env::remove_var("HORUS_SOURCE_DIR");

        let msgs = result.expect("discover_messages should succeed");
        // Should find Twist (control) + Imu + Range (sensor) = 3
        assert_eq!(msgs.len(), 3, "should discover 3 message types");

        // Sorted by module then name
        let names: Vec<&str> = msgs.iter().map(|m| m.name.as_str()).collect();
        assert!(names.contains(&"Twist"));
        assert!(names.contains(&"Imu"));
        assert!(names.contains(&"Range"));

        // Verify module assignment
        let twist = msgs.iter().find(|m| m.name == "Twist").unwrap();
        assert_eq!(twist.module, "control");
        assert_eq!(twist.fields.len(), 2);

        let imu = msgs.iter().find(|m| m.name == "Imu").unwrap();
        assert_eq!(imu.module, "sensor");
        assert_eq!(imu.fields.len(), 6);

        drop(tmp);
    }

    #[test]
    fn discover_messages_skips_mod_rs() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let msgs = discover_messages().unwrap();
        std::env::remove_var("HORUS_SOURCE_DIR");

        // mod.rs should be skipped — no messages from it
        for m in &msgs {
            assert_ne!(m.module, "mod", "mod.rs should be skipped");
        }

        drop(tmp);
    }

    #[test]
    fn discover_messages_skips_non_rs_files() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let msgs = discover_messages().unwrap();
        std::env::remove_var("HORUS_SOURCE_DIR");

        // non_rs.txt should not produce any messages
        for m in &msgs {
            assert_ne!(m.source_file.as_str(), "non_rs");
        }

        drop(tmp);
    }

    #[test]
    fn discover_messages_sorts_by_module_then_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let msgs = discover_messages().unwrap();
        std::env::remove_var("HORUS_SOURCE_DIR");

        // Verify sorting: control < sensor
        let modules: Vec<&str> = msgs.iter().map(|m| m.module.as_str()).collect();
        // control::Twist comes first, then sensor::Imu, sensor::Range
        assert_eq!(modules[0], "control");
        assert_eq!(modules[1], "sensor");
        assert_eq!(modules[2], "sensor");
        // Within sensor, Imu < Range alphabetically
        assert_eq!(msgs[1].name, "Imu");
        assert_eq!(msgs[2].name, "Range");

        drop(tmp);
    }

    #[test]
    fn discover_messages_error_when_no_dir_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // Point to a nonexistent directory
        std::env::set_var(
            "HORUS_SOURCE_DIR",
            "/tmp/definitely_does_not_exist_horus_test",
        );
        // Also clear HORUS_SOURCE to avoid fallback
        let prev_source = std::env::var("HORUS_SOURCE").ok();
        std::env::remove_var("HORUS_SOURCE");

        let result = discover_messages();
        std::env::remove_var("HORUS_SOURCE_DIR");
        if let Some(v) = prev_source {
            std::env::set_var("HORUS_SOURCE", v);
        }

        // The fake HORUS_SOURCE_DIR won't work, but fallback paths (e.g. a real
        // source tree at ~/softmata/horus, or the installer's cached copy) may
        // still succeed on a developer machine.
        match result {
            Ok(msgs) => {
                // Fallback path found messages — they should be well-formed
                assert!(
                    msgs.iter().all(|m| !m.name.is_empty()),
                    "all discovered messages should have non-empty names"
                );
            }
            Err(e) => {
                // No fallback path found — error should mention the missing directory
                let msg = e.to_string();
                assert!(
                    msg.contains("message definitions") || msg.contains("HORUS_SOURCE"),
                    "error should mention the message definitions or HORUS_SOURCE, got: {msg}"
                );
            }
        }
    }

    #[test]
    fn list_messages_json_output() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        // list_messages prints to stdout; just verify it doesn't error
        let result = list_messages(false, None, true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "list_messages(json=true) should succeed");
        drop(tmp);
    }

    #[test]
    fn list_messages_verbose() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(true, None, false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "list_messages(verbose=true) should succeed");
        drop(tmp);
    }

    #[test]
    fn list_messages_compact() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, None, false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "list_messages(verbose=false, json=false) should succeed"
        );
        drop(tmp);
    }

    #[test]
    fn list_messages_with_filter_match() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, Some("twist"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn list_messages_with_filter_no_match() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, Some("nonexistent_xyz"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        // Should succeed (prints "no types found")
        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn list_messages_filter_by_module() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        // "sensor" should match Imu and Range
        let result = list_messages(false, Some("sensor"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn list_messages_filter_case_insensitive() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        // "TWIST" should match "Twist" case-insensitively
        let result = list_messages(false, Some("TWIST"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn show_message_found_by_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("Twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "show_message('Twist') should succeed");
        drop(tmp);
    }

    #[test]
    fn show_message_case_insensitive() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "show_message('twist') should match case-insensitively"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_by_module_qualified_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("control::Twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "show_message('control::Twist') should succeed"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_not_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("NonExistent", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_err(),
            "show_message for unknown type should error"
        );
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("NonExistent"),
            "error should mention the missing type name"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_json() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = show_message("Imu", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "show_message('Imu', json=true) should succeed"
        );
        drop(tmp);
    }

    #[test]
    fn show_message_with_empty_fields() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_types").join("src");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(msgs_dir.join("empty.rs"), "pub struct EmptyMsg {}\n").unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = show_message("EmptyMsg", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn message_hash_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("Twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "message_hash('Twist') should succeed");
        drop(tmp);
    }

    #[test]
    fn message_hash_not_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("DoesNotExist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_err());
        drop(tmp);
    }

    #[test]
    fn message_hash_json() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("Twist", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "message_hash json mode should succeed");
        drop(tmp);
    }

    #[test]
    fn message_hash_case_insensitive() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("twist", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(
            result.is_ok(),
            "message_hash should match case-insensitively"
        );
        drop(tmp);
    }

    #[test]
    fn message_hash_qualified_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = message_hash("sensor::Imu", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        assert!(result.is_ok(), "message_hash('sensor::Imu') should succeed");
        drop(tmp);
    }

    #[test]
    fn list_messages_empty_dir() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_types").join("src");
        fs::create_dir_all(&msgs_dir).unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = list_messages(false, None, false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        // No messages found — should print "No message types found." and return Ok
        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn list_messages_empty_dir_with_filter() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_types").join("src");
        fs::create_dir_all(&msgs_dir).unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = list_messages(false, Some("anything"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn list_messages_json_with_filter() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(false, Some("control"), true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    #[test]
    fn list_messages_verbose_with_filter() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let (tmp, root) = setup_messages_dir();

        std::env::set_var("HORUS_SOURCE_DIR", root.to_str().unwrap());
        let result = list_messages(true, Some("sensor"), false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    // =========================================================================
    // Roundtrip: parse -> hash consistency
    // =========================================================================

    #[test]
    fn parse_and_hash_roundtrip() {
        let source = r#"
/// Control command
pub struct Cmd {
    pub throttle: f64,
    pub steering: f64,
}
"#;
        let msgs = parse_messages_from_source(source, "ctrl", "ctrl.rs".into());
        assert_eq!(msgs.len(), 1);
        let hash1 = compute_message_definition_hash(&msgs[0]);
        let hash2 = compute_message_definition_hash(&msgs[0]);
        assert_eq!(hash1, hash2, "hash should be deterministic across calls");
        assert_eq!(hash1.len(), 10);
    }

    #[test]
    fn parse_and_hash_changes_when_field_added() {
        let source_v1 = r#"
pub struct Msg {
    pub a: f64,
}
"#;
        let source_v2 = r#"
pub struct Msg {
    pub a: f64,
    pub b: f64,
}
"#;
        let msgs_v1 = parse_messages_from_source(source_v1, "test", "test.rs".into());
        let msgs_v2 = parse_messages_from_source(source_v2, "test", "test.rs".into());
        assert_ne!(
            compute_message_definition_hash(&msgs_v1[0]),
            compute_message_definition_hash(&msgs_v2[0]),
            "adding a field should change the hash"
        );
    }

    #[test]
    fn parse_and_hash_changes_when_field_type_changes() {
        let source_v1 = r#"
pub struct Msg {
    pub value: f32,
}
"#;
        let source_v2 = r#"
pub struct Msg {
    pub value: f64,
}
"#;
        let msgs_v1 = parse_messages_from_source(source_v1, "test", "test.rs".into());
        let msgs_v2 = parse_messages_from_source(source_v2, "test", "test.rs".into());
        assert_ne!(
            compute_message_definition_hash(&msgs_v1[0]),
            compute_message_definition_hash(&msgs_v2[0]),
            "changing field type should change the hash"
        );
    }

    #[test]
    fn parse_and_hash_stable_when_doc_changes() {
        let source_v1 = r#"
/// Version 1 doc
pub struct Msg {
    pub x: f64,
}
"#;
        let source_v2 = r#"
/// Completely different documentation
pub struct Msg {
    pub x: f64,
}
"#;
        let msgs_v1 = parse_messages_from_source(source_v1, "test", "test.rs".into());
        let msgs_v2 = parse_messages_from_source(source_v2, "test", "test.rs".into());
        assert_eq!(
            compute_message_definition_hash(&msgs_v1[0]),
            compute_message_definition_hash(&msgs_v2[0]),
            "doc changes should not affect the hash"
        );
    }

    // =========================================================================
    // Edge case: field doc comes from the line immediately above
    // =========================================================================

    #[test]
    fn parse_messages_field_doc_only_from_immediate_predecessor() {
        let source = r#"
pub struct Msg {
    /// Doc for first
    pub first: f64,
    /// Doc for second
    pub second: f64,
}
"#;
        let msgs = parse_messages_from_source(source, "t", "t.rs".into());
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0].fields[0].doc, "Doc for first");
        assert_eq!(msgs[0].fields[1].doc, "Doc for second");
    }

    #[test]
    fn parse_messages_field_no_doc_when_prev_is_not_doc_comment() {
        let source = r#"
pub struct Msg {
    pub first: f64,
    pub second: f64,
}
"#;
        let msgs = parse_messages_from_source(source, "t", "t.rs".into());
        assert_eq!(msgs.len(), 1);
        // first field's previous line is "{" — not a doc comment
        assert!(msgs[0].fields[0].doc.is_empty());
        // second field's previous line is "pub first: f64," — not a doc comment
        assert!(msgs[0].fields[1].doc.is_empty());
    }

    // =========================================================================
    // Edge: discover_messages with HORUS_SOURCE env var
    // =========================================================================

    #[test]
    fn discover_messages_via_horus_source_env() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_types").join("src");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(
            msgs_dir.join("test.rs"),
            "pub struct Ping { pub seq: u32, }\n",
        )
        .unwrap();

        // Use HORUS_SOURCE_DIR (checked first, highest priority)
        let prev_dir = std::env::var("HORUS_SOURCE_DIR").ok();
        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());

        let result = discover_messages();

        if let Some(v) = prev_dir {
            std::env::set_var("HORUS_SOURCE_DIR", v);
        } else {
            std::env::remove_var("HORUS_SOURCE_DIR");
        }

        let msgs = result.expect("should discover via HORUS_SOURCE_DIR");
        assert!(
            msgs.iter().any(|m| m.name == "Ping"),
            "should find Ping message in {:?}",
            msgs.iter().map(|m| &m.name).collect::<Vec<_>>()
        );
        drop(tmp);
    }

    // =========================================================================
    // show_message with doc
    // =========================================================================

    #[test]
    fn show_message_displays_doc() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_types").join("src");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(
            msgs_dir.join("info.rs"),
            r#"
/// A well-documented message.
/// With multiple lines.
pub struct DocMsg {
    /// The value
    pub value: f64,
}
"#,
        )
        .unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = show_message("DocMsg", false);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }

    // =========================================================================
    // show_message JSON output includes all expected fields
    // =========================================================================

    #[test]
    fn show_message_json_with_fields() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let msgs_dir = tmp.path().join("horus_types").join("src");
        fs::create_dir_all(&msgs_dir).unwrap();

        fs::write(
            msgs_dir.join("json_test.rs"),
            r#"
pub struct JsonTest {
    pub alpha: f64,
    pub beta: String,
}
"#,
        )
        .unwrap();

        std::env::set_var("HORUS_SOURCE_DIR", tmp.path().to_str().unwrap());
        let result = show_message("JsonTest", true);
        std::env::remove_var("HORUS_SOURCE_DIR");

        result.unwrap();
        drop(tmp);
    }
}

#[cfg(test)]
mod robotics_message_discovery_tests {
    use super::*;

    /// `horus msg` scanned only `horus_types/src`, so it could describe the 25
    /// universal types and none of the standard robotics messages — which are
    /// the ones users actually publish. A live topic reported its type
    /// correctly while the introspection command denied the type existed:
    ///
    ///     $ horus topic info cmd_vel   ->  Message Type: CmdVel
    ///     $ horus msg info CmdVel      ->  Message type 'CmdVel' not found
    #[test]
    fn standard_robotics_messages_are_discoverable() {
        // Serialised against the tests that mutate HORUS_SOURCE_DIR/HORUS_SOURCE:
        // discovery reads both, so a concurrent fixture would swap the registry
        // out from under this assertion.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let files = linked_message_files();
        if files.is_empty() {
            eprintln!("skipping: no linked message crate checkout on this machine");
            return;
        }

        let messages = discover_messages().expect("discovery must succeed");
        let names: Vec<&str> = messages.iter().map(|m| m.name.as_str()).collect();

        for expected in ["CmdVel", "Imu", "LaserScan"] {
            assert!(
                names.contains(&expected),
                "`{expected}` is a standard robotics message and must be \
                 describable by `horus msg info`; found {} types",
                names.len()
            );
        }
    }

    /// The universal types must not be lost by adding the second source.
    #[test]
    fn universal_types_are_still_present() {
        // Serialised against the tests that mutate HORUS_SOURCE_DIR/HORUS_SOURCE:
        // discovery reads both, so a concurrent fixture would swap the registry
        // out from under this assertion.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let messages = discover_messages().expect("discovery must succeed");
        let names: Vec<&str> = messages.iter().map(|m| m.name.as_str()).collect();
        for expected in ["Twist", "Pose2D"] {
            assert!(names.contains(&expected), "lost `{expected}`: {names:?}");
        }
    }

    /// A type present in two source trees (a local checkout and the cargo git
    /// cache) must be listed once.
    #[test]
    fn types_are_not_duplicated_across_source_trees() {
        // Serialised against the tests that mutate HORUS_SOURCE_DIR/HORUS_SOURCE:
        // discovery reads both, so a concurrent fixture would swap the registry
        // out from under this assertion.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let messages = discover_messages().expect("discovery must succeed");
        let mut seen = std::collections::HashSet::new();
        for m in &messages {
            assert!(
                seen.insert((m.module.clone(), m.name.clone())),
                "`{}::{}` reported more than once",
                m.module,
                m.name
            );
        }
    }

    /// A linked message crate that is neither named `horus-robotics` nor lays
    /// its messages out as a directory must still be in the registry.
    ///
    /// Discovery keyed on the literal directory prefix `horus-robotics-` and on
    /// finding a `src/messages` **directory**. `horus-tf` — a first-party,
    /// always-linked dependency — ships `src/messages.rs` instead, so
    /// `horus msg info TFMessage` and `horus msg info StaticTransformStamped`
    /// both answered "not found", and any crate with another name was invisible
    /// whatever its layout.
    ///
    /// Hermetic: a synthetic source tree whose lockfile names a synthetic
    /// crate, resolved through the sibling-checkout path.
    #[test]
    fn a_flat_messages_module_in_any_linked_crate_is_discovered() {
        let tmp = tempfile::tempdir().unwrap();
        let source_root = tmp.path().join("horus");
        std::fs::create_dir_all(&source_root).unwrap();
        std::fs::write(
            source_root.join("Cargo.lock"),
            "[[package]]\nname = \"my-msgs\"\nversion = \"0.1.0\"\n\
             source = \"git+https://example.invalid/my-msgs.git?rev=abc1234#abc1234def\"\n",
        )
        .unwrap();

        let crate_src = tmp.path().join("my-msgs").join("src");
        std::fs::create_dir_all(&crate_src).unwrap();
        std::fs::write(
            crate_src.join("messages.rs"),
            "/// A flat message module\npub struct Beacon {\n    pub id: u32,\n}\n",
        )
        .unwrap();

        // `true` is exactly the state `HORUS_SOURCE_DIR=<dir>` puts us in.
        let files = linked_message_files_for(Some(&source_root), true);
        let found: Vec<String> = files
            .iter()
            .map(|(f, m)| format!("{}:{}", m, f.display()))
            .collect();

        assert!(
            files
                .iter()
                .any(|(f, m)| f.ends_with("src/messages.rs") && m == "my_msgs"),
            "a crate shipping src/messages.rs must contribute it under the \
             crate's own module label; got {found:?}"
        );
    }

    /// The directory layout must keep working, with one module per file.
    #[test]
    fn a_messages_directory_in_any_linked_crate_is_discovered() {
        let tmp = tempfile::tempdir().unwrap();
        let source_root = tmp.path().join("horus");
        std::fs::create_dir_all(&source_root).unwrap();
        std::fs::write(
            source_root.join("Cargo.lock"),
            "[[package]]\nname = \"my-msgs\"\nversion = \"0.1.0\"\n\
             source = \"git+https://example.invalid/my-msgs.git?rev=abc1234#abc1234def\"\n",
        )
        .unwrap();

        let msgs = tmp.path().join("my-msgs").join("src").join("messages");
        std::fs::create_dir_all(&msgs).unwrap();
        std::fs::write(msgs.join("mod.rs"), "pub mod sensor;\n").unwrap();
        std::fs::write(
            msgs.join("sensor.rs"),
            "pub struct Beacon {\n    pub id: u32,\n}\n",
        )
        .unwrap();

        let files = linked_message_files_for(Some(&source_root), true);
        assert!(
            files
                .iter()
                .any(|(f, m)| f.ends_with("messages/sensor.rs") && m == "sensor"),
            "got {files:?}"
        );
        assert!(
            !files.iter().any(|(f, _)| f.ends_with("mod.rs")),
            "mod.rs is not a message module: {files:?}"
        );
    }

    /// Pointing `HORUS_SOURCE_DIR` at a real HORUS checkout — the use the
    /// environment-variable reference documents ("path to a HORUS checkout,
    /// used to resolve message types and sources") — must not empty the
    /// registry.
    ///
    /// It used to return early from discovery whenever the variable was set, so
    /// the documented switch reproduced the original bug verbatim:
    /// `HORUS_SOURCE_DIR=/path/to/horus horus msg info CmdVel` -> "Message type
    /// 'CmdVel' not found", and `horus msg list` fell from 97 entries to 31.
    #[test]
    fn an_explicit_source_dir_still_spans_the_linked_message_crates() {
        // Serialised against the tests that mutate HORUS_SOURCE_DIR/HORUS_SOURCE:
        // discovery reads both, so a concurrent fixture would swap the registry
        // out from under this assertion.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let Ok(root) = crate::commands::run::find_horus_source_dir() else {
            eprintln!("skipping: no HORUS source tree on this machine");
            return;
        };
        if !root.join("Cargo.lock").is_file() {
            eprintln!("skipping: {} has no Cargo.lock", root.display());
            return;
        }
        let linked = git_packages_in_lock(&root.join("Cargo.lock"));
        let Some((name, rev)) = linked.iter().find(|(n, r)| {
            git_checkout_dir(n, r).is_some_and(|d| !message_files_in_crate(&d, n).is_empty())
        }) else {
            eprintln!("skipping: no linked message crate is checked out");
            return;
        };

        let files = linked_message_files_for(Some(&root), true);
        assert!(
            !files.is_empty(),
            "`{name}` at {rev} ships message definitions and is linked by \
             {}, but an explicit HORUS_SOURCE_DIR reported no message files \
             at all",
            root.display()
        );
    }

    /// End to end on this machine: the transform messages horus-tf publishes
    /// must be describable. Skipped where cargo has not checked horus-tf out.
    #[test]
    fn horus_tf_message_types_are_in_the_registry() {
        // Serialised against the tests that mutate HORUS_SOURCE_DIR/HORUS_SOURCE:
        // discovery reads both, so a concurrent fixture would swap the registry
        // out from under this assertion.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let checked_out = cargo_home()
            .map(|h| h.join("git").join("checkouts"))
            .and_then(|c| fs::read_dir(c).ok())
            .map(|it| {
                it.flatten()
                    .any(|e| e.file_name().to_string_lossy().starts_with("horus-tf-"))
            })
            .unwrap_or(false);
        if !checked_out {
            eprintln!("skipping: no horus-tf checkout on this machine");
            return;
        }

        let messages = discover_messages().expect("discovery must succeed");
        let names: Vec<&str> = messages.iter().map(|m| m.name.as_str()).collect();
        for expected in ["TFMessage", "StaticTransformStamped"] {
            assert!(
                names.contains(&expected),
                "`{expected}` is published by horus-tf and must be describable \
                 by `horus msg info`; found {} types",
                names.len()
            );
        }
    }

    /// Every discovered type must be spellable on the command line.
    ///
    /// `pub struct PauseRequest;` was registered as "PauseRequest;", so
    /// `horus msg list` advertised a name that `horus msg info` then rejected.
    #[test]
    fn every_discovered_name_is_a_plain_identifier() {
        // Serialised against the tests that mutate HORUS_SOURCE_DIR/HORUS_SOURCE:
        // discovery reads both, so a concurrent fixture would swap the registry
        // out from under this assertion.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let messages = discover_messages().expect("discovery must succeed");
        for m in &messages {
            assert!(
                !m.name.is_empty()
                    && m.name
                        .chars()
                        .all(|c| c.is_ascii_alphanumeric() || c == '_'),
                "`{}` (from {}) is not a name a user can type",
                m.name,
                m.source_file
            );
        }
    }

    fn msg(module: &str, name: &str, fields: &[(&str, &str)]) -> MessageInfo {
        MessageInfo {
            name: name.to_string(),
            module: module.to_string(),
            fields: fields
                .iter()
                .map(|(n, t)| FieldInfo {
                    name: n.to_string(),
                    field_type: t.to_string(),
                    doc: String::new(),
                })
                .collect(),
            doc: String::new(),
            source_file: format!("{module}.rs"),
        }
    }

    /// Two different types sharing a name must not be silently picked between.
    ///
    /// `TransformStamped` exists in `horus_types::math`
    /// (translation/rotation/timestamp_ns) and in `horus-tf`
    /// (parent_frame/child_frame/timestamp_ns/transform). Lookup took the first
    /// match in sort order, so `horus msg hash TransformStamped` printed a hash
    /// for a type the user was not asking about — the exact
    /// wrong-layout-number confusion the command exists to settle.
    #[test]
    fn a_name_shared_by_two_layouts_is_reported_as_ambiguous() {
        let messages = vec![
            msg(
                "math",
                "TransformStamped",
                &[("translation", "[f64; 3]"), ("timestamp_ns", "u64")],
            ),
            msg(
                "tf",
                "TransformStamped",
                &[("parent_frame", "[u8; 64]"), ("timestamp_ns", "u64")],
            ),
        ];

        let err = resolve_message(&messages, "TransformStamped")
            .expect_err("an ambiguous name must not resolve to a guess");
        let text = format!("{err}");
        assert!(text.contains("ambiguous"), "{text}");
        assert!(
            text.contains("math::TransformStamped") && text.contains("tf::TransformStamped"),
            "the error must name both candidates: {text}"
        );

        // ...and the qualified spelling it suggests must work.
        let picked = resolve_message(&messages, "tf::TransformStamped").expect("qualified lookup");
        assert_eq!(picked.module, "tf");
        assert_eq!(
            resolve_message(&messages, "math::TransformStamped")
                .unwrap()
                .module,
            "math"
        );
    }

    /// The same layout under two module labels is not a question, so it must
    /// still answer rather than error.
    #[test]
    fn a_name_shared_by_identical_layouts_still_resolves() {
        let messages = vec![
            msg("a", "Ping", &[("timestamp_ns", "u64")]),
            msg("b", "Ping", &[("timestamp_ns", "u64")]),
        ];
        let picked = resolve_message(&messages, "Ping").expect("identical layouts must resolve");
        assert_eq!(picked.name, "Ping");
    }

    /// An unknown name keeps the message `horus msg list` is advertised in.
    #[test]
    fn an_unknown_name_still_says_not_found() {
        let messages = vec![msg("a", "Ping", &[("timestamp_ns", "u64")])];
        let err = resolve_message(&messages, "Nope").expect_err("must not resolve");
        let text = format!("{err}");
        assert!(
            text.contains("Nope") && text.contains("not found"),
            "{text}"
        );
    }

    #[test]
    fn git_packages_are_read_out_of_a_lockfile() {
        let tmp = tempfile::tempdir().unwrap();
        let lock = tmp.path().join("Cargo.lock");
        std::fs::write(
            &lock,
            "[[package]]\nname = \"serde\"\nversion = \"1.0\"\n\
             source = \"registry+https://github.com/rust-lang/crates.io-index\"\n\n\
             [[package]]\nname = \"horus-tf\"\nversion = \"0.2.0\"\n\
             source = \"git+https://github.com/softmata/horus-tf.git?rev=d525dfc#d525dfcf58b9\"\n\n\
             [[package]]\nname = \"horus_core\"\nversion = \"0.3.0\"\n",
        )
        .unwrap();

        let pkgs = git_packages_in_lock(&lock);
        assert_eq!(
            pkgs,
            vec![("horus-tf".to_string(), "d525dfcf58b9".to_string())],
            "only git packages, paired with their resolved sha"
        );
    }

    #[test]
    fn a_flat_module_is_labelled_by_its_crate() {
        assert_eq!(crate_module_label("horus-tf"), "tf");
        assert_eq!(crate_module_label("horus_tf"), "tf");
        assert_eq!(crate_module_label("my-msgs"), "my_msgs");
        assert_eq!(crate_module_label("horus"), "horus");
    }

    /// Discovery must never fail because an optional source is absent.
    #[test]
    fn missing_robotics_checkout_is_not_an_error() {
        // Serialised against the tests that mutate HORUS_SOURCE_DIR/HORUS_SOURCE:
        // discovery reads both, so a concurrent fixture would swap the registry
        // out from under this assertion.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        // linked_message_files only returns files that exist, so an absent
        // checkout contributes nothing rather than erroring.
        for (file, _module) in linked_message_files() {
            assert!(
                file.is_file(),
                "{} was returned but does not exist",
                file.display()
            );
        }
    }
}

// ─── horus msg gen ───────────────────────────────────────────────────────────

/// Generate Rust, C++ and Python artifacts from `msgs/*.hmsg`.
///
/// A message type usable from all three languages currently has to be written
/// into six places, each with its own syntax. They do not stay in sync: the
/// registries hold 91, 75, 75, 68, 62, 61 and 60 entries, and `horus_cpp`'s
/// layout contract exists because the Rust and C++ definitions of
/// `JointCommand` once drifted to 928 bytes against 88 — an 840-byte overrun on
/// every receive.
///
/// This produces all of them from one definition, with one layout hash.
pub fn generate_messages(check: bool, json: bool) -> HorusResult<()> {
    use crate::msgspec::{
        canonical, emit_cpp, emit_ffi, emit_python, emit_rust, layout, parse, Package,
    };

    let (manifest, root) = crate::manifest::HorusManifest::find_and_load()
        .map_err(|e| HorusError::Config(ConfigError::Other(format!("{e}"))))?;

    let msgs_dir = root.join("msgs");
    if !msgs_dir.is_dir() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "No msgs/ directory in {}.\n\n\
             Message definitions live in `msgs/*.hmsg`. Create one:\n\n  \
             mkdir msgs && cat > msgs/{}.hmsg <<'EOF'\n  \
             #[topic = \"sensor.data\"]\n  \
             SensorReading {{\n      \
             timestamp_ns: u64,\n      \
             value: f32,\n  \
             }}\n  EOF",
            root.display(),
            manifest.package.name.replace('-', "_")
        ))));
    }

    // Sorted, so the generated output does not depend on directory order.
    let mut files: Vec<std::path::PathBuf> = std::fs::read_dir(&msgs_dir)
        .map_err(|e| HorusError::Config(ConfigError::Other(format!("reading msgs/: {e}"))))?
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "hmsg"))
        .collect();
    files.sort();

    if files.is_empty() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "{} contains no .hmsg files",
            msgs_dir.display()
        ))));
    }

    let mut messages = Vec::new();
    let mut diags = Vec::new();
    for file in &files {
        let text = std::fs::read_to_string(file).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!("{}: {e}", file.display())))
        })?;
        match parse::parse_file(&text, file) {
            Ok(mut m) => messages.append(&mut m),
            Err(mut d) => diags.append(&mut d),
        }
    }

    if !diags.is_empty() {
        let body = diags
            .iter()
            .map(|d| format!("  {d}"))
            .collect::<Vec<_>>()
            .join("\n");
        return Err(HorusError::Config(ConfigError::Other(format!(
            "{} problem(s) in message definitions:\n{body}",
            diags.len()
        ))));
    }

    // Duplicate names across files would produce two types with one name.
    let mut seen: std::collections::HashMap<&str, &std::path::Path> =
        std::collections::HashMap::new();
    for m in &messages {
        if let Some(prev) = seen.insert(&m.name, &m.src) {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "message `{}` is declared twice: {} and {}",
                m.name,
                prev.display(),
                m.src.display()
            ))));
        }
    }

    let mut pkg = Package {
        name: manifest.package.name.clone(),
        messages,
    };

    // Resolve every reference, and order the messages so a nested type is
    // emitted before the message that embeds it.
    //
    // The parser records a reference by name and leaves the language paths
    // empty, because a `.hmsg` may name a type declared in another file.
    // Nothing filled them in, and `canonical::render_rust` returns that path
    // verbatim — so `wind: Vec3` became `pub wind: ,` in the generated Rust,
    // `    wind;` in the header, and `WeatherData|temperature:f32|wind:|ts:u64`
    // in the layout identity, while this command printed `Generated 2
    // message(s)`. A reference this generator cannot size is still rejected
    // rather than guessed — a wrong size produces a header whose static_assert
    // passes against the wrong number — but now with a file:line:col and the
    // fields to write instead.
    if let Err(diags) = crate::msgspec::resolve::resolve(&mut pkg) {
        let body = diags
            .iter()
            .map(|d| format!("  {d}"))
            .collect::<Vec<_>>()
            .join("\n");
        return Err(HorusError::Config(ConfigError::Other(format!(
            "{} problem(s) in message definitions:\n{body}",
            diags.len()
        ))));
    }

    // Layouts, in the dependency order `resolve` just established, so every
    // nested type is already sized when the message embedding it is reached.
    let mut env = layout::builtin_layouts();
    for m in &pkg.messages {
        match layout::compute(m, &env) {
            Ok(l) => {
                env.insert(m.name.clone(), (l.size, l.align));
            }
            Err(missing) => {
                // `resolve` rejects unknown names and cycles, and orders what
                // is left, so nothing should reach here. Report rather than
                // panic: an unsized field would otherwise be emitted as a
                // struct with a missing member.
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "{}: `{}` could not be sized while laying out `{}`",
                    m.src.display(),
                    missing,
                    m.name
                ))));
            }
        }
    }

    let pkg = pkg;

    let horus_src = crate::commands::run::run_rust::find_horus_source_dir()
        .map_err(|e| HorusError::Config(ConfigError::Other(format!("{e:#}"))))?;

    let gen_root = root.join(".horus/generated");
    let artifacts: Vec<(std::path::PathBuf, String)> = vec![
        (
            gen_root.join("msgs/Cargo.toml"),
            emit_rust::cargo_toml(&pkg, &horus_src, &manifest.package.version),
        ),
        (
            gen_root.join("msgs/src/lib.rs"),
            emit_rust::lib_rs(&pkg, &env),
        ),
        (
            gen_root.join("msgs_ffi/Cargo.toml"),
            emit_ffi::cargo_toml(&pkg, &horus_src, &manifest.package.version),
        ),
        (gen_root.join("msgs_ffi/src/lib.rs"), emit_ffi::lib_rs(&pkg)),
        (
            gen_root
                .join("include")
                .join(pkg.name.replace('-', "_"))
                .join("msgs.hpp"),
            emit_cpp::header(&pkg, &env),
        ),
        (
            gen_root.join("python").join("msgs.py"),
            emit_python::module(&pkg, &env),
        ),
    ];

    if check {
        let mut stale = Vec::new();
        for (path, want) in &artifacts {
            let have = std::fs::read_to_string(path).unwrap_or_default();
            if &have != want {
                stale.push(path.clone());
            }
        }
        if !stale.is_empty() {
            let body = stale
                .iter()
                .map(|p| format!("  {}", p.display()))
                .collect::<Vec<_>>()
                .join("\n");
            return Err(HorusError::Config(ConfigError::Other(format!(
                "generated message artifacts are out of date:\n{body}\n\n\
                 Run `horus msg gen`."
            ))));
        }
        if !json {
            println!(
                "{} {} message(s) up to date",
                cli_output::ICON_SUCCESS.green(),
                pkg.messages.len()
            );
        }
        return Ok(());
    }

    for (path, contents) in &artifacts {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).map_err(|e| {
                HorusError::Config(ConfigError::Other(format!("{}: {e}", parent.display())))
            })?;
        }
        std::fs::write(path, contents).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!("{}: {e}", path.display())))
        })?;
    }

    if json {
        let entries: Vec<String> = pkg
            .messages
            .iter()
            .map(|m| {
                let l = layout::compute(m, &env).ok();
                format!(
                    "{{\"name\":\"{}\",\"hash\":\"0x{:08x}\",\"canonical\":{:?},\"size\":{},\"fields\":{}}}",
                    m.name,
                    canonical::layout_hash(m),
                    canonical::canonical_form(m),
                    l.as_ref().map(|l| l.size).unwrap_or(0),
                    m.fields.len()
                )
            })
            .collect();
        println!(
            "{{\"messages\":[{}],\"artifacts\":[{}]}}",
            entries.join(","),
            artifacts
                .iter()
                .map(|(p, _)| format!("{:?}", p.display().to_string()))
                .collect::<Vec<_>>()
                .join(",")
        );
        return Ok(());
    }

    println!(
        "{} Generated {} message(s) from {} file(s)",
        cli_output::ICON_SUCCESS.green(),
        pkg.messages.len(),
        files.len()
    );
    for m in &pkg.messages {
        let l = layout::compute(m, &env).ok();
        let size = l.as_ref().map(|l| l.size).unwrap_or(0);
        let pad = l.as_ref().map(|l| l.padding).unwrap_or(0);
        println!(
            "    {:<24} 0x{:08x}  {} bytes{}",
            m.name,
            canonical::layout_hash(m),
            size,
            if pad > 0 {
                format!(" ({pad} padding)")
            } else {
                String::new()
            }
        );
    }
    println!();
    for (path, _) in &artifacts {
        let rel = path.strip_prefix(&root).unwrap_or(path);
        println!("    {}", rel.display().to_string().dimmed());
    }
    Ok(())
}

#[cfg(test)]
mod builtin_layout_identity_tests {
    use super::*;

    /// Every type in `horus_types`, with the layout identity the runtime
    /// carries for it.
    ///
    /// Written out rather than iterated, because there is no way to iterate the
    /// inherent constants of a set of types — and because a type added to
    /// `horus_types` and forgotten here is caught by
    /// `every_horus_types_pod_message_is_listed_here` below.
    fn runtime_identities() -> Vec<(&'static str, u32, &'static str)> {
        macro_rules! row {
            ($t:ty) => {
                (stringify!($t), <$t>::LAYOUT_HASH, <$t>::LAYOUT_CANONICAL)
            };
        }
        use horus_types::*;
        vec![
            row!(Twist),
            row!(Pose2D),
            row!(TransformStamped),
            row!(Point3),
            row!(Vector3),
            row!(Quaternion),
            row!(Pose3D),
            row!(PoseStamped),
            row!(PoseWithCovariance),
            row!(TwistWithCovariance),
            row!(Accel),
            row!(AccelStamped),
            row!(Heartbeat),
            row!(DiagnosticStatus),
            row!(EmergencyStop),
            row!(ResourceUsage),
            row!(DiagnosticValue),
            row!(DiagnosticReport),
            row!(NodeHeartbeat),
            row!(SafetyStatus),
            row!(Clock),
            row!(TimeReference),
            row!(SimSync),
            row!(RateRequest),
        ]
    }

    /// The source `horus msg` scrapes, compiled in so the test does not depend
    /// on where the checkout is.
    const SOURCES: &[(&str, &str)] = &[
        ("math", include_str!("../../../horus_types/src/math.rs")),
        (
            "diagnostics",
            include_str!("../../../horus_types/src/diagnostics.rs"),
        ),
        ("time", include_str!("../../../horus_types/src/time.rs")),
    ];

    fn scraped() -> Vec<MessageInfo> {
        SOURCES
            .iter()
            .flat_map(|(module, src)| {
                parse_messages_from_source(src, module, format!("{module}.rs"))
            })
            .collect()
    }

    /// `horus msg hash Twist` must print the number `Topic::new_checked`
    /// compares for `Twist`.
    ///
    /// The command exists to be held up against a layout-mismatch error, and
    /// for two of the shipped types it printed a number that error can never
    /// show: `parse_field` dropped every field whose name begins with `_`,
    /// calling them "padding fields", so `Clock`'s `_pad: [u8; 6]` and
    /// `SimSync`'s `_pad: [u8; 7]` were missing from the canonical form the CLI
    /// hashed. Padding is layout — `A { x: u8, _pad: [u8; 7], y: u64 }` and
    /// `A { x: u8, y: u64 }` are different types — so the number was wrong, not
    /// merely different.
    #[test]
    fn the_cli_canonical_form_matches_the_runtime_one() {
        let messages = scraped();
        for (name, hash, canonical) in runtime_identities() {
            let msg = messages
                .iter()
                .find(|m| m.name == name)
                .unwrap_or_else(|| panic!("`{name}` was not scraped out of horus_types"));
            let fields: Vec<(String, String)> = msg
                .fields
                .iter()
                .map(|f| (f.name.clone(), f.field_type.clone()))
                .collect();
            let form = canonical_message_form(name, &fields);
            assert_eq!(
                form, canonical,
                "`horus msg` builds a different canonical form for `{name}` than the \
                 runtime does, so the hash it prints is not the hash \
                 `Topic::new_checked` compares"
            );
            assert_eq!(
                compute_message_definition_hash(msg),
                format!("{hash:#010x}"),
                "`horus msg hash {name}` and `{name}::LAYOUT_HASH` disagree"
            );
        }
    }

    /// A `_`-prefixed field must survive scraping.
    ///
    /// The narrow version of the test above: it fails on the one line that was
    /// wrong, so a future "skip padding" reintroduction is named precisely.
    #[test]
    fn a_padding_field_is_part_of_the_layout() {
        assert_eq!(
            parse_field("    _pad: [u8; 6],"),
            Some(("_pad".to_string(), "[u8; 6]".to_string())),
            "padding occupies bytes on the wire; a layout identity that omits it \
             cannot tell two differently-padded structs apart"
        );
    }

    /// A type registered as a POD message but left out of `runtime_identities`
    /// would silently escape the check above.
    #[test]
    fn every_horus_types_pod_message_is_listed_here() {
        let listed: Vec<&str> = runtime_identities().iter().map(|(n, _, _)| *n).collect();
        let mut registered = Vec::new();
        for (_, src) in SOURCES {
            let mut rest = *src;
            while let Some(at) = rest.find("impl_pod_message! {") {
                rest = &rest[at + "impl_pod_message! {".len()..];
                let end = rest.find("\n}\n").unwrap_or(rest.len());
                for line in rest[..end].lines() {
                    let line = line.trim();
                    if let Some(name) = line.strip_suffix(" {") {
                        if name.chars().next().is_some_and(|c| c.is_ascii_uppercase()) {
                            registered.push(name.to_string());
                        }
                    }
                }
                rest = &rest[end..];
            }
        }
        assert!(
            !registered.is_empty(),
            "no impl_pod_message! registrations found — the scan below is vacuous"
        );
        for name in &registered {
            assert!(
                listed.contains(&name.as_str()),
                "`{name}` is registered as a POD message but is not checked against \
                 the CLI's hash; add it to runtime_identities()"
            );
        }
    }
}
