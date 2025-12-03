//! Xacro (XML Macro) preprocessor for URDF files
//!
//! Xacro is a macro language used with URDF to simplify robot description files.
//! Most ROS robots distribute their descriptions as .xacro files rather than raw URDF.
//!
//! This module provides:
//! - Xacro file preprocessing using the `xacro` command-line tool
//! - Fallback to embedded xacro processing for simple cases
//! - Integration with the existing URDF loader

use anyhow::{Context, Result};
use std::collections::HashMap;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::Command;
use tracing::{debug, info};

/// Xacro preprocessor
///
/// Converts .xacro files to URDF by either:
/// 1. Using the ROS `xacro` command-line tool (preferred)
/// 2. Using a built-in simple macro processor (fallback)
pub struct XacroPreprocessor {
    /// Additional arguments passed to xacro (e.g., ["robot_name:=my_robot"])
    pub args: Vec<String>,
    /// Environment variables for xacro execution
    pub env: HashMap<String, String>,
    /// Cache of processed xacro files (path -> URDF content)
    cache: HashMap<PathBuf, String>,
    /// Whether to use the system xacro command
    use_system_xacro: bool,
}

impl Default for XacroPreprocessor {
    fn default() -> Self {
        Self::new()
    }
}

impl XacroPreprocessor {
    /// Create a new Xacro preprocessor
    pub fn new() -> Self {
        // Check if system xacro is available
        let use_system_xacro = Command::new("xacro")
            .arg("--version")
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false);

        if use_system_xacro {
            info!("System xacro command found, will use for preprocessing");
        } else {
            info!("System xacro not found, will use built-in macro processor");
        }

        Self {
            args: Vec::new(),
            env: HashMap::new(),
            cache: HashMap::new(),
            use_system_xacro,
        }
    }

    /// Add a xacro argument (e.g., "robot_name:=my_robot")
    pub fn with_arg(mut self, arg: impl Into<String>) -> Self {
        self.args.push(arg.into());
        self
    }

    /// Add multiple xacro arguments
    pub fn with_args(mut self, args: impl IntoIterator<Item = impl Into<String>>) -> Self {
        self.args.extend(args.into_iter().map(|a| a.into()));
        self
    }

    /// Add an environment variable
    pub fn with_env(mut self, key: impl Into<String>, value: impl Into<String>) -> Self {
        self.env.insert(key.into(), value.into());
        self
    }

    /// Force use of built-in processor (skip system xacro)
    pub fn use_builtin_processor(mut self) -> Self {
        self.use_system_xacro = false;
        self
    }

    /// Process a xacro file and return URDF content
    pub fn process(&mut self, xacro_path: impl AsRef<Path>) -> Result<String> {
        let path = xacro_path.as_ref();
        let canonical_path = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());

        // Check cache
        if let Some(cached) = self.cache.get(&canonical_path) {
            debug!("Using cached URDF for: {}", canonical_path.display());
            return Ok(cached.clone());
        }

        let urdf_content = if self.use_system_xacro {
            self.process_with_system_xacro(path)?
        } else {
            self.process_with_builtin(path)?
        };

        // Cache the result
        self.cache.insert(canonical_path, urdf_content.clone());

        Ok(urdf_content)
    }

    /// Process xacro using system command
    fn process_with_system_xacro(&self, xacro_path: &Path) -> Result<String> {
        info!(
            "Processing xacro with system command: {}",
            xacro_path.display()
        );

        let mut cmd = Command::new("xacro");
        cmd.arg(xacro_path);

        // Add custom arguments
        for arg in &self.args {
            cmd.arg(arg);
        }

        // Add environment variables
        for (key, value) in &self.env {
            cmd.env(key, value);
        }

        // Set working directory to xacro file's directory for relative includes
        if let Some(parent) = xacro_path.parent() {
            cmd.current_dir(parent);
        }

        let output = cmd.output().with_context(|| {
            format!(
                "Failed to execute xacro command for: {}",
                xacro_path.display()
            )
        })?;

        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            anyhow::bail!(
                "Xacro processing failed for {}:\n{}",
                xacro_path.display(),
                stderr
            );
        }

        let urdf_content =
            String::from_utf8(output.stdout).with_context(|| "Xacro output is not valid UTF-8")?;

        debug!(
            "Xacro processed successfully, generated {} bytes of URDF",
            urdf_content.len()
        );

        Ok(urdf_content)
    }

    /// Process xacro using built-in macro processor
    ///
    /// This is a simplified processor that handles common xacro constructs:
    /// - <xacro:include>
    /// - <xacro:property>
    /// - <xacro:macro> and <xacro:call>
    /// - ${...} expressions (simple variable substitution)
    /// - <xacro:if> and <xacro:unless>
    fn process_with_builtin(&self, xacro_path: &Path) -> Result<String> {
        info!(
            "Processing xacro with built-in processor: {}",
            xacro_path.display()
        );

        let content = std::fs::read_to_string(xacro_path)
            .with_context(|| format!("Failed to read xacro file: {}", xacro_path.display()))?;

        let base_path = xacro_path.parent().unwrap_or(Path::new("."));

        let mut processor = BuiltinXacroProcessor::new(base_path);

        // Add arguments as properties
        for arg in &self.args {
            if let Some((key, value)) = arg.split_once(":=") {
                processor.set_property(key, value);
            }
        }

        processor.process(&content)
    }

    /// Process a xacro file and save the result to a temporary file
    pub fn process_to_temp_file(&mut self, xacro_path: impl AsRef<Path>) -> Result<PathBuf> {
        let urdf_content = self.process(xacro_path.as_ref())?;

        // Create temp file
        let mut temp_file = tempfile::NamedTempFile::with_suffix(".urdf")
            .context("Failed to create temporary URDF file")?;

        temp_file
            .write_all(urdf_content.as_bytes())
            .context("Failed to write URDF to temporary file")?;

        let path = temp_file.into_temp_path().keep()?;
        Ok(path)
    }

    /// Clear the processing cache
    pub fn clear_cache(&mut self) {
        self.cache.clear();
    }

    /// Check if a file is a xacro file
    pub fn is_xacro_file(path: &Path) -> bool {
        path.extension()
            .map(|ext| ext.to_str().unwrap_or("").to_lowercase())
            .map(|ext| ext == "xacro" || ext == "urdf.xacro")
            .unwrap_or(false)
    }
}

/// Built-in xacro processor for environments without ROS
struct BuiltinXacroProcessor {
    base_path: PathBuf,
    properties: HashMap<String, String>,
    macros: HashMap<String, XacroMacro>,
}

#[derive(Clone)]
struct XacroMacro {
    params: Vec<String>,
    default_values: HashMap<String, String>,
    body: String,
}

impl BuiltinXacroProcessor {
    fn new(base_path: &Path) -> Self {
        Self {
            base_path: base_path.to_path_buf(),
            properties: HashMap::new(),
            macros: HashMap::new(),
        }
    }

    fn set_property(&mut self, name: &str, value: &str) {
        self.properties.insert(name.to_string(), value.to_string());
    }

    fn process(&mut self, content: &str) -> Result<String> {
        let mut result = content.to_string();

        // First pass: collect properties and macros, process includes
        result = self.first_pass(&result)?;

        // Second pass: expand macros and substitute variables
        result = self.expand_macros(&result)?;
        result = self.substitute_variables(&result)?;

        // Third pass: remove xacro namespace declarations and clean up
        result = self.cleanup(&result);

        Ok(result)
    }

    fn first_pass(&mut self, content: &str) -> Result<String> {
        let mut result = content.to_string();

        // Process includes first
        result = self.process_includes(&result)?;

        // Extract and process properties
        self.extract_properties(&result);

        // Extract macros
        self.extract_macros(&result);

        Ok(result)
    }

    fn process_includes(&self, content: &str) -> Result<String> {
        let mut result = content.to_string();

        // Find all <xacro:include filename="..."/> patterns
        let include_pattern =
            regex::Regex::new(r#"<xacro:include\s+filename\s*=\s*["']([^"']+)["']\s*/?\s*>"#)
                .unwrap();

        // Iterate and replace includes (need to do this in a loop as includes may contain includes)
        let mut max_iterations = 100; // Prevent infinite loops
        while max_iterations > 0 {
            if let Some(caps) = include_pattern.captures(&result) {
                let full_match = caps.get(0).unwrap().as_str();
                let filename = &caps[1];

                // Resolve include path
                let include_path = self.resolve_include_path(filename)?;

                debug!("Processing xacro include: {}", include_path.display());

                let include_content =
                    std::fs::read_to_string(&include_path).with_context(|| {
                        format!("Failed to read included file: {}", include_path.display())
                    })?;

                // Replace the include with the content (stripped of outer robot tag if present)
                let stripped_content = self.strip_outer_robot_tag(&include_content);
                result = result.replace(full_match, &stripped_content);

                max_iterations -= 1;
            } else {
                break;
            }
        }

        Ok(result)
    }

    fn resolve_include_path(&self, filename: &str) -> Result<PathBuf> {
        // Handle package:// URIs
        if let Some(rest) = filename.strip_prefix("package://") {
            // Try to resolve package path
            if let Some(slash_pos) = rest.find('/') {
                let package_name = &rest[..slash_pos];
                let relative_path = &rest[slash_pos + 1..];

                // Try common ROS package locations
                let package_paths = [
                    format!("/opt/ros/humble/share/{}", package_name),
                    format!("/opt/ros/iron/share/{}", package_name),
                    format!("/opt/ros/jazzy/share/{}", package_name),
                    format!(
                        "{}/ros2_ws/src/{}",
                        std::env::var("HOME").unwrap_or_default(),
                        package_name
                    ),
                    format!(
                        "{}/catkin_ws/src/{}",
                        std::env::var("HOME").unwrap_or_default(),
                        package_name
                    ),
                ];

                for pkg_path in &package_paths {
                    let full_path = Path::new(pkg_path).join(relative_path);
                    if full_path.exists() {
                        return Ok(full_path);
                    }
                }

                // Also try relative to base path
                let local_path = self.base_path.join(package_name).join(relative_path);
                if local_path.exists() {
                    return Ok(local_path);
                }
            }
        }

        // Handle $(find package) syntax
        if filename.contains("$(find") {
            let find_pattern = regex::Regex::new(r"\$\(find\s+([^)]+)\)").unwrap();
            if let Some(caps) = find_pattern.captures(filename) {
                let package_name = &caps[1];
                let rest = filename.replace(&caps[0], "");

                // Try common locations
                let package_paths = [
                    format!("/opt/ros/humble/share/{}", package_name),
                    format!("/opt/ros/iron/share/{}", package_name),
                ];

                for pkg_path in &package_paths {
                    let full_path = PathBuf::from(pkg_path).join(rest.trim_start_matches('/'));
                    if full_path.exists() {
                        return Ok(full_path);
                    }
                }
            }
        }

        // Try relative path
        let relative_path = self.base_path.join(filename);
        if relative_path.exists() {
            return Ok(relative_path);
        }

        // Try absolute path
        let absolute_path = PathBuf::from(filename);
        if absolute_path.exists() {
            return Ok(absolute_path);
        }

        anyhow::bail!("Cannot resolve xacro include: {}", filename)
    }

    fn strip_outer_robot_tag(&self, content: &str) -> String {
        // Remove <?xml ...?> declaration
        let content = regex::Regex::new(r"<\?xml[^?]*\?>")
            .unwrap()
            .replace_all(content, "");

        // Remove outer <robot> tags but keep the content
        let content = regex::Regex::new(r"(?s)<robot[^>]*>(.*)</robot>")
            .unwrap()
            .replace(&content, "$1");

        content.to_string()
    }

    fn extract_properties(&mut self, content: &str) {
        // Match <xacro:property name="..." value="..."/>
        let property_pattern = regex::Regex::new(
            r#"<xacro:property\s+name\s*=\s*["']([^"']+)["']\s+value\s*=\s*["']([^"']*)["']\s*/?\s*>"#
        ).unwrap();

        for caps in property_pattern.captures_iter(content) {
            let name = caps[1].to_string();
            let value = caps[2].to_string();
            debug!("Found xacro property: {} = {}", name, value);
            self.properties.insert(name, value);
        }

        // Also match <xacro:property name="...">value</xacro:property>
        let property_block_pattern = regex::Regex::new(
            r#"<xacro:property\s+name\s*=\s*["']([^"']+)["']\s*>([^<]*)</xacro:property>"#,
        )
        .unwrap();

        for caps in property_block_pattern.captures_iter(content) {
            let name = caps[1].to_string();
            let value = caps[2].trim().to_string();
            debug!("Found xacro property block: {} = {}", name, value);
            self.properties.insert(name, value);
        }
    }

    fn extract_macros(&mut self, content: &str) {
        // Match <xacro:macro name="..." params="...">...</xacro:macro>
        let macro_pattern = regex::Regex::new(
            r#"(?s)<xacro:macro\s+name\s*=\s*["']([^"']+)["'](?:\s+params\s*=\s*["']([^"']*)["'])?\s*>(.*?)</xacro:macro>"#
        ).unwrap();

        for caps in macro_pattern.captures_iter(content) {
            let name = caps[1].to_string();
            let params_str = caps.get(2).map(|m| m.as_str()).unwrap_or("");
            let body = caps[3].to_string();

            let (params, default_values) = self.parse_macro_params(params_str);

            debug!("Found xacro macro: {} with {} params", name, params.len());

            self.macros.insert(
                name,
                XacroMacro {
                    params,
                    default_values,
                    body,
                },
            );
        }
    }

    fn parse_macro_params(&self, params_str: &str) -> (Vec<String>, HashMap<String, String>) {
        let mut params = Vec::new();
        let mut defaults = HashMap::new();

        for param in params_str.split_whitespace() {
            if let Some((name, default_value)) = param.split_once(":=") {
                // Parameter with default value
                let clean_name = name.trim_start_matches('*').to_string();
                params.push(clean_name.clone());
                defaults.insert(clean_name, default_value.to_string());
            } else {
                // Parameter without default
                let clean_name = param.trim_start_matches('*').to_string();
                params.push(clean_name);
            }
        }

        (params, defaults)
    }

    fn expand_macros(&self, content: &str) -> Result<String> {
        let mut result = content.to_string();

        // Remove macro definitions (we've already extracted them)
        let macro_def_pattern = regex::Regex::new(
            r#"(?s)<xacro:macro\s+name\s*=\s*["'][^"']+["'](?:\s+params\s*=\s*["'][^"']*["'])?\s*>.*?</xacro:macro>"#
        ).unwrap();
        result = macro_def_pattern.replace_all(&result, "").to_string();

        // Expand macro calls
        let mut max_iterations = 100;
        while max_iterations > 0 {
            let mut expanded = false;

            for (macro_name, macro_def) in &self.macros {
                // Pattern for macro call: <xacro:macro_name param="value" .../>
                let call_pattern = regex::Regex::new(&format!(
                    r#"<xacro:{}\s*([^/>]*)/?\s*>"#,
                    regex::escape(macro_name)
                ))
                .unwrap();

                if let Some(caps) = call_pattern.captures(&result) {
                    let full_match = caps.get(0).unwrap().as_str();
                    let attrs_str = &caps[1];

                    // Parse attributes
                    let attrs = self.parse_attributes(attrs_str);

                    // Substitute parameters in macro body
                    let mut expanded_body = macro_def.body.clone();
                    for param in &macro_def.params {
                        let value = attrs
                            .get(param)
                            .or_else(|| macro_def.default_values.get(param))
                            .cloned()
                            .unwrap_or_default();

                        // Substitute ${param} with value
                        let pattern = format!("${{{}}}", param);
                        expanded_body = expanded_body.replace(&pattern, &value);
                    }

                    result = result.replace(full_match, &expanded_body);
                    expanded = true;
                    break;
                }
            }

            if !expanded {
                break;
            }
            max_iterations -= 1;
        }

        Ok(result)
    }

    fn parse_attributes(&self, attrs_str: &str) -> HashMap<String, String> {
        let mut attrs = HashMap::new();

        let attr_pattern = regex::Regex::new(r#"(\w+)\s*=\s*["']([^"']*)["']"#).unwrap();

        for caps in attr_pattern.captures_iter(attrs_str) {
            attrs.insert(caps[1].to_string(), caps[2].to_string());
        }

        attrs
    }

    fn substitute_variables(&self, content: &str) -> Result<String> {
        let mut result = content.to_string();

        // Substitute ${property} with values
        let var_pattern = regex::Regex::new(r"\$\{([^}]+)\}").unwrap();

        let mut max_iterations = 100;
        while max_iterations > 0 {
            let before = result.clone();

            result = var_pattern
                .replace_all(&result, |caps: &regex::Captures| {
                    let expr = &caps[1];
                    self.evaluate_expression(expr)
                })
                .to_string();

            if result == before {
                break;
            }
            max_iterations -= 1;
        }

        Ok(result)
    }

    fn evaluate_expression(&self, expr: &str) -> String {
        let expr = expr.trim();

        // Simple variable lookup
        if let Some(value) = self.properties.get(expr) {
            return value.clone();
        }

        // Handle simple arithmetic (e.g., "${width/2}")
        if expr.contains('/') || expr.contains('*') || expr.contains('+') || expr.contains('-') {
            if let Some(result) = self.evaluate_simple_math(expr) {
                return result;
            }
        }

        // Return original expression if can't evaluate
        format!("${{{}}}", expr)
    }

    fn evaluate_simple_math(&self, expr: &str) -> Option<String> {
        // Substitute variables first
        let mut expr = expr.to_string();
        for (name, value) in &self.properties {
            expr = expr.replace(name, value);
        }

        // Try to parse and evaluate simple expressions
        // This is a basic implementation - full xacro supports Python expressions

        // Handle simple binary operations
        let operators = ['/', '*', '+', '-'];
        for op in operators {
            if let Some(pos) = expr.find(op) {
                let left = expr[..pos].trim();
                let right = expr[pos + 1..].trim();

                if let (Ok(l), Ok(r)) = (left.parse::<f64>(), right.parse::<f64>()) {
                    let result = match op {
                        '/' => l / r,
                        '*' => l * r,
                        '+' => l + r,
                        '-' => l - r,
                        _ => return None,
                    };
                    return Some(format!("{}", result));
                }
            }
        }

        None
    }

    fn cleanup(&self, content: &str) -> String {
        let mut result = content.to_string();

        // Remove xacro property definitions
        let property_pattern = regex::Regex::new(r#"<xacro:property\s+[^>]*>\s*"#).unwrap();
        result = property_pattern.replace_all(&result, "").to_string();

        let property_block_pattern =
            regex::Regex::new(r#"<xacro:property\s+[^>]*>.*?</xacro:property>\s*"#).unwrap();
        result = property_block_pattern.replace_all(&result, "").to_string();

        // Remove xacro:if and xacro:unless blocks (simplified - just removes the tags)
        let if_pattern = regex::Regex::new(r#"<xacro:if\s+[^>]*>\s*"#).unwrap();
        result = if_pattern.replace_all(&result, "").to_string();
        result = result.replace("</xacro:if>", "");

        let unless_pattern = regex::Regex::new(r#"<xacro:unless\s+[^>]*>\s*"#).unwrap();
        result = unless_pattern.replace_all(&result, "").to_string();
        result = result.replace("</xacro:unless>", "");

        // Remove xacro namespace declarations
        let xmlns_pattern = regex::Regex::new(r#"\s*xmlns:xacro\s*=\s*["'][^"']*["']"#).unwrap();
        result = xmlns_pattern.replace_all(&result, "").to_string();

        // Remove empty lines
        let empty_lines = regex::Regex::new(r"\n\s*\n\s*\n").unwrap();
        while empty_lines.is_match(&result) {
            result = empty_lines.replace_all(&result, "\n\n").to_string();
        }

        result.trim().to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_xacro_file() {
        assert!(XacroPreprocessor::is_xacro_file(Path::new("robot.xacro")));
        assert!(XacroPreprocessor::is_xacro_file(Path::new(
            "robot.urdf.xacro"
        )));
        assert!(!XacroPreprocessor::is_xacro_file(Path::new("robot.urdf")));
        assert!(!XacroPreprocessor::is_xacro_file(Path::new("robot.xml")));
    }

    #[test]
    fn test_simple_property_substitution() {
        let mut processor = BuiltinXacroProcessor::new(Path::new("."));
        processor.set_property("width", "0.5");
        processor.set_property("height", "1.0");

        let content = r#"<box size="${width} ${height} 0.1"/>"#;
        let result = processor.process(content).unwrap();

        assert!(result.contains("0.5"));
        assert!(result.contains("1.0"));
    }

    #[test]
    fn test_property_extraction() {
        let mut processor = BuiltinXacroProcessor::new(Path::new("."));

        let content = r#"
            <xacro:property name="width" value="0.5"/>
            <xacro:property name="height" value="1.0"/>
            <box size="${width} ${height} 0.1"/>
        "#;

        let result = processor.process(content).unwrap();
        assert!(result.contains("0.5"));
        assert!(result.contains("1.0"));
    }

    #[test]
    fn test_simple_math() {
        let mut processor = BuiltinXacroProcessor::new(Path::new("."));
        processor.set_property("width", "1.0");

        let content = r#"<origin xyz="0 0 ${width/2}"/>"#;
        let result = processor.process(content).unwrap();

        assert!(result.contains("0.5"));
    }

    #[test]
    fn test_xacro_preprocessor_creation() {
        let preprocessor = XacroPreprocessor::new();
        assert!(preprocessor.args.is_empty());
        assert!(preprocessor.env.is_empty());
    }

    #[test]
    fn test_xacro_preprocessor_with_args() {
        let preprocessor = XacroPreprocessor::new()
            .with_arg("robot_name:=test_robot")
            .with_arg("use_sim:=true");

        assert_eq!(preprocessor.args.len(), 2);
        assert!(preprocessor
            .args
            .contains(&"robot_name:=test_robot".to_string()));
    }
}
