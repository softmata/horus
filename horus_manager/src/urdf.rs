//! Lightweight URDF sensor extraction for driver key validation.
//!
//! Parses `<gazebo>` extension tags from URDF XML to extract sensor metadata
//! (name, type, link). Used by horus_manager to validate that `[drivers]`
//! keys in horus.toml match actual URDF sensor names.
//!
//! This is intentionally minimal — full URDF parsing lives in `nexus_morphology`
//! and `horus-sim3d/robot/gazebo.rs`. We only need sensor names and types.

use std::path::Path;

/// A sensor discovered in the URDF's `<gazebo>` extensions.
#[derive(Debug, Clone)]
pub struct UrdfSensor {
    /// Sensor name from `<sensor name="...">`.
    pub name: String,
    /// Sensor type from `<sensor type="...">` (e.g., "imu", "ray", "camera").
    pub sensor_type: String,
    /// Link name from `<gazebo reference="...">`.
    pub link_name: String,
}

/// Extract sensor metadata from a URDF file's `<gazebo>` extensions.
///
/// Returns an empty Vec if the file doesn't exist, can't be read, or has
/// no `<gazebo>` sensor definitions. Never errors — this is best-effort
/// validation, not a hard requirement.
pub fn extract_sensors_from_urdf(path: &Path) -> Vec<UrdfSensor> {
    let content = match std::fs::read_to_string(path) {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };
    extract_sensors_from_str(&content)
}

/// Extract sensor metadata from URDF XML string.
pub fn extract_sensors_from_str(xml: &str) -> Vec<UrdfSensor> {
    let mut sensors = Vec::new();

    // Find each <gazebo reference="LINK"> block
    let mut search_pos = 0;
    while let Some(gz_start) = xml[search_pos..].find("<gazebo") {
        let gz_start = search_pos + gz_start;
        let gz_tag_end = match xml[gz_start..].find('>') {
            Some(p) => gz_start + p,
            None => break,
        };

        // Extract reference="LINK_NAME" attribute
        let gz_tag = &xml[gz_start..=gz_tag_end];
        let link_name = extract_attr(gz_tag, "reference").unwrap_or_default();

        // Find matching </gazebo>
        let gz_block_end = match xml[gz_tag_end..].find("</gazebo>") {
            Some(p) => gz_tag_end + p + 9,
            None => break,
        };
        let gz_block = &xml[gz_start..gz_block_end];

        // Find <sensor name="..." type="..."> within this gazebo block
        let mut sensor_search = 0;
        while let Some(s_start) = gz_block[sensor_search..].find("<sensor") {
            let s_start = sensor_search + s_start;
            let s_tag_end = match gz_block[s_start..].find('>') {
                Some(p) => s_start + p,
                None => break,
            };
            let sensor_tag = &gz_block[s_start..=s_tag_end];

            let name = extract_attr(sensor_tag, "name").unwrap_or_default();
            let sensor_type = extract_attr(sensor_tag, "type").unwrap_or_default();

            if !name.is_empty() {
                sensors.push(UrdfSensor {
                    name,
                    sensor_type,
                    link_name: link_name.clone(),
                });
            }

            sensor_search = s_tag_end + 1;
        }

        search_pos = gz_block_end;
    }

    sensors
}

/// Extract a simple XML attribute value: `attr="value"` → `value`.
fn extract_attr(tag: &str, attr_name: &str) -> Option<String> {
    let pattern = format!("{}=\"", attr_name);
    let start = tag.find(&pattern)? + pattern.len();
    let end = tag[start..].find('"')? + start;
    Some(tag[start..end].to_string())
}

/// Validate that `[drivers]` keys match URDF sensor names.
///
/// Returns a list of warnings for driver keys that don't match any URDF sensor.
/// Empty return means all driver keys are valid (or no URDF is available).
pub fn validate_driver_keys(driver_keys: &[&str], urdf_sensors: &[UrdfSensor]) -> Vec<String> {
    if urdf_sensors.is_empty() {
        return Vec::new();
    }

    let sensor_names: std::collections::HashSet<&str> =
        urdf_sensors.iter().map(|s| s.name.as_str()).collect();

    let mut warnings = Vec::new();
    for key in driver_keys {
        if !sensor_names.contains(key) {
            let suggestion = find_closest_match(key, &sensor_names);
            let msg = if let Some(closest) = suggestion {
                format!(
                    "driver '{}' not found in URDF sensors. Did you mean '{}'?",
                    key, closest
                )
            } else {
                format!(
                    "driver '{}' not found in URDF sensors. Available: [{}]",
                    key,
                    urdf_sensors
                        .iter()
                        .map(|s| s.name.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                )
            };
            warnings.push(msg);
        }
    }
    warnings
}

/// Simple closest-match finder using character overlap.
fn find_closest_match<'a>(
    target: &str,
    candidates: &std::collections::HashSet<&'a str>,
) -> Option<&'a str> {
    candidates
        .iter()
        .filter(|c| {
            // Simple heuristic: shares at least half the characters
            let shared = target.chars().filter(|ch| c.contains(*ch)).count();
            shared > target.len() / 2
        })
        .min_by_key(|c| {
            // Levenshtein-like: count differing characters
            let max_len = target.len().max(c.len());
            let shared = target
                .chars()
                .zip(c.chars())
                .filter(|(a, b)| a == b)
                .count();
            max_len - shared
        })
        .copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extract_sensors_from_gazebo_tags() {
        let urdf = r#"
<robot name="test">
  <link name="base_link"/>
  <link name="lidar_link"/>
  <gazebo reference="lidar_link">
    <sensor name="front_lidar" type="ray">
      <update_rate>10</update_rate>
    </sensor>
  </gazebo>
  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
    </sensor>
  </gazebo>
</robot>"#;

        let sensors = extract_sensors_from_str(urdf);
        assert_eq!(sensors.len(), 2);

        assert_eq!(sensors[0].name, "front_lidar");
        assert_eq!(sensors[0].sensor_type, "ray");
        assert_eq!(sensors[0].link_name, "lidar_link");

        assert_eq!(sensors[1].name, "imu_sensor");
        assert_eq!(sensors[1].sensor_type, "imu");
        assert_eq!(sensors[1].link_name, "base_link");
    }

    #[test]
    fn extract_sensors_no_gazebo_tags() {
        let urdf = r#"<robot name="simple"><link name="base_link"/></robot>"#;
        let sensors = extract_sensors_from_str(urdf);
        assert!(sensors.is_empty());
    }

    #[test]
    fn extract_sensors_camera() {
        let urdf = r#"
<gazebo reference="camera_link">
  <sensor name="rgb_camera" type="camera">
    <camera>
      <image><width>640</width><height>480</height></image>
    </camera>
  </sensor>
</gazebo>"#;
        let sensors = extract_sensors_from_str(urdf);
        assert_eq!(sensors.len(), 1);
        assert_eq!(sensors[0].name, "rgb_camera");
        assert_eq!(sensors[0].sensor_type, "camera");
    }

    #[test]
    fn validate_driver_keys_all_match() {
        let sensors = vec![
            UrdfSensor {
                name: "front_lidar".into(),
                sensor_type: "ray".into(),
                link_name: "lidar_link".into(),
            },
            UrdfSensor {
                name: "imu_sensor".into(),
                sensor_type: "imu".into(),
                link_name: "base_link".into(),
            },
        ];
        let warnings = validate_driver_keys(&["front_lidar", "imu_sensor"], &sensors);
        assert!(warnings.is_empty());
    }

    #[test]
    fn validate_driver_keys_mismatch() {
        let sensors = vec![UrdfSensor {
            name: "front_lidar".into(),
            sensor_type: "ray".into(),
            link_name: "lidar_link".into(),
        }];
        let warnings = validate_driver_keys(&["frnt_lidar"], &sensors);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].contains("frnt_lidar"));
        assert!(warnings[0].contains("front_lidar")); // suggestion
    }

    #[test]
    fn validate_driver_keys_empty_urdf() {
        let warnings = validate_driver_keys(&["anything"], &[]);
        assert!(warnings.is_empty()); // no URDF = no validation
    }

    #[test]
    fn extract_attr_basic() {
        assert_eq!(
            extract_attr(r#"<sensor name="foo" type="bar">"#, "name"),
            Some("foo".into())
        );
        assert_eq!(
            extract_attr(r#"<sensor name="foo" type="bar">"#, "type"),
            Some("bar".into())
        );
        assert_eq!(extract_attr(r#"<sensor name="foo">"#, "missing"), None);
    }

    #[test]
    fn test_multiple_sensors_in_one_gazebo_block() {
        let urdf = r#"
<robot name="multi_sensor">
  <link name="sensor_link"/>
  <gazebo reference="sensor_link">
    <sensor name="front_camera" type="camera">
      <update_rate>30</update_rate>
    </sensor>
    <sensor name="rear_camera" type="camera">
      <update_rate>15</update_rate>
    </sensor>
  </gazebo>
</robot>"#;

        let sensors = extract_sensors_from_str(urdf);
        assert_eq!(sensors.len(), 2, "expected 2 sensors in one gazebo block");

        assert_eq!(sensors[0].name, "front_camera");
        assert_eq!(sensors[0].sensor_type, "camera");
        assert_eq!(sensors[0].link_name, "sensor_link");

        assert_eq!(sensors[1].name, "rear_camera");
        assert_eq!(sensors[1].sensor_type, "camera");
        assert_eq!(sensors[1].link_name, "sensor_link");
    }

    #[test]
    fn test_find_closest_match_basic() {
        let mut candidates = std::collections::HashSet::new();
        candidates.insert("front_lidar");
        candidates.insert("imu_sensor");
        candidates.insert("rear_camera");

        // Close misspelling should find a match
        let result = find_closest_match("frnt_lidar", &candidates);
        assert_eq!(
            result,
            Some("front_lidar"),
            "should suggest front_lidar for frnt_lidar"
        );

        // Completely unrelated string should return None
        let result = find_closest_match("xyz", &candidates);
        assert!(result.is_none(), "should not suggest anything for 'xyz'");

        // Exact match is its own closest
        let result = find_closest_match("imu_sensor", &candidates);
        assert_eq!(
            result,
            Some("imu_sensor"),
            "exact match should return itself"
        );
    }
}
