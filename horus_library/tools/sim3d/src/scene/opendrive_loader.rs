//! OpenDRIVE (.xodr) Road Network Loader
//!
//! Supports loading road networks from OpenDRIVE format (ASAM OpenDRIVE 1.4/1.5/1.6/1.7)
//! commonly used for autonomous driving simulation.
//!
//! Features:
//! - Road geometry (lines, arcs, spirals, polynomials)
//! - Lane definitions and lane boundaries
//! - Road markings and signals
//! - Junction connections
//! - Elevation profiles
//! - Surface properties and friction zones

use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

use crate::error::{EnhancedError, ErrorCategory, Result};

// ============================================================================
// OpenDRIVE Data Structures
// ============================================================================

/// Complete OpenDRIVE road network
#[derive(Debug, Clone, Default)]
pub struct OpenDRIVENetwork {
    /// Header information
    pub header: OpenDRIVEHeader,
    /// All roads in the network
    pub roads: Vec<Road>,
    /// Junction definitions
    pub junctions: Vec<Junction>,
    /// Controllers for signals
    pub controllers: Vec<Controller>,
    /// Geographic reference
    pub geo_reference: Option<GeoReference>,
}

/// OpenDRIVE header information
#[derive(Debug, Clone, Default)]
pub struct OpenDRIVEHeader {
    /// OpenDRIVE version
    pub rev_major: u32,
    pub rev_minor: u32,
    /// Map name
    pub name: String,
    /// Version string
    pub version: String,
    /// Creation date
    pub date: String,
    /// North offset for geographic coordinates
    pub north: f64,
    /// South offset
    pub south: f64,
    /// East offset
    pub east: f64,
    /// West offset
    pub west: f64,
    /// Vendor info
    pub vendor: String,
}

/// Geographic reference information
#[derive(Debug, Clone)]
pub struct GeoReference {
    /// Proj4 string or WKT
    pub proj_string: String,
    /// EPSG code if available
    pub epsg_code: Option<u32>,
}

/// Road definition
#[derive(Debug, Clone)]
pub struct Road {
    /// Unique road ID
    pub id: String,
    /// Road name
    pub name: String,
    /// Total length
    pub length: f64,
    /// Junction ID if part of junction (-1 if not)
    pub junction: i32,
    /// Road rule (RHT or LHT)
    pub rule: TrafficRule,
    /// Plan view geometry
    pub plan_view: Vec<Geometry>,
    /// Elevation profile
    pub elevation_profile: Vec<Elevation>,
    /// Lateral profile (superelevation, crossfall)
    pub lateral_profile: LateralProfile,
    /// Lane sections
    pub lanes: LaneSection,
    /// Road objects (signs, barriers, etc.)
    pub objects: Vec<RoadObject>,
    /// Traffic signals
    pub signals: Vec<Signal>,
    /// Surface properties
    pub surface: Option<Surface>,
    /// Link to predecessor/successor roads
    pub link: RoadLink,
}

/// Traffic rule (driving side)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TrafficRule {
    #[default]
    RHT, // Right-hand traffic
    LHT, // Left-hand traffic
}

/// Road link connections
#[derive(Debug, Clone, Default)]
pub struct RoadLink {
    /// Predecessor road connection
    pub predecessor: Option<RoadConnection>,
    /// Successor road connection
    pub successor: Option<RoadConnection>,
}

/// Connection to another road element
#[derive(Debug, Clone)]
pub struct RoadConnection {
    /// Element type (road or junction)
    pub element_type: ConnectionElementType,
    /// Element ID
    pub element_id: String,
    /// Contact point (start or end)
    pub contact_point: ContactPoint,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionElementType {
    Road,
    Junction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ContactPoint {
    Start,
    End,
}

/// Plan view geometry element
#[derive(Debug, Clone)]
pub struct Geometry {
    /// S coordinate (distance along reference line)
    pub s: f64,
    /// X coordinate at start
    pub x: f64,
    /// Y coordinate at start
    pub y: f64,
    /// Heading at start (radians)
    pub hdg: f64,
    /// Length of this geometry element
    pub length: f64,
    /// Geometry type
    pub geometry_type: GeometryType,
}

/// Types of road geometry
#[derive(Debug, Clone)]
pub enum GeometryType {
    /// Straight line
    Line,
    /// Arc with constant curvature
    Arc { curvature: f64 },
    /// Euler spiral (clothoid)
    Spiral { curv_start: f64, curv_end: f64 },
    /// Cubic polynomial
    Poly3 { a: f64, b: f64, c: f64, d: f64 },
    /// Parametric cubic polynomial
    ParamPoly3 {
        au: f64,
        bu: f64,
        cu: f64,
        du: f64,
        av: f64,
        bv: f64,
        cv: f64,
        dv: f64,
        p_range: ParamRange,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamRange {
    ArcLength,
    Normalized,
}

/// Elevation element
#[derive(Debug, Clone)]
pub struct Elevation {
    /// S coordinate
    pub s: f64,
    /// Polynomial coefficients: a + b*ds + c*ds^2 + d*ds^3
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

/// Lateral profile
#[derive(Debug, Clone, Default)]
pub struct LateralProfile {
    /// Superelevation records
    pub superelevation: Vec<Superelevation>,
    /// Crossfall records
    pub crossfall: Vec<Crossfall>,
}

/// Superelevation (banking) record
#[derive(Debug, Clone)]
pub struct Superelevation {
    pub s: f64,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

/// Crossfall record
#[derive(Debug, Clone)]
pub struct Crossfall {
    pub s: f64,
    pub side: CrossfallSide,
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrossfallSide {
    Left,
    Right,
    Both,
}

/// Lane section containing all lanes
#[derive(Debug, Clone, Default)]
pub struct LaneSection {
    /// Lane section start s-coordinate
    pub s: f64,
    /// Left lanes (positive IDs)
    pub left: Vec<Lane>,
    /// Center lane (ID 0)
    pub center: Option<Lane>,
    /// Right lanes (negative IDs)
    pub right: Vec<Lane>,
}

/// Individual lane definition
#[derive(Debug, Clone)]
pub struct Lane {
    /// Lane ID (positive=left, 0=center, negative=right)
    pub id: i32,
    /// Lane type
    pub lane_type: LaneType,
    /// Level (true if on same level as reference line)
    pub level: bool,
    /// Lane width records
    pub width: Vec<LaneWidth>,
    /// Road marks on this lane
    pub road_marks: Vec<RoadMark>,
    /// Speed limit (m/s)
    pub speed_limit: Option<f64>,
    /// Link to lanes in predecessor/successor sections
    pub link: LaneLink,
    /// Material/surface
    pub material: Option<LaneMaterial>,
}

/// Lane types
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LaneType {
    #[default]
    Driving,
    Stop,
    Shoulder,
    Biking,
    Sidewalk,
    Border,
    Restricted,
    Parking,
    Bidirectional,
    Median,
    Special1,
    Special2,
    Special3,
    RoadWorks,
    Tram,
    Rail,
    Entry,
    Exit,
    OffRamp,
    OnRamp,
    None,
}

/// Lane width polynomial
#[derive(Debug, Clone)]
pub struct LaneWidth {
    /// Offset from section start
    pub s_offset: f64,
    /// Polynomial: a + b*ds + c*ds^2 + d*ds^3
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub d: f64,
}

/// Road marking on lane
#[derive(Debug, Clone)]
pub struct RoadMark {
    pub s_offset: f64,
    pub mark_type: RoadMarkType,
    pub weight: RoadMarkWeight,
    pub color: RoadMarkColor,
    pub width: f64,
    pub lane_change: LaneChange,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RoadMarkType {
    #[default]
    None,
    Solid,
    Broken,
    SolidSolid,
    SolidBroken,
    BrokenSolid,
    BrokenBroken,
    BottsDots,
    Grass,
    Curb,
    Edge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RoadMarkWeight {
    #[default]
    Standard,
    Bold,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RoadMarkColor {
    #[default]
    White,
    Yellow,
    Red,
    Blue,
    Green,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LaneChange {
    #[default]
    Both,
    Increase,
    Decrease,
    None,
}

/// Lane connection link
#[derive(Debug, Clone, Default)]
pub struct LaneLink {
    pub predecessor: Option<i32>,
    pub successor: Option<i32>,
}

/// Lane material
#[derive(Debug, Clone)]
pub struct LaneMaterial {
    pub surface: String,
    pub friction: f64,
    pub roughness: f64,
}

/// Road object (signs, barriers, etc.)
#[derive(Debug, Clone)]
pub struct RoadObject {
    pub id: String,
    pub name: String,
    pub s: f64,
    pub t: f64,
    pub z_offset: f64,
    pub object_type: String,
    pub orientation: Orientation,
    pub width: f64,
    pub length: f64,
    pub height: f64,
    pub hdg: f64,
    pub pitch: f64,
    pub roll: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Orientation {
    #[default]
    Plus,
    Minus,
    None,
}

/// Traffic signal
#[derive(Debug, Clone)]
pub struct Signal {
    pub id: String,
    pub name: String,
    pub s: f64,
    pub t: f64,
    pub z_offset: f64,
    pub signal_type: String,
    pub subtype: String,
    pub orientation: Orientation,
    pub dynamic: bool,
    pub value: f64,
    pub width: f64,
    pub height: f64,
    pub country: String,
}

/// Surface properties
#[derive(Debug, Clone)]
pub struct Surface {
    pub friction_zones: Vec<FrictionZone>,
}

#[derive(Debug, Clone)]
pub struct FrictionZone {
    pub s_start: f64,
    pub s_end: f64,
    pub friction: f64,
}

/// Junction definition
#[derive(Debug, Clone)]
pub struct Junction {
    pub id: String,
    pub name: String,
    pub junction_type: JunctionType,
    pub connections: Vec<JunctionConnection>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JunctionType {
    #[default]
    Default,
    Virtual,
    Direct,
}

/// Junction connection
#[derive(Debug, Clone)]
pub struct JunctionConnection {
    pub id: String,
    pub incoming_road: String,
    pub connecting_road: String,
    pub contact_point: ContactPoint,
    pub lane_links: Vec<JunctionLaneLink>,
}

/// Lane link within junction
#[derive(Debug, Clone)]
pub struct JunctionLaneLink {
    pub from: i32,
    pub to: i32,
}

/// Signal controller
#[derive(Debug, Clone)]
pub struct Controller {
    pub id: String,
    pub name: String,
    pub sequence: u32,
    pub controls: Vec<ControlEntry>,
}

#[derive(Debug, Clone)]
pub struct ControlEntry {
    pub signal_id: String,
    pub control_type: String,
}

// ============================================================================
// OpenDRIVE Loader
// ============================================================================

/// Loader for OpenDRIVE files
pub struct OpenDRIVELoader {
    base_path: PathBuf,
}

impl OpenDRIVELoader {
    /// Create a new loader
    pub fn new() -> Self {
        Self {
            base_path: PathBuf::new(),
        }
    }

    /// Set base path for resolving references
    pub fn with_base_path<P: AsRef<Path>>(mut self, path: P) -> Self {
        self.base_path = path.as_ref().to_path_buf();
        self
    }

    /// Load OpenDRIVE network from file
    pub fn load<P: AsRef<Path>>(&self, path: P) -> Result<OpenDRIVENetwork> {
        let path = self.resolve_path(path.as_ref());
        let content = std::fs::read_to_string(&path).map_err(|e| {
            EnhancedError::new(format!("Failed to read OpenDRIVE file: {}", e))
                .with_file(&path)
                .with_category(ErrorCategory::FileNotFound)
        })?;

        self.parse(&content, &path)
    }

    /// Parse OpenDRIVE XML content
    pub fn parse(&self, content: &str, source_path: &Path) -> Result<OpenDRIVENetwork> {
        let doc = roxmltree::Document::parse(content).map_err(|e| {
            EnhancedError::new(format!("Failed to parse OpenDRIVE XML: {}", e))
                .with_file(source_path)
                .with_category(ErrorCategory::ParseError)
        })?;

        let root = doc.root_element();
        if root.tag_name().name() != "OpenDRIVE" {
            return Err(EnhancedError::new("Not a valid OpenDRIVE file")
                .with_file(source_path)
                .with_hint("Root element must be <OpenDRIVE>"));
        }

        let mut network = OpenDRIVENetwork::default();

        // Parse header
        if let Some(header) = root.children().find(|n| n.tag_name().name() == "header") {
            network.header = self.parse_header(&header);
        }

        // Parse roads
        for road_node in root.children().filter(|n| n.tag_name().name() == "road") {
            if let Ok(road) = self.parse_road(&road_node) {
                network.roads.push(road);
            }
        }

        // Parse junctions
        for junction_node in root
            .children()
            .filter(|n| n.tag_name().name() == "junction")
        {
            if let Ok(junction) = self.parse_junction(&junction_node) {
                network.junctions.push(junction);
            }
        }

        // Parse controllers
        for controller_node in root
            .children()
            .filter(|n| n.tag_name().name() == "controller")
        {
            if let Ok(controller) = self.parse_controller(&controller_node) {
                network.controllers.push(controller);
            }
        }

        // Parse geo reference
        if let Some(geo_ref) = root
            .children()
            .find(|n| n.tag_name().name() == "geoReference")
        {
            if let Some(text) = geo_ref.text() {
                network.geo_reference = Some(GeoReference {
                    proj_string: text.trim().to_string(),
                    epsg_code: None,
                });
            }
        }

        info!(
            "Loaded OpenDRIVE network '{}' with {} roads and {} junctions",
            network.header.name,
            network.roads.len(),
            network.junctions.len()
        );

        Ok(network)
    }

    fn parse_header(&self, node: &roxmltree::Node) -> OpenDRIVEHeader {
        OpenDRIVEHeader {
            rev_major: node
                .attribute("revMajor")
                .and_then(|s| s.parse().ok())
                .unwrap_or(1),
            rev_minor: node
                .attribute("revMinor")
                .and_then(|s| s.parse().ok())
                .unwrap_or(4),
            name: node.attribute("name").unwrap_or("").to_string(),
            version: node.attribute("version").unwrap_or("").to_string(),
            date: node.attribute("date").unwrap_or("").to_string(),
            north: node
                .attribute("north")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            south: node
                .attribute("south")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            east: node
                .attribute("east")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            west: node
                .attribute("west")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            vendor: node.attribute("vendor").unwrap_or("").to_string(),
        }
    }

    fn parse_road(&self, node: &roxmltree::Node) -> Result<Road> {
        let id = node.attribute("id").unwrap_or("").to_string();
        let name = node.attribute("name").unwrap_or("").to_string();
        let length = node
            .attribute("length")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);
        let junction = node
            .attribute("junction")
            .and_then(|s| s.parse().ok())
            .unwrap_or(-1);
        let rule = match node.attribute("rule") {
            Some("LHT") => TrafficRule::LHT,
            _ => TrafficRule::RHT,
        };

        let mut road = Road {
            id,
            name,
            length,
            junction,
            rule,
            plan_view: Vec::new(),
            elevation_profile: Vec::new(),
            lateral_profile: LateralProfile::default(),
            lanes: LaneSection::default(),
            objects: Vec::new(),
            signals: Vec::new(),
            surface: None,
            link: RoadLink::default(),
        };

        // Parse link
        if let Some(link_node) = node.children().find(|n| n.tag_name().name() == "link") {
            road.link = self.parse_road_link(&link_node);
        }

        // Parse plan view
        if let Some(pv_node) = node.children().find(|n| n.tag_name().name() == "planView") {
            for geom_node in pv_node
                .children()
                .filter(|n| n.tag_name().name() == "geometry")
            {
                if let Ok(geom) = self.parse_geometry(&geom_node) {
                    road.plan_view.push(geom);
                }
            }
        }

        // Parse elevation profile
        if let Some(ep_node) = node
            .children()
            .find(|n| n.tag_name().name() == "elevationProfile")
        {
            for elev_node in ep_node
                .children()
                .filter(|n| n.tag_name().name() == "elevation")
            {
                road.elevation_profile
                    .push(self.parse_elevation(&elev_node));
            }
        }

        // Parse lateral profile
        if let Some(lp_node) = node
            .children()
            .find(|n| n.tag_name().name() == "lateralProfile")
        {
            road.lateral_profile = self.parse_lateral_profile(&lp_node);
        }

        // Parse lanes
        if let Some(lanes_node) = node.children().find(|n| n.tag_name().name() == "lanes") {
            if let Some(section_node) = lanes_node
                .children()
                .find(|n| n.tag_name().name() == "laneSection")
            {
                road.lanes = self.parse_lane_section(&section_node);
            }
        }

        // Parse objects
        if let Some(objects_node) = node.children().find(|n| n.tag_name().name() == "objects") {
            for obj_node in objects_node
                .children()
                .filter(|n| n.tag_name().name() == "object")
            {
                if let Ok(obj) = self.parse_road_object(&obj_node) {
                    road.objects.push(obj);
                }
            }
        }

        // Parse signals
        if let Some(signals_node) = node.children().find(|n| n.tag_name().name() == "signals") {
            for sig_node in signals_node
                .children()
                .filter(|n| n.tag_name().name() == "signal")
            {
                if let Ok(sig) = self.parse_signal(&sig_node) {
                    road.signals.push(sig);
                }
            }
        }

        // Parse surface
        if let Some(surface_node) = node.children().find(|n| n.tag_name().name() == "surface") {
            road.surface = Some(self.parse_surface(&surface_node));
        }

        Ok(road)
    }

    fn parse_road_link(&self, node: &roxmltree::Node) -> RoadLink {
        let mut link = RoadLink::default();

        if let Some(pred) = node
            .children()
            .find(|n| n.tag_name().name() == "predecessor")
        {
            link.predecessor = self.parse_road_connection(&pred);
        }

        if let Some(succ) = node.children().find(|n| n.tag_name().name() == "successor") {
            link.successor = self.parse_road_connection(&succ);
        }

        link
    }

    fn parse_road_connection(&self, node: &roxmltree::Node) -> Option<RoadConnection> {
        let element_type = match node.attribute("elementType") {
            Some("road") => ConnectionElementType::Road,
            Some("junction") => ConnectionElementType::Junction,
            _ => return None,
        };

        let element_id = node.attribute("elementId")?.to_string();

        let contact_point = match node.attribute("contactPoint") {
            Some("end") => ContactPoint::End,
            _ => ContactPoint::Start,
        };

        Some(RoadConnection {
            element_type,
            element_id,
            contact_point,
        })
    }

    fn parse_geometry(&self, node: &roxmltree::Node) -> Result<Geometry> {
        let s = node
            .attribute("s")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);
        let x = node
            .attribute("x")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);
        let y = node
            .attribute("y")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);
        let hdg = node
            .attribute("hdg")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);
        let length = node
            .attribute("length")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);

        // Determine geometry type
        let geometry_type = if node.children().any(|n| n.tag_name().name() == "line") {
            GeometryType::Line
        } else if let Some(arc_node) = node.children().find(|n| n.tag_name().name() == "arc") {
            let curvature = arc_node
                .attribute("curvature")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            GeometryType::Arc { curvature }
        } else if let Some(spiral_node) = node.children().find(|n| n.tag_name().name() == "spiral")
        {
            let curv_start = spiral_node
                .attribute("curvStart")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let curv_end = spiral_node
                .attribute("curvEnd")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            GeometryType::Spiral {
                curv_start,
                curv_end,
            }
        } else if let Some(poly_node) = node.children().find(|n| n.tag_name().name() == "poly3") {
            let a = poly_node
                .attribute("a")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let b = poly_node
                .attribute("b")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let c = poly_node
                .attribute("c")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let d = poly_node
                .attribute("d")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            GeometryType::Poly3 { a, b, c, d }
        } else if let Some(ppoly_node) = node
            .children()
            .find(|n| n.tag_name().name() == "paramPoly3")
        {
            let au = ppoly_node
                .attribute("aU")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let bu = ppoly_node
                .attribute("bU")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let cu = ppoly_node
                .attribute("cU")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let du = ppoly_node
                .attribute("dU")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let av = ppoly_node
                .attribute("aV")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let bv = ppoly_node
                .attribute("bV")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let cv = ppoly_node
                .attribute("cV")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let dv = ppoly_node
                .attribute("dV")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let p_range = match ppoly_node.attribute("pRange") {
                Some("normalized") => ParamRange::Normalized,
                _ => ParamRange::ArcLength,
            };
            GeometryType::ParamPoly3 {
                au,
                bu,
                cu,
                du,
                av,
                bv,
                cv,
                dv,
                p_range,
            }
        } else {
            GeometryType::Line
        };

        Ok(Geometry {
            s,
            x,
            y,
            hdg,
            length,
            geometry_type,
        })
    }

    fn parse_elevation(&self, node: &roxmltree::Node) -> Elevation {
        Elevation {
            s: node
                .attribute("s")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            a: node
                .attribute("a")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            b: node
                .attribute("b")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            c: node
                .attribute("c")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            d: node
                .attribute("d")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
        }
    }

    fn parse_lateral_profile(&self, node: &roxmltree::Node) -> LateralProfile {
        let mut profile = LateralProfile::default();

        for se_node in node
            .children()
            .filter(|n| n.tag_name().name() == "superelevation")
        {
            profile.superelevation.push(Superelevation {
                s: se_node
                    .attribute("s")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                a: se_node
                    .attribute("a")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                b: se_node
                    .attribute("b")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                c: se_node
                    .attribute("c")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                d: se_node
                    .attribute("d")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
            });
        }

        for cf_node in node
            .children()
            .filter(|n| n.tag_name().name() == "crossfall")
        {
            let side = match cf_node.attribute("side") {
                Some("left") => CrossfallSide::Left,
                Some("right") => CrossfallSide::Right,
                _ => CrossfallSide::Both,
            };
            profile.crossfall.push(Crossfall {
                s: cf_node
                    .attribute("s")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                side,
                a: cf_node
                    .attribute("a")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                b: cf_node
                    .attribute("b")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                c: cf_node
                    .attribute("c")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                d: cf_node
                    .attribute("d")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
            });
        }

        profile
    }

    fn parse_lane_section(&self, node: &roxmltree::Node) -> LaneSection {
        let mut section = LaneSection {
            s: node
                .attribute("s")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            ..Default::default()
        };

        // Parse left lanes
        if let Some(left_node) = node.children().find(|n| n.tag_name().name() == "left") {
            for lane_node in left_node
                .children()
                .filter(|n| n.tag_name().name() == "lane")
            {
                if let Ok(lane) = self.parse_lane(&lane_node) {
                    section.left.push(lane);
                }
            }
        }

        // Parse center lane
        if let Some(center_node) = node.children().find(|n| n.tag_name().name() == "center") {
            if let Some(lane_node) = center_node
                .children()
                .find(|n| n.tag_name().name() == "lane")
            {
                if let Ok(lane) = self.parse_lane(&lane_node) {
                    section.center = Some(lane);
                }
            }
        }

        // Parse right lanes
        if let Some(right_node) = node.children().find(|n| n.tag_name().name() == "right") {
            for lane_node in right_node
                .children()
                .filter(|n| n.tag_name().name() == "lane")
            {
                if let Ok(lane) = self.parse_lane(&lane_node) {
                    section.right.push(lane);
                }
            }
        }

        section
    }

    fn parse_lane(&self, node: &roxmltree::Node) -> Result<Lane> {
        let id = node
            .attribute("id")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0);
        let lane_type = match node.attribute("type") {
            Some("driving") => LaneType::Driving,
            Some("stop") => LaneType::Stop,
            Some("shoulder") => LaneType::Shoulder,
            Some("biking") => LaneType::Biking,
            Some("sidewalk") => LaneType::Sidewalk,
            Some("border") => LaneType::Border,
            Some("restricted") => LaneType::Restricted,
            Some("parking") => LaneType::Parking,
            Some("bidirectional") => LaneType::Bidirectional,
            Some("median") => LaneType::Median,
            Some("entry") => LaneType::Entry,
            Some("exit") => LaneType::Exit,
            Some("offRamp") => LaneType::OffRamp,
            Some("onRamp") => LaneType::OnRamp,
            Some("none") => LaneType::None,
            _ => LaneType::Driving,
        };

        let level = node
            .attribute("level")
            .map(|s| s == "true")
            .unwrap_or(false);

        let mut lane = Lane {
            id,
            lane_type,
            level,
            width: Vec::new(),
            road_marks: Vec::new(),
            speed_limit: None,
            link: LaneLink::default(),
            material: None,
        };

        // Parse width records
        for width_node in node.children().filter(|n| n.tag_name().name() == "width") {
            lane.width.push(LaneWidth {
                s_offset: width_node
                    .attribute("sOffset")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                a: width_node
                    .attribute("a")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                b: width_node
                    .attribute("b")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                c: width_node
                    .attribute("c")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                d: width_node
                    .attribute("d")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
            });
        }

        // Parse road marks
        for rm_node in node
            .children()
            .filter(|n| n.tag_name().name() == "roadMark")
        {
            lane.road_marks.push(self.parse_road_mark(&rm_node));
        }

        // Parse link
        if let Some(link_node) = node.children().find(|n| n.tag_name().name() == "link") {
            if let Some(pred) = link_node
                .children()
                .find(|n| n.tag_name().name() == "predecessor")
            {
                lane.link.predecessor = pred.attribute("id").and_then(|s| s.parse().ok());
            }
            if let Some(succ) = link_node
                .children()
                .find(|n| n.tag_name().name() == "successor")
            {
                lane.link.successor = succ.attribute("id").and_then(|s| s.parse().ok());
            }
        }

        // Parse speed
        if let Some(speed_node) = node.children().find(|n| n.tag_name().name() == "speed") {
            lane.speed_limit = speed_node.attribute("max").and_then(|s| s.parse().ok());
        }

        Ok(lane)
    }

    fn parse_road_mark(&self, node: &roxmltree::Node) -> RoadMark {
        RoadMark {
            s_offset: node
                .attribute("sOffset")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            mark_type: match node.attribute("type") {
                Some("solid") => RoadMarkType::Solid,
                Some("broken") => RoadMarkType::Broken,
                Some("solid solid") => RoadMarkType::SolidSolid,
                Some("solid broken") => RoadMarkType::SolidBroken,
                Some("broken solid") => RoadMarkType::BrokenSolid,
                Some("broken broken") => RoadMarkType::BrokenBroken,
                Some("botts dots") => RoadMarkType::BottsDots,
                Some("grass") => RoadMarkType::Grass,
                Some("curb") => RoadMarkType::Curb,
                Some("edge") => RoadMarkType::Edge,
                _ => RoadMarkType::None,
            },
            weight: match node.attribute("weight") {
                Some("bold") => RoadMarkWeight::Bold,
                _ => RoadMarkWeight::Standard,
            },
            color: match node.attribute("color") {
                Some("yellow") => RoadMarkColor::Yellow,
                Some("red") => RoadMarkColor::Red,
                Some("blue") => RoadMarkColor::Blue,
                Some("green") => RoadMarkColor::Green,
                _ => RoadMarkColor::White,
            },
            width: node
                .attribute("width")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.15),
            lane_change: match node.attribute("laneChange") {
                Some("increase") => LaneChange::Increase,
                Some("decrease") => LaneChange::Decrease,
                Some("none") => LaneChange::None,
                _ => LaneChange::Both,
            },
        }
    }

    fn parse_road_object(&self, node: &roxmltree::Node) -> Result<RoadObject> {
        Ok(RoadObject {
            id: node.attribute("id").unwrap_or("").to_string(),
            name: node.attribute("name").unwrap_or("").to_string(),
            s: node
                .attribute("s")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            t: node
                .attribute("t")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            z_offset: node
                .attribute("zOffset")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            object_type: node.attribute("type").unwrap_or("").to_string(),
            orientation: match node.attribute("orientation") {
                Some("-") => Orientation::Minus,
                Some("none") => Orientation::None,
                _ => Orientation::Plus,
            },
            width: node
                .attribute("width")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            length: node
                .attribute("length")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            height: node
                .attribute("height")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            hdg: node
                .attribute("hdg")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            pitch: node
                .attribute("pitch")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            roll: node
                .attribute("roll")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
        })
    }

    fn parse_signal(&self, node: &roxmltree::Node) -> Result<Signal> {
        Ok(Signal {
            id: node.attribute("id").unwrap_or("").to_string(),
            name: node.attribute("name").unwrap_or("").to_string(),
            s: node
                .attribute("s")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            t: node
                .attribute("t")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            z_offset: node
                .attribute("zOffset")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            signal_type: node.attribute("type").unwrap_or("").to_string(),
            subtype: node.attribute("subtype").unwrap_or("").to_string(),
            orientation: match node.attribute("orientation") {
                Some("-") => Orientation::Minus,
                Some("none") => Orientation::None,
                _ => Orientation::Plus,
            },
            dynamic: node
                .attribute("dynamic")
                .map(|s| s == "yes")
                .unwrap_or(false),
            value: node
                .attribute("value")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            width: node
                .attribute("width")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            height: node
                .attribute("height")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            country: node.attribute("country").unwrap_or("").to_string(),
        })
    }

    fn parse_surface(&self, node: &roxmltree::Node) -> Surface {
        let mut friction_zones = Vec::new();

        for crg_node in node.children().filter(|n| n.tag_name().name() == "CRG") {
            // CRG (curved regular grid) surfaces
            let s_start = crg_node
                .attribute("sStart")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            let s_end = crg_node
                .attribute("sEnd")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0);
            friction_zones.push(FrictionZone {
                s_start,
                s_end,
                friction: 0.8, // Default asphalt friction
            });
        }

        Surface { friction_zones }
    }

    fn parse_junction(&self, node: &roxmltree::Node) -> Result<Junction> {
        let mut junction = Junction {
            id: node.attribute("id").unwrap_or("").to_string(),
            name: node.attribute("name").unwrap_or("").to_string(),
            junction_type: match node.attribute("type") {
                Some("virtual") => JunctionType::Virtual,
                Some("direct") => JunctionType::Direct,
                _ => JunctionType::Default,
            },
            connections: Vec::new(),
        };

        for conn_node in node
            .children()
            .filter(|n| n.tag_name().name() == "connection")
        {
            let mut connection = JunctionConnection {
                id: conn_node.attribute("id").unwrap_or("").to_string(),
                incoming_road: conn_node
                    .attribute("incomingRoad")
                    .unwrap_or("")
                    .to_string(),
                connecting_road: conn_node
                    .attribute("connectingRoad")
                    .unwrap_or("")
                    .to_string(),
                contact_point: match conn_node.attribute("contactPoint") {
                    Some("end") => ContactPoint::End,
                    _ => ContactPoint::Start,
                },
                lane_links: Vec::new(),
            };

            for link_node in conn_node
                .children()
                .filter(|n| n.tag_name().name() == "laneLink")
            {
                if let (Some(from), Some(to)) = (
                    link_node.attribute("from").and_then(|s| s.parse().ok()),
                    link_node.attribute("to").and_then(|s| s.parse().ok()),
                ) {
                    connection.lane_links.push(JunctionLaneLink { from, to });
                }
            }

            junction.connections.push(connection);
        }

        Ok(junction)
    }

    fn parse_controller(&self, node: &roxmltree::Node) -> Result<Controller> {
        let mut controller = Controller {
            id: node.attribute("id").unwrap_or("").to_string(),
            name: node.attribute("name").unwrap_or("").to_string(),
            sequence: node
                .attribute("sequence")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0),
            controls: Vec::new(),
        };

        for control_node in node.children().filter(|n| n.tag_name().name() == "control") {
            controller.controls.push(ControlEntry {
                signal_id: control_node.attribute("signalId").unwrap_or("").to_string(),
                control_type: control_node.attribute("type").unwrap_or("").to_string(),
            });
        }

        Ok(controller)
    }

    fn resolve_path(&self, path: &Path) -> PathBuf {
        if path.is_absolute() {
            path.to_path_buf()
        } else {
            self.base_path.join(path)
        }
    }
}

impl Default for OpenDRIVELoader {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Road Mesh Generator
// ============================================================================

/// Generates road meshes from OpenDRIVE data
pub struct RoadMeshGenerator {
    /// Resolution for sampling (points per meter)
    pub resolution: f32,
}

impl RoadMeshGenerator {
    /// Create a new road mesh generator
    pub fn new() -> Self {
        Self { resolution: 1.0 }
    }

    /// Set sampling resolution
    pub fn with_resolution(mut self, resolution: f32) -> Self {
        self.resolution = resolution;
        self
    }

    /// Generate mesh for entire road network
    pub fn generate_network(&self, network: &OpenDRIVENetwork) -> Vec<RoadMesh> {
        let mut meshes = Vec::new();

        for road in &network.roads {
            if let Some(mesh) = self.generate_road(road) {
                meshes.push(mesh);
            }
        }

        meshes
    }

    /// Generate mesh for a single road
    pub fn generate_road(&self, road: &Road) -> Option<RoadMesh> {
        if road.plan_view.is_empty() {
            return None;
        }

        // Sample reference line
        let num_samples = (road.length * self.resolution as f64).ceil() as usize + 1;
        let reference_points: Vec<RoadPoint> = (0..num_samples)
            .map(|i| {
                let s = (i as f64 / (num_samples - 1) as f64) * road.length;
                self.sample_road_point(road, s)
            })
            .collect();

        // Calculate road width at each point
        let half_widths: Vec<(f64, f64)> = reference_points
            .iter()
            .map(|p| self.get_road_width(road, p.s))
            .collect();

        // Generate vertices
        let mut positions = Vec::with_capacity(num_samples * 2);
        let mut normals = Vec::with_capacity(num_samples * 2);
        let mut uvs = Vec::with_capacity(num_samples * 2);

        for (i, (point, (left_w, right_w))) in
            reference_points.iter().zip(half_widths.iter()).enumerate()
        {
            // Calculate perpendicular direction
            let perp = Vec2::new(-point.direction.y, point.direction.x);

            // Left edge
            let left_pos = Vec3::new(
                (point.x as f32) + perp.x * (*left_w as f32),
                point.z as f32,
                (point.y as f32) + perp.y * (*left_w as f32),
            );
            positions.push([left_pos.x, left_pos.y, left_pos.z]);
            normals.push([0.0, 1.0, 0.0]);
            uvs.push([0.0, i as f32 / (num_samples - 1) as f32]);

            // Right edge
            let right_pos = Vec3::new(
                (point.x as f32) - perp.x * (*right_w as f32),
                point.z as f32,
                (point.y as f32) - perp.y * (*right_w as f32),
            );
            positions.push([right_pos.x, right_pos.y, right_pos.z]);
            normals.push([0.0, 1.0, 0.0]);
            uvs.push([1.0, i as f32 / (num_samples - 1) as f32]);
        }

        // Generate indices
        let mut indices = Vec::with_capacity((num_samples - 1) * 6);
        for i in 0..(num_samples - 1) {
            let i0 = (i * 2) as u32;
            let i1 = i0 + 1;
            let i2 = i0 + 2;
            let i3 = i0 + 3;

            indices.extend_from_slice(&[i0, i2, i1, i1, i2, i3]);
        }

        let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, default());
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.insert_indices(Indices::U32(indices));

        Some(RoadMesh {
            road_id: road.id.clone(),
            mesh,
            reference_points,
        })
    }

    /// Sample a point on the road reference line
    fn sample_road_point(&self, road: &Road, s: f64) -> RoadPoint {
        // Find the geometry segment containing s
        let geom = road
            .plan_view
            .iter()
            .rev()
            .find(|g| g.s <= s)
            .unwrap_or(&road.plan_view[0]);

        let ds = s - geom.s;

        // Calculate position based on geometry type
        let (local_x, local_y, hdg_offset) = match &geom.geometry_type {
            GeometryType::Line => (ds, 0.0, 0.0),
            GeometryType::Arc { curvature } => {
                if curvature.abs() < 1e-10 {
                    (ds, 0.0, 0.0)
                } else {
                    let radius = 1.0 / curvature;
                    let angle = ds * curvature;
                    let x = radius * angle.sin();
                    let y = radius * (1.0 - angle.cos());
                    (x, y, angle)
                }
            }
            GeometryType::Spiral {
                curv_start,
                curv_end,
            } => {
                // Simplified Euler spiral (clothoid) approximation
                let curvature = curv_start + (curv_end - curv_start) * ds / geom.length;
                let angle = ds * (curv_start + curvature) / 2.0;
                let x = ds * angle.cos();
                let y = ds * angle.sin();
                (x, y, angle)
            }
            GeometryType::Poly3 { a, b, c, d } => {
                let v = a + b * ds + c * ds * ds + d * ds * ds * ds;
                let dv = b + 2.0 * c * ds + 3.0 * d * ds * ds;
                (ds, v, dv.atan())
            }
            GeometryType::ParamPoly3 {
                au,
                bu,
                cu,
                du,
                av,
                bv,
                cv,
                dv,
                p_range,
            } => {
                let p = match p_range {
                    ParamRange::ArcLength => ds,
                    ParamRange::Normalized => ds / geom.length,
                };
                let u = au + bu * p + cu * p * p + du * p * p * p;
                let v = av + bv * p + cv * p * p + dv * p * p * p;
                let du_dp = bu + 2.0 * cu * p + 3.0 * du * p * p;
                let dv_dp = bv + 2.0 * cv * p + 3.0 * dv * p * p;
                (u, v, dv_dp.atan2(du_dp))
            }
        };

        // Transform to global coordinates
        let cos_hdg = geom.hdg.cos();
        let sin_hdg = geom.hdg.sin();
        let x = geom.x + local_x * cos_hdg - local_y * sin_hdg;
        let y = geom.y + local_x * sin_hdg + local_y * cos_hdg;
        let hdg = geom.hdg + hdg_offset;

        // Calculate elevation
        let z = self.sample_elevation(road, s);

        RoadPoint {
            s,
            x,
            y,
            z,
            hdg,
            direction: Vec2::new(hdg.cos() as f32, hdg.sin() as f32),
        }
    }

    /// Sample elevation at s coordinate
    fn sample_elevation(&self, road: &Road, s: f64) -> f64 {
        if road.elevation_profile.is_empty() {
            return 0.0;
        }

        let elev = road
            .elevation_profile
            .iter()
            .rev()
            .find(|e| e.s <= s)
            .unwrap_or(&road.elevation_profile[0]);

        let ds = s - elev.s;
        elev.a + elev.b * ds + elev.c * ds * ds + elev.d * ds * ds * ds
    }

    /// Get road width (left and right) at s coordinate
    fn get_road_width(&self, road: &Road, s: f64) -> (f64, f64) {
        let ds = s - road.lanes.s;

        // Calculate left width
        let left_width: f64 = road
            .lanes
            .left
            .iter()
            .filter(|lane| lane.lane_type != LaneType::None)
            .map(|lane| self.sample_lane_width(lane, ds))
            .sum();

        // Calculate right width
        let right_width: f64 = road
            .lanes
            .right
            .iter()
            .filter(|lane| lane.lane_type != LaneType::None)
            .map(|lane| self.sample_lane_width(lane, ds))
            .sum();

        (left_width.max(1.5), right_width.max(1.5))
    }

    /// Sample lane width at offset
    fn sample_lane_width(&self, lane: &Lane, ds: f64) -> f64 {
        if lane.width.is_empty() {
            return 3.5; // Default lane width
        }

        let width_rec = lane
            .width
            .iter()
            .rev()
            .find(|w| w.s_offset <= ds)
            .unwrap_or(&lane.width[0]);

        let local_ds = ds - width_rec.s_offset;
        (width_rec.a
            + width_rec.b * local_ds
            + width_rec.c * local_ds * local_ds
            + width_rec.d * local_ds * local_ds * local_ds)
            .abs()
    }
}

impl Default for RoadMeshGenerator {
    fn default() -> Self {
        Self::new()
    }
}

/// Point on road reference line
#[derive(Debug, Clone)]
pub struct RoadPoint {
    /// S coordinate
    pub s: f64,
    /// X position
    pub x: f64,
    /// Y position
    pub y: f64,
    /// Z position (elevation)
    pub z: f64,
    /// Heading angle
    pub hdg: f64,
    /// Direction vector
    pub direction: Vec2,
}

/// Generated road mesh
pub struct RoadMesh {
    /// Road ID
    pub road_id: String,
    /// Generated mesh
    pub mesh: Mesh,
    /// Reference line points
    pub reference_points: Vec<RoadPoint>,
}

// ============================================================================
// Bevy Components
// ============================================================================

/// Component marking an entity as OpenDRIVE road network
#[derive(Component)]
pub struct OpenDRIVERoadNetwork {
    /// Network data
    pub network: OpenDRIVENetwork,
    /// Road ID to entity mapping
    pub road_entities: HashMap<String, Entity>,
}

/// Component for individual road
#[derive(Component)]
pub struct OpenDRIVERoad {
    /// Road ID
    pub road_id: String,
    /// Road length
    pub length: f64,
}

// ============================================================================
// Bevy Plugin
// ============================================================================

/// Plugin for OpenDRIVE road networks
pub struct OpenDRIVEPlugin;

impl Plugin for OpenDRIVEPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<LoadOpenDRIVEEvent>()
            .add_systems(Update, handle_load_opendrive);
    }
}

/// Event to load OpenDRIVE network
#[derive(Event)]
pub struct LoadOpenDRIVEEvent {
    /// Path to .xodr file
    pub path: PathBuf,
    /// Mesh resolution
    pub resolution: f32,
}

fn handle_load_opendrive(
    mut events: EventReader<LoadOpenDRIVEEvent>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for event in events.read() {
        let loader =
            OpenDRIVELoader::new().with_base_path(event.path.parent().unwrap_or(Path::new(".")));

        let network = match loader.load(&event.path) {
            Ok(n) => n,
            Err(e) => {
                error!("Failed to load OpenDRIVE: {}", e);
                continue;
            }
        };

        let generator = RoadMeshGenerator::new().with_resolution(event.resolution);
        let road_meshes = generator.generate_network(&network);

        // Create road material
        let road_material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.22),
            perceptual_roughness: 0.9,
            metallic: 0.0,
            ..default()
        });

        let mut road_entities = HashMap::new();

        // Spawn road entities
        for road_mesh in road_meshes {
            let mesh_handle = meshes.add(road_mesh.mesh);
            let entity = commands
                .spawn((
                    OpenDRIVERoad {
                        road_id: road_mesh.road_id.clone(),
                        length: road_mesh
                            .reference_points
                            .last()
                            .map(|p| p.s)
                            .unwrap_or(0.0),
                    },
                    Mesh3d(mesh_handle),
                    MeshMaterial3d(road_material.clone()),
                    Transform::default(),
                    GlobalTransform::default(),
                    Name::new(format!("Road_{}", road_mesh.road_id)),
                ))
                .id();

            road_entities.insert(road_mesh.road_id, entity);
        }

        // Spawn network entity
        commands.spawn((
            OpenDRIVERoadNetwork {
                network,
                road_entities,
            },
            Transform::default(),
            GlobalTransform::default(),
            Name::new("OpenDRIVE_Network"),
        ));

        info!("OpenDRIVE network loaded successfully");
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_road() {
        let xml = r#"<?xml version="1.0"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="6" name="Test" version="1.0"/>
    <road id="1" name="TestRoad" length="100.0" junction="-1">
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
                <line/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.0">
                <center>
                    <lane id="0" type="driving" level="false"/>
                </center>
                <right>
                    <lane id="-1" type="driving" level="false">
                        <width sOffset="0" a="3.5" b="0" c="0" d="0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
</OpenDRIVE>"#;

        let loader = OpenDRIVELoader::new();
        let network = loader.parse(xml, Path::new("test.xodr")).unwrap();

        assert_eq!(network.header.name, "Test");
        assert_eq!(network.roads.len(), 1);
        assert_eq!(network.roads[0].id, "1");
        assert_eq!(network.roads[0].length, 100.0);
    }

    #[test]
    fn test_geometry_sampling() {
        let road = Road {
            id: "1".to_string(),
            name: "Test".to_string(),
            length: 100.0,
            junction: -1,
            rule: TrafficRule::RHT,
            plan_view: vec![Geometry {
                s: 0.0,
                x: 0.0,
                y: 0.0,
                hdg: 0.0,
                length: 100.0,
                geometry_type: GeometryType::Line,
            }],
            elevation_profile: vec![],
            lateral_profile: LateralProfile::default(),
            lanes: LaneSection::default(),
            objects: vec![],
            signals: vec![],
            surface: None,
            link: RoadLink::default(),
        };

        let generator = RoadMeshGenerator::new();
        let point = generator.sample_road_point(&road, 50.0);

        assert!((point.x - 50.0).abs() < 0.001);
        assert!((point.y).abs() < 0.001);
    }

    #[test]
    fn test_arc_geometry() {
        let road = Road {
            id: "1".to_string(),
            name: "Test".to_string(),
            length: std::f64::consts::PI * 10.0, // Quarter circle with radius 10
            junction: -1,
            rule: TrafficRule::RHT,
            plan_view: vec![Geometry {
                s: 0.0,
                x: 0.0,
                y: 0.0,
                hdg: 0.0,
                length: std::f64::consts::PI * 10.0 / 2.0,
                geometry_type: GeometryType::Arc { curvature: 0.1 },
            }],
            elevation_profile: vec![],
            lateral_profile: LateralProfile::default(),
            lanes: LaneSection::default(),
            objects: vec![],
            signals: vec![],
            surface: None,
            link: RoadLink::default(),
        };

        let generator = RoadMeshGenerator::new();
        let point = generator.sample_road_point(&road, std::f64::consts::PI * 5.0);

        // After quarter circle, should be at approximately (10, 10)
        assert!((point.x - 10.0).abs() < 0.1);
        assert!((point.y - 10.0).abs() < 0.1);
    }
}
