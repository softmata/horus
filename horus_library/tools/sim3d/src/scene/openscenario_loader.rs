//! OpenSCENARIO (.xosc) Scenario Scripting Loader
//!
//! Supports loading scenarios from OpenSCENARIO format (ASAM OpenSCENARIO 1.0/1.1/1.2/2.0)
//! for autonomous driving scenario simulation.
//!
//! Features:
//! - Storyboard with acts, maneuver groups, and maneuvers
//! - Entity actions (vehicle, pedestrian, misc object)
//! - Conditions and triggers
//! - Parameter declarations and variations
//! - Trajectory following and routing
//! - Traffic signal state changes

use bevy::prelude::*;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

use crate::error::{EnhancedError, ErrorCategory, Result};

// ============================================================================
// OpenSCENARIO Data Structures
// ============================================================================

/// Complete OpenSCENARIO scenario
#[derive(Debug, Clone)]
pub struct OpenSCENARIOScenario {
    /// File header
    pub file_header: FileHeader,
    /// Parameter declarations
    pub parameter_declarations: Vec<ParameterDeclaration>,
    /// Catalog locations
    pub catalog_locations: CatalogLocations,
    /// Road network reference
    pub road_network: RoadNetwork,
    /// Entity definitions
    pub entities: Vec<ScenarioEntity>,
    /// Storyboard with scenario logic
    pub storyboard: Storyboard,
}

/// File header information
#[derive(Debug, Clone, Default)]
pub struct FileHeader {
    pub rev_major: u32,
    pub rev_minor: u32,
    pub date: String,
    pub description: String,
    pub author: String,
}

/// Parameter declaration
#[derive(Debug, Clone)]
pub struct ParameterDeclaration {
    pub name: String,
    pub parameter_type: ParameterType,
    pub value: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterType {
    Integer,
    Double,
    String,
    UnsignedInt,
    UnsignedShort,
    Boolean,
    DateTime,
}

/// Catalog locations
#[derive(Debug, Clone, Default)]
pub struct CatalogLocations {
    pub vehicle_catalog: Option<PathBuf>,
    pub pedestrian_catalog: Option<PathBuf>,
    pub misc_object_catalog: Option<PathBuf>,
    pub environment_catalog: Option<PathBuf>,
    pub maneuver_catalog: Option<PathBuf>,
    pub trajectory_catalog: Option<PathBuf>,
    pub route_catalog: Option<PathBuf>,
    pub controller_catalog: Option<PathBuf>,
}

/// Road network reference
#[derive(Debug, Clone, Default)]
pub struct RoadNetwork {
    pub logic_file: Option<PathBuf>,
    pub scene_graph_file: Option<PathBuf>,
    pub traffic_signals: Vec<TrafficSignalController>,
}

/// Traffic signal controller
#[derive(Debug, Clone)]
pub struct TrafficSignalController {
    pub name: String,
    pub delay: f64,
    pub reference: String,
    pub phases: Vec<TrafficSignalPhase>,
}

/// Traffic signal phase
#[derive(Debug, Clone)]
pub struct TrafficSignalPhase {
    pub duration: f64,
    pub state: Vec<TrafficSignalState>,
}

/// Traffic signal state
#[derive(Debug, Clone)]
pub struct TrafficSignalState {
    pub traffic_signal_id: String,
    pub state: String,
}

/// Scenario entity (vehicle, pedestrian, misc object)
#[derive(Debug, Clone)]
pub struct ScenarioEntity {
    pub name: String,
    pub entity_type: EntityType,
    pub object_ref: Option<String>,
    pub vehicle: Option<Vehicle>,
    pub pedestrian: Option<Pedestrian>,
    pub misc_object: Option<MiscObject>,
    pub controller: Option<EntityController>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityType {
    Vehicle,
    Pedestrian,
    MiscObject,
}

/// Vehicle definition
#[derive(Debug, Clone)]
pub struct Vehicle {
    pub name: String,
    pub vehicle_category: VehicleCategory,
    pub bounding_box: BoundingBox,
    pub performance: Performance,
    pub axles: Axles,
    pub properties: HashMap<String, String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VehicleCategory {
    #[default]
    Car,
    Van,
    Truck,
    Trailer,
    Semitrailer,
    Bus,
    Motorbike,
    Bicycle,
    Train,
    Tram,
}

/// Pedestrian definition
#[derive(Debug, Clone)]
pub struct Pedestrian {
    pub name: String,
    pub model: String,
    pub mass: f64,
    pub pedestrian_category: PedestrianCategory,
    pub bounding_box: BoundingBox,
    pub properties: HashMap<String, String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PedestrianCategory {
    #[default]
    Pedestrian,
    Wheelchair,
    Animal,
}

/// Miscellaneous object definition
#[derive(Debug, Clone)]
pub struct MiscObject {
    pub name: String,
    pub misc_object_category: MiscObjectCategory,
    pub mass: f64,
    pub bounding_box: BoundingBox,
    pub properties: HashMap<String, String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MiscObjectCategory {
    #[default]
    None,
    Obstacle,
    Pole,
    Tree,
    Vegetation,
    Barrier,
    Building,
    ParkingSpace,
    Patch,
    Railing,
    TrafficIsland,
    Crosswalk,
    StreetLamp,
    Gantry,
    SoundBarrier,
    Wind,
    RoadMark,
}

/// Bounding box
#[derive(Debug, Clone, Default)]
pub struct BoundingBox {
    pub center: Position3D,
    pub dimensions: Dimensions,
}

/// 3D position
#[derive(Debug, Clone, Default)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// Dimensions
#[derive(Debug, Clone, Default)]
pub struct Dimensions {
    pub width: f64,
    pub length: f64,
    pub height: f64,
}

/// Vehicle performance
#[derive(Debug, Clone, Default)]
pub struct Performance {
    pub max_speed: f64,
    pub max_acceleration: f64,
    pub max_deceleration: f64,
}

/// Vehicle axles
#[derive(Debug, Clone, Default)]
pub struct Axles {
    pub front_axle: Axle,
    pub rear_axle: Axle,
    pub additional_axles: Vec<Axle>,
}

/// Axle definition
#[derive(Debug, Clone, Default)]
pub struct Axle {
    pub max_steering: f64,
    pub wheel_diameter: f64,
    pub track_width: f64,
    pub position_x: f64,
    pub position_z: f64,
}

/// Entity controller
#[derive(Debug, Clone)]
pub struct EntityController {
    pub name: String,
    pub controller_type: ControllerType,
    pub properties: HashMap<String, String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ControllerType {
    #[default]
    Default,
    External,
    Override,
}

// ============================================================================
// Storyboard
// ============================================================================

/// Storyboard containing scenario logic
#[derive(Debug, Clone)]
pub struct Storyboard {
    pub init: Init,
    pub stories: Vec<Story>,
    pub stop_trigger: Option<Trigger>,
}

/// Initialization actions
#[derive(Debug, Clone, Default)]
pub struct Init {
    pub actions: Vec<InitAction>,
}

/// Initialization action
#[derive(Debug, Clone)]
pub struct InitAction {
    pub entity_ref: Option<String>,
    pub global_action: Option<GlobalAction>,
    pub private_action: Option<PrivateAction>,
}

/// Story definition
#[derive(Debug, Clone)]
pub struct Story {
    pub name: String,
    pub acts: Vec<Act>,
}

/// Act within a story
#[derive(Debug, Clone)]
pub struct Act {
    pub name: String,
    pub maneuver_groups: Vec<ManeuverGroup>,
    pub start_trigger: Option<Trigger>,
    pub stop_trigger: Option<Trigger>,
}

/// Maneuver group
#[derive(Debug, Clone)]
pub struct ManeuverGroup {
    pub name: String,
    pub maximum_execution_count: u32,
    pub actors: Vec<String>,
    pub maneuvers: Vec<Maneuver>,
}

/// Maneuver definition
#[derive(Debug, Clone)]
pub struct Maneuver {
    pub name: String,
    pub events: Vec<Event>,
}

/// Event within a maneuver
#[derive(Debug, Clone)]
pub struct Event {
    pub name: String,
    pub priority: EventPriority,
    pub maximum_execution_count: u32,
    pub actions: Vec<EventAction>,
    pub start_trigger: Option<Trigger>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum EventPriority {
    #[default]
    Overwrite,
    Skip,
    Parallel,
}

/// Event action
#[derive(Debug, Clone)]
pub struct EventAction {
    pub name: String,
    pub global_action: Option<GlobalAction>,
    pub private_action: Option<PrivateAction>,
    pub user_defined_action: Option<UserDefinedAction>,
}

// ============================================================================
// Actions
// ============================================================================

/// Global action (affects world state)
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
pub enum GlobalAction {
    Environment(EnvironmentAction),
    EntityAction(EntityAction),
    ParameterAction(ParameterAction),
    InfrastructureAction(InfrastructureAction),
    TrafficAction(TrafficAction),
}

/// Environment action
#[derive(Debug, Clone)]
pub struct EnvironmentAction {
    pub environment: Environment,
}

/// Environment definition
#[derive(Debug, Clone, Default)]
pub struct Environment {
    pub name: String,
    pub time_of_day: Option<TimeOfDay>,
    pub weather: Option<Weather>,
    pub road_condition: Option<RoadCondition>,
}

#[derive(Debug, Clone)]
pub struct TimeOfDay {
    pub animation: bool,
    pub date_time: String,
}

#[derive(Debug, Clone, Default)]
pub struct Weather {
    pub cloud_state: CloudState,
    pub sun: Option<Sun>,
    pub fog: Option<Fog>,
    pub precipitation: Option<Precipitation>,
    pub wind: Option<Wind>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CloudState {
    #[default]
    Free,
    Cloudy,
    Overcast,
    Rainy,
    SkyOff,
}

#[derive(Debug, Clone)]
pub struct Sun {
    pub intensity: f64,
    pub azimuth: f64,
    pub elevation: f64,
}

#[derive(Debug, Clone)]
pub struct Fog {
    pub visual_range: f64,
}

#[derive(Debug, Clone)]
pub struct Precipitation {
    pub precipitation_type: PrecipitationType,
    pub intensity: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PrecipitationType {
    #[default]
    Dry,
    Rain,
    Snow,
}

#[derive(Debug, Clone)]
pub struct Wind {
    pub direction: f64,
    pub speed: f64,
}

#[derive(Debug, Clone)]
pub struct RoadCondition {
    pub friction_scale_factor: f64,
}

/// Entity action
#[derive(Debug, Clone)]
pub struct EntityAction {
    pub entity_ref: String,
    pub action_type: EntityActionType,
}

#[derive(Debug, Clone)]
pub enum EntityActionType {
    Add(AddEntityAction),
    Delete,
}

#[derive(Debug, Clone)]
pub struct AddEntityAction {
    pub position: Position,
}

/// Parameter action
#[derive(Debug, Clone)]
pub struct ParameterAction {
    pub parameter_ref: String,
    pub value: String,
}

/// Infrastructure action
#[derive(Debug, Clone)]
pub struct InfrastructureAction {
    pub traffic_signal_action: Option<TrafficSignalAction>,
}

#[derive(Debug, Clone)]
pub struct TrafficSignalAction {
    pub traffic_signal_controller_ref: String,
    pub phase: String,
}

/// Traffic action
#[derive(Debug, Clone)]
pub struct TrafficAction {
    pub traffic_source_action: Option<TrafficSourceAction>,
    pub traffic_sink_action: Option<TrafficSinkAction>,
    pub traffic_swarm_action: Option<TrafficSwarmAction>,
}

#[derive(Debug, Clone)]
pub struct TrafficSourceAction {
    pub rate: f64,
    pub radius: f64,
    pub position: Position,
    pub vehicle_category_distribution: Vec<VehicleCategoryDistribution>,
}

#[derive(Debug, Clone)]
pub struct TrafficSinkAction {
    pub rate: f64,
    pub radius: f64,
    pub position: Position,
}

#[derive(Debug, Clone)]
pub struct TrafficSwarmAction {
    pub semi_major_axis: f64,
    pub semi_minor_axis: f64,
    pub inner_radius: f64,
    pub offset: f64,
    pub number_of_vehicles: u32,
    pub velocity: f64,
    pub central_object_ref: String,
}

#[derive(Debug, Clone)]
pub struct VehicleCategoryDistribution {
    pub category: VehicleCategory,
    pub weight: f64,
}

/// Private action (affects specific entity)
#[derive(Debug, Clone)]
pub enum PrivateAction {
    Longitudinal(LongitudinalAction),
    Lateral(LateralAction),
    Visibility(VisibilityAction),
    Synchronize(SynchronizeAction),
    Activate(ActivateController),
    Controller(AssignController),
    Teleport(TeleportAction),
    Routing(RoutingAction),
}

/// Longitudinal action
#[derive(Debug, Clone)]
pub struct LongitudinalAction {
    pub speed_action: Option<SpeedAction>,
    pub longitudinal_distance_action: Option<LongitudinalDistanceAction>,
}

#[derive(Debug, Clone)]
pub struct SpeedAction {
    pub speed_action_dynamics: TransitionDynamics,
    pub speed_action_target: SpeedActionTarget,
}

#[derive(Debug, Clone)]
pub struct TransitionDynamics {
    pub dynamics_shape: DynamicsShape,
    pub value: f64,
    pub dynamics_dimension: DynamicsDimension,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DynamicsShape {
    #[default]
    Linear,
    Cubic,
    Sinusoidal,
    Step,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DynamicsDimension {
    #[default]
    Rate,
    Time,
    Distance,
}

#[derive(Debug, Clone)]
pub enum SpeedActionTarget {
    Absolute(AbsoluteTargetSpeed),
    Relative(RelativeTargetSpeed),
}

#[derive(Debug, Clone)]
pub struct AbsoluteTargetSpeed {
    pub value: f64,
}

#[derive(Debug, Clone)]
pub struct RelativeTargetSpeed {
    pub entity_ref: String,
    pub value: f64,
    pub speed_target_value_type: SpeedTargetValueType,
    pub continuous: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpeedTargetValueType {
    #[default]
    Delta,
    Factor,
}

#[derive(Debug, Clone)]
pub struct LongitudinalDistanceAction {
    pub entity_ref: String,
    pub distance: Option<f64>,
    pub time_gap: Option<f64>,
    pub freespace: bool,
    pub continuous: bool,
    pub displacement: Option<LongitudinalDisplacement>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LongitudinalDisplacement {
    #[default]
    Any,
    LeadingReferencedEntity,
    TrailingReferencedEntity,
}

/// Lateral action
#[derive(Debug, Clone)]
pub struct LateralAction {
    pub lane_change_action: Option<LaneChangeAction>,
    pub lane_offset_action: Option<LaneOffsetAction>,
    pub lateral_distance_action: Option<LateralDistanceAction>,
}

#[derive(Debug, Clone)]
pub struct LaneChangeAction {
    pub lane_change_action_dynamics: TransitionDynamics,
    pub lane_change_target: LaneChangeTarget,
}

#[derive(Debug, Clone)]
pub enum LaneChangeTarget {
    Absolute(AbsoluteTargetLane),
    Relative(RelativeTargetLane),
}

#[derive(Debug, Clone)]
pub struct AbsoluteTargetLane {
    pub value: String,
}

#[derive(Debug, Clone)]
pub struct RelativeTargetLane {
    pub entity_ref: String,
    pub value: i32,
}

#[derive(Debug, Clone)]
pub struct LaneOffsetAction {
    pub lane_offset_action_dynamics: LaneOffsetActionDynamics,
    pub lane_offset_target: LaneOffsetTarget,
}

#[derive(Debug, Clone)]
pub struct LaneOffsetActionDynamics {
    pub dynamics_shape: DynamicsShape,
    pub max_lateral_acc: Option<f64>,
}

#[derive(Debug, Clone)]
pub enum LaneOffsetTarget {
    Absolute(AbsoluteTargetLaneOffset),
    Relative(RelativeTargetLaneOffset),
}

#[derive(Debug, Clone)]
pub struct AbsoluteTargetLaneOffset {
    pub value: f64,
}

#[derive(Debug, Clone)]
pub struct RelativeTargetLaneOffset {
    pub entity_ref: String,
    pub value: f64,
}

#[derive(Debug, Clone)]
pub struct LateralDistanceAction {
    pub entity_ref: String,
    pub distance: f64,
    pub freespace: bool,
    pub continuous: bool,
}

/// Visibility action
#[derive(Debug, Clone)]
pub struct VisibilityAction {
    pub graphics: bool,
    pub traffic: bool,
    pub sensors: bool,
}

/// Synchronize action
#[derive(Debug, Clone)]
pub struct SynchronizeAction {
    pub master_entity_ref: String,
    pub target_position_master: Position,
    pub target_position: Position,
    pub final_speed: Option<FinalSpeed>,
}

#[derive(Debug, Clone)]
pub struct FinalSpeed {
    pub absolute_speed: Option<AbsoluteSpeed>,
    pub relative_speed_to_master: Option<RelativeSpeedToMaster>,
}

#[derive(Debug, Clone)]
pub struct AbsoluteSpeed {
    pub value: f64,
}

#[derive(Debug, Clone)]
pub struct RelativeSpeedToMaster {
    pub value: f64,
    pub speed_target_value_type: SpeedTargetValueType,
}

/// Activate controller
#[derive(Debug, Clone)]
pub struct ActivateController {
    pub lateral: Option<bool>,
    pub longitudinal: Option<bool>,
}

/// Assign controller
#[derive(Debug, Clone)]
pub struct AssignController {
    pub controller: EntityController,
}

/// Teleport action
#[derive(Debug, Clone)]
pub struct TeleportAction {
    pub position: Position,
}

/// Routing action
#[derive(Debug, Clone)]
pub struct RoutingAction {
    pub assign_route_action: Option<AssignRouteAction>,
    pub follow_trajectory_action: Option<FollowTrajectoryAction>,
    pub acquire_position_action: Option<AcquirePositionAction>,
}

#[derive(Debug, Clone)]
pub struct AssignRouteAction {
    pub route: Route,
}

#[derive(Debug, Clone)]
pub struct Route {
    pub name: String,
    pub closed: bool,
    pub waypoints: Vec<Waypoint>,
}

#[derive(Debug, Clone)]
pub struct Waypoint {
    pub route_strategy: RouteStrategy,
    pub position: Position,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RouteStrategy {
    #[default]
    Fastest,
    Shortest,
    LeastIntersections,
    Random,
}

#[derive(Debug, Clone)]
pub struct FollowTrajectoryAction {
    pub trajectory: Trajectory,
    pub time_reference: Option<TimeReference>,
    pub trajectory_following_mode: TrajectoryFollowingMode,
}

#[derive(Debug, Clone)]
pub struct Trajectory {
    pub name: String,
    pub closed: bool,
    pub shape: TrajectoryShape,
}

#[derive(Debug, Clone)]
pub enum TrajectoryShape {
    Polyline(Polyline),
    Clothoid(Clothoid),
    Nurbs(Nurbs),
}

#[derive(Debug, Clone)]
pub struct Polyline {
    pub vertices: Vec<Vertex>,
}

#[derive(Debug, Clone)]
pub struct Vertex {
    pub time: Option<f64>,
    pub position: Position,
}

#[derive(Debug, Clone)]
pub struct Clothoid {
    pub curvature: f64,
    pub curvature_dot: f64,
    pub length: f64,
    pub start_time: Option<f64>,
    pub stop_time: Option<f64>,
    pub position: Position,
}

#[derive(Debug, Clone)]
pub struct Nurbs {
    pub order: u32,
    pub control_points: Vec<ControlPoint>,
    pub knots: Vec<f64>,
}

#[derive(Debug, Clone)]
pub struct ControlPoint {
    pub time: Option<f64>,
    pub weight: f64,
    pub position: Position,
}

#[derive(Debug, Clone)]
pub struct TimeReference {
    pub timing: Option<Timing>,
    pub none: bool,
}

#[derive(Debug, Clone)]
pub struct Timing {
    pub domain_absolute_relative: DomainAbsoluteRelative,
    pub scale: f64,
    pub offset: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DomainAbsoluteRelative {
    #[default]
    Absolute,
    Relative,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TrajectoryFollowingMode {
    #[default]
    Follow,
    Position,
}

#[derive(Debug, Clone)]
pub struct AcquirePositionAction {
    pub position: Position,
}

/// User-defined action
#[derive(Debug, Clone)]
pub struct UserDefinedAction {
    pub custom_command_action: CustomCommandAction,
}

#[derive(Debug, Clone)]
pub struct CustomCommandAction {
    pub command_type: String,
    pub content: String,
}

// ============================================================================
// Positions
// ============================================================================

/// Position types
#[derive(Debug, Clone)]
pub enum Position {
    World(WorldPosition),
    RelativeWorld(RelativeWorldPosition),
    RelativeObject(RelativeObjectPosition),
    Road(RoadPosition),
    RelativeRoad(RelativeRoadPosition),
    Lane(LanePosition),
    RelativeLane(RelativeLanePosition),
    Route(RoutePosition),
    Trajectory(TrajectoryPosition),
}

#[derive(Debug, Clone)]
pub struct WorldPosition {
    pub x: f64,
    pub y: f64,
    pub z: Option<f64>,
    pub h: Option<f64>,
    pub p: Option<f64>,
    pub r: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct RelativeWorldPosition {
    pub entity_ref: String,
    pub dx: f64,
    pub dy: f64,
    pub dz: Option<f64>,
    pub orientation: Option<Orientation>,
}

#[derive(Debug, Clone)]
pub struct RelativeObjectPosition {
    pub entity_ref: String,
    pub dx: f64,
    pub dy: f64,
    pub dz: Option<f64>,
    pub orientation: Option<Orientation>,
}

#[derive(Debug, Clone)]
pub struct RoadPosition {
    pub road_id: String,
    pub s: f64,
    pub t: f64,
    pub orientation: Option<Orientation>,
}

#[derive(Debug, Clone)]
pub struct RelativeRoadPosition {
    pub entity_ref: String,
    pub ds: f64,
    pub dt: f64,
    pub orientation: Option<Orientation>,
}

#[derive(Debug, Clone)]
pub struct LanePosition {
    pub road_id: String,
    pub lane_id: String,
    pub s: f64,
    pub offset: Option<f64>,
    pub orientation: Option<Orientation>,
}

#[derive(Debug, Clone)]
pub struct RelativeLanePosition {
    pub entity_ref: String,
    pub d_lane: i32,
    pub ds: f64,
    pub offset: Option<f64>,
    pub orientation: Option<Orientation>,
}

#[derive(Debug, Clone)]
pub struct RoutePosition {
    pub route_ref: String,
    pub orientation: Option<Orientation>,
    pub in_route_position: InRoutePosition,
}

#[derive(Debug, Clone)]
pub struct InRoutePosition {
    pub from_current_entity: Option<FromCurrentEntity>,
    pub from_road_coordinates: Option<FromRoadCoordinates>,
    pub from_lane_coordinates: Option<FromLaneCoordinates>,
}

#[derive(Debug, Clone)]
pub struct FromCurrentEntity {
    pub entity_ref: String,
}

#[derive(Debug, Clone)]
pub struct FromRoadCoordinates {
    pub path_s: f64,
    pub t: f64,
}

#[derive(Debug, Clone)]
pub struct FromLaneCoordinates {
    pub path_s: f64,
    pub lane_id: String,
    pub lane_offset: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct TrajectoryPosition {
    pub s: f64,
    pub t: Option<f64>,
    pub orientation: Option<Orientation>,
    pub trajectory_ref: String,
}

#[derive(Debug, Clone)]
pub struct Orientation {
    pub orientation_type: OrientationType,
    pub h: Option<f64>,
    pub p: Option<f64>,
    pub r: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OrientationType {
    #[default]
    Relative,
    Absolute,
}

// ============================================================================
// Triggers and Conditions
// ============================================================================

/// Trigger definition
#[derive(Debug, Clone)]
pub struct Trigger {
    pub condition_groups: Vec<ConditionGroup>,
}

/// Condition group (AND logic within group)
#[derive(Debug, Clone)]
pub struct ConditionGroup {
    pub conditions: Vec<Condition>,
}

/// Condition definition
#[derive(Debug, Clone)]
pub struct Condition {
    pub name: String,
    pub delay: f64,
    pub condition_edge: ConditionEdge,
    pub by_entity_condition: Option<ByEntityCondition>,
    pub by_value_condition: Option<ByValueCondition>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ConditionEdge {
    #[default]
    Rising,
    Falling,
    RisingOrFalling,
    None,
}

/// By-entity condition
#[derive(Debug, Clone)]
pub struct ByEntityCondition {
    pub triggering_entities: TriggeringEntities,
    pub entity_condition: EntityCondition,
}

#[derive(Debug, Clone)]
pub struct TriggeringEntities {
    pub rule: TriggeringEntitiesRule,
    pub entity_refs: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TriggeringEntitiesRule {
    #[default]
    Any,
    All,
}

#[derive(Debug, Clone)]
pub enum EntityCondition {
    EndOfRoad(EndOfRoadCondition),
    Collision(CollisionCondition),
    OffRoad(OffRoadCondition),
    TimeHeadway(TimeHeadwayCondition),
    TimeToCollision(TimeToCollisionCondition),
    Acceleration(AccelerationCondition),
    StandStill(StandStillCondition),
    Speed(SpeedCondition),
    RelativeSpeed(RelativeSpeedCondition),
    TraveledDistance(TraveledDistanceCondition),
    ReachPosition(ReachPositionCondition),
    Distance(DistanceCondition),
    RelativeDistance(RelativeDistanceCondition),
}

#[derive(Debug, Clone)]
pub struct EndOfRoadCondition {
    pub duration: f64,
}

#[derive(Debug, Clone)]
pub struct CollisionCondition {
    pub entity_ref: Option<String>,
    pub by_type: Option<EntityType>,
}

#[derive(Debug, Clone)]
pub struct OffRoadCondition {
    pub duration: f64,
}

#[derive(Debug, Clone)]
pub struct TimeHeadwayCondition {
    pub entity_ref: String,
    pub value: f64,
    pub freespace: bool,
    pub along_route: bool,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct TimeToCollisionCondition {
    pub value: f64,
    pub freespace: bool,
    pub along_route: bool,
    pub rule: Rule,
    pub time_to_collision_condition_target: TimeToCollisionConditionTarget,
}

#[derive(Debug, Clone)]
pub enum TimeToCollisionConditionTarget {
    Entity(String),
    Position(Position),
}

#[derive(Debug, Clone)]
pub struct AccelerationCondition {
    pub value: f64,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct StandStillCondition {
    pub duration: f64,
}

#[derive(Debug, Clone)]
pub struct SpeedCondition {
    pub value: f64,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct RelativeSpeedCondition {
    pub entity_ref: String,
    pub value: f64,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct TraveledDistanceCondition {
    pub value: f64,
}

#[derive(Debug, Clone)]
pub struct ReachPositionCondition {
    pub tolerance: f64,
    pub position: Position,
}

#[derive(Debug, Clone)]
pub struct DistanceCondition {
    pub value: f64,
    pub freespace: bool,
    pub along_route: bool,
    pub rule: Rule,
    pub position: Position,
}

#[derive(Debug, Clone)]
pub struct RelativeDistanceCondition {
    pub entity_ref: String,
    pub value: f64,
    pub freespace: bool,
    pub relative_distance_type: RelativeDistanceType,
    pub rule: Rule,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RelativeDistanceType {
    #[default]
    Longitudinal,
    Lateral,
    Euclidean,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Rule {
    #[default]
    GreaterThan,
    LessThan,
    EqualTo,
    GreaterOrEqual,
    LessOrEqual,
    NotEqualTo,
}

/// By-value condition
#[derive(Debug, Clone)]
#[allow(clippy::enum_variant_names)]
pub enum ByValueCondition {
    ParameterCondition(ParameterCondition),
    TimeOfDayCondition(TimeOfDayCondition),
    SimulationTimeCondition(SimulationTimeCondition),
    StoryboardElementStateCondition(StoryboardElementStateCondition),
    UserDefinedValueCondition(UserDefinedValueCondition),
    TrafficSignalCondition(TrafficSignalCondition),
    TrafficSignalControllerCondition(TrafficSignalControllerCondition),
}

#[derive(Debug, Clone)]
pub struct ParameterCondition {
    pub parameter_ref: String,
    pub value: String,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct TimeOfDayCondition {
    pub date_time: String,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct SimulationTimeCondition {
    pub value: f64,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct StoryboardElementStateCondition {
    pub storyboard_element_ref: String,
    pub storyboard_element_type: StoryboardElementType,
    pub state: StoryboardElementState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoryboardElementType {
    Story,
    Act,
    ManeuverGroup,
    Maneuver,
    Event,
    Action,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoryboardElementState {
    StartTransition,
    EndTransition,
    StopTransition,
    SkipTransition,
    CompleteState,
    RunningState,
    StandbyState,
}

#[derive(Debug, Clone)]
pub struct UserDefinedValueCondition {
    pub name: String,
    pub value: String,
    pub rule: Rule,
}

#[derive(Debug, Clone)]
pub struct TrafficSignalCondition {
    pub traffic_signal_id: String,
    pub state: String,
}

#[derive(Debug, Clone)]
pub struct TrafficSignalControllerCondition {
    pub traffic_signal_controller_ref: String,
    pub phase: String,
}

// ============================================================================
// OpenSCENARIO Loader
// ============================================================================

/// Loader for OpenSCENARIO files
pub struct OpenSCENARIOLoader {
    base_path: PathBuf,
}

impl OpenSCENARIOLoader {
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

    /// Load OpenSCENARIO from file
    pub fn load<P: AsRef<Path>>(&self, path: P) -> Result<OpenSCENARIOScenario> {
        let path = self.resolve_path(path.as_ref());
        let content = std::fs::read_to_string(&path).map_err(|e| {
            EnhancedError::new(format!("Failed to read OpenSCENARIO file: {}", e))
                .with_file(&path)
                .with_category(ErrorCategory::FileNotFound)
        })?;

        self.parse(&content, &path)
    }

    /// Parse OpenSCENARIO XML content
    pub fn parse(&self, content: &str, source_path: &Path) -> Result<OpenSCENARIOScenario> {
        let doc = roxmltree::Document::parse(content).map_err(|e| {
            EnhancedError::new(format!("Failed to parse OpenSCENARIO XML: {}", e))
                .with_file(source_path)
                .with_category(ErrorCategory::ParseError)
        })?;

        let root = doc.root_element();
        if root.tag_name().name() != "OpenSCENARIO" {
            return Err(EnhancedError::new("Not a valid OpenSCENARIO file")
                .with_file(source_path)
                .with_hint("Root element must be <OpenSCENARIO>"));
        }

        let mut scenario = OpenSCENARIOScenario {
            file_header: FileHeader::default(),
            parameter_declarations: Vec::new(),
            catalog_locations: CatalogLocations::default(),
            road_network: RoadNetwork::default(),
            entities: Vec::new(),
            storyboard: Storyboard {
                init: Init::default(),
                stories: Vec::new(),
                stop_trigger: None,
            },
        };

        // Parse FileHeader
        if let Some(header) = root
            .children()
            .find(|n| n.tag_name().name() == "FileHeader")
        {
            scenario.file_header = self.parse_file_header(&header);
        }

        // Parse ParameterDeclarations
        if let Some(params) = root
            .children()
            .find(|n| n.tag_name().name() == "ParameterDeclarations")
        {
            scenario.parameter_declarations = self.parse_parameter_declarations(&params);
        }

        // Parse CatalogLocations
        if let Some(catalogs) = root
            .children()
            .find(|n| n.tag_name().name() == "CatalogLocations")
        {
            scenario.catalog_locations = self.parse_catalog_locations(&catalogs);
        }

        // Parse RoadNetwork
        if let Some(road_net) = root
            .children()
            .find(|n| n.tag_name().name() == "RoadNetwork")
        {
            scenario.road_network = self.parse_road_network(&road_net);
        }

        // Parse Entities
        if let Some(entities) = root.children().find(|n| n.tag_name().name() == "Entities") {
            scenario.entities = self.parse_entities(&entities);
        }

        // Parse Storyboard
        if let Some(storyboard) = root
            .children()
            .find(|n| n.tag_name().name() == "Storyboard")
        {
            scenario.storyboard = self.parse_storyboard(&storyboard);
        }

        info!(
            "Loaded OpenSCENARIO: {} with {} entities",
            scenario.file_header.description,
            scenario.entities.len()
        );

        Ok(scenario)
    }

    fn parse_file_header(&self, node: &roxmltree::Node) -> FileHeader {
        FileHeader {
            rev_major: node
                .attribute("revMajor")
                .and_then(|s| s.parse().ok())
                .unwrap_or(1),
            rev_minor: node
                .attribute("revMinor")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0),
            date: node.attribute("date").unwrap_or("").to_string(),
            description: node.attribute("description").unwrap_or("").to_string(),
            author: node.attribute("author").unwrap_or("").to_string(),
        }
    }

    fn parse_parameter_declarations(&self, node: &roxmltree::Node) -> Vec<ParameterDeclaration> {
        node.children()
            .filter(|n| n.tag_name().name() == "ParameterDeclaration")
            .map(|n| ParameterDeclaration {
                name: n.attribute("name").unwrap_or("").to_string(),
                parameter_type: match n.attribute("parameterType") {
                    Some("integer") => ParameterType::Integer,
                    Some("double") => ParameterType::Double,
                    Some("boolean") => ParameterType::Boolean,
                    Some("unsignedInt") => ParameterType::UnsignedInt,
                    Some("unsignedShort") => ParameterType::UnsignedShort,
                    Some("dateTime") => ParameterType::DateTime,
                    _ => ParameterType::String,
                },
                value: n.attribute("value").unwrap_or("").to_string(),
            })
            .collect()
    }

    fn parse_catalog_locations(&self, node: &roxmltree::Node) -> CatalogLocations {
        let mut locations = CatalogLocations::default();

        for child in node.children().filter(|n| n.is_element()) {
            let path = child
                .children()
                .find(|n| n.tag_name().name() == "Directory")
                .and_then(|d| d.attribute("path"))
                .map(PathBuf::from);

            match child.tag_name().name() {
                "VehicleCatalog" => locations.vehicle_catalog = path,
                "PedestrianCatalog" => locations.pedestrian_catalog = path,
                "MiscObjectCatalog" => locations.misc_object_catalog = path,
                "EnvironmentCatalog" => locations.environment_catalog = path,
                "ManeuverCatalog" => locations.maneuver_catalog = path,
                "TrajectoryCatalog" => locations.trajectory_catalog = path,
                "RouteCatalog" => locations.route_catalog = path,
                "ControllerCatalog" => locations.controller_catalog = path,
                _ => {}
            }
        }

        locations
    }

    fn parse_road_network(&self, node: &roxmltree::Node) -> RoadNetwork {
        let mut network = RoadNetwork::default();

        if let Some(logic) = node.children().find(|n| n.tag_name().name() == "LogicFile") {
            network.logic_file = logic.attribute("filepath").map(PathBuf::from);
        }

        if let Some(scene) = node
            .children()
            .find(|n| n.tag_name().name() == "SceneGraphFile")
        {
            network.scene_graph_file = scene.attribute("filepath").map(PathBuf::from);
        }

        network
    }

    fn parse_entities(&self, node: &roxmltree::Node) -> Vec<ScenarioEntity> {
        node.children()
            .filter(|n| n.tag_name().name() == "ScenarioObject")
            .filter_map(|n| self.parse_scenario_object(&n))
            .collect()
    }

    fn parse_scenario_object(&self, node: &roxmltree::Node) -> Option<ScenarioEntity> {
        let name = node.attribute("name")?.to_string();

        let mut entity = ScenarioEntity {
            name,
            entity_type: EntityType::Vehicle,
            object_ref: None,
            vehicle: None,
            pedestrian: None,
            misc_object: None,
            controller: None,
        };

        // Check for catalog reference
        if let Some(cat_ref) = node
            .children()
            .find(|n| n.tag_name().name() == "CatalogReference")
        {
            entity.object_ref = Some(format!(
                "{}:{}",
                cat_ref.attribute("catalogName").unwrap_or(""),
                cat_ref.attribute("entryName").unwrap_or("")
            ));
        }

        // Parse inline definitions
        if let Some(vehicle) = node.children().find(|n| n.tag_name().name() == "Vehicle") {
            entity.entity_type = EntityType::Vehicle;
            entity.vehicle = self.parse_vehicle(&vehicle);
        } else if let Some(ped) = node
            .children()
            .find(|n| n.tag_name().name() == "Pedestrian")
        {
            entity.entity_type = EntityType::Pedestrian;
            entity.pedestrian = self.parse_pedestrian(&ped);
        } else if let Some(misc) = node
            .children()
            .find(|n| n.tag_name().name() == "MiscObject")
        {
            entity.entity_type = EntityType::MiscObject;
            entity.misc_object = self.parse_misc_object(&misc);
        }

        // Parse ObjectController
        if let Some(ctrl) = node
            .children()
            .find(|n| n.tag_name().name() == "ObjectController")
        {
            entity.controller = self.parse_entity_controller(&ctrl);
        }

        Some(entity)
    }

    fn parse_vehicle(&self, node: &roxmltree::Node) -> Option<Vehicle> {
        Some(Vehicle {
            name: node.attribute("name").unwrap_or("").to_string(),
            vehicle_category: match node.attribute("vehicleCategory") {
                Some("car") => VehicleCategory::Car,
                Some("van") => VehicleCategory::Van,
                Some("truck") => VehicleCategory::Truck,
                Some("bus") => VehicleCategory::Bus,
                Some("motorbike") => VehicleCategory::Motorbike,
                Some("bicycle") => VehicleCategory::Bicycle,
                Some("train") => VehicleCategory::Train,
                Some("tram") => VehicleCategory::Tram,
                _ => VehicleCategory::Car,
            },
            bounding_box: self.parse_bounding_box(node),
            performance: self.parse_performance(node),
            axles: self.parse_axles(node),
            properties: self.parse_properties(node),
        })
    }

    fn parse_pedestrian(&self, node: &roxmltree::Node) -> Option<Pedestrian> {
        Some(Pedestrian {
            name: node.attribute("name").unwrap_or("").to_string(),
            model: node.attribute("model").unwrap_or("").to_string(),
            mass: node
                .attribute("mass")
                .and_then(|s| s.parse().ok())
                .unwrap_or(80.0),
            pedestrian_category: match node.attribute("pedestrianCategory") {
                Some("wheelchair") => PedestrianCategory::Wheelchair,
                Some("animal") => PedestrianCategory::Animal,
                _ => PedestrianCategory::Pedestrian,
            },
            bounding_box: self.parse_bounding_box(node),
            properties: self.parse_properties(node),
        })
    }

    fn parse_misc_object(&self, node: &roxmltree::Node) -> Option<MiscObject> {
        Some(MiscObject {
            name: node.attribute("name").unwrap_or("").to_string(),
            misc_object_category: match node.attribute("miscObjectCategory") {
                Some("obstacle") => MiscObjectCategory::Obstacle,
                Some("pole") => MiscObjectCategory::Pole,
                Some("tree") => MiscObjectCategory::Tree,
                Some("barrier") => MiscObjectCategory::Barrier,
                Some("building") => MiscObjectCategory::Building,
                _ => MiscObjectCategory::None,
            },
            mass: node
                .attribute("mass")
                .and_then(|s| s.parse().ok())
                .unwrap_or(1.0),
            bounding_box: self.parse_bounding_box(node),
            properties: self.parse_properties(node),
        })
    }

    fn parse_bounding_box(&self, node: &roxmltree::Node) -> BoundingBox {
        if let Some(bb) = node
            .children()
            .find(|n| n.tag_name().name() == "BoundingBox")
        {
            let center = bb
                .children()
                .find(|n| n.tag_name().name() == "Center")
                .map(|c| Position3D {
                    x: c.attribute("x").and_then(|s| s.parse().ok()).unwrap_or(0.0),
                    y: c.attribute("y").and_then(|s| s.parse().ok()).unwrap_or(0.0),
                    z: c.attribute("z").and_then(|s| s.parse().ok()).unwrap_or(0.0),
                })
                .unwrap_or_default();

            let dimensions = bb
                .children()
                .find(|n| n.tag_name().name() == "Dimensions")
                .map(|d| Dimensions {
                    width: d
                        .attribute("width")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(0.0),
                    length: d
                        .attribute("length")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(0.0),
                    height: d
                        .attribute("height")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(0.0),
                })
                .unwrap_or_default();

            BoundingBox { center, dimensions }
        } else {
            BoundingBox::default()
        }
    }

    fn parse_performance(&self, node: &roxmltree::Node) -> Performance {
        node.children()
            .find(|n| n.tag_name().name() == "Performance")
            .map(|p| Performance {
                max_speed: p
                    .attribute("maxSpeed")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(50.0),
                max_acceleration: p
                    .attribute("maxAcceleration")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(3.0),
                max_deceleration: p
                    .attribute("maxDeceleration")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(6.0),
            })
            .unwrap_or_default()
    }

    fn parse_axles(&self, node: &roxmltree::Node) -> Axles {
        if let Some(axles_node) = node.children().find(|n| n.tag_name().name() == "Axles") {
            let parse_axle = |n: &roxmltree::Node| Axle {
                max_steering: n
                    .attribute("maxSteering")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                wheel_diameter: n
                    .attribute("wheelDiameter")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.6),
                track_width: n
                    .attribute("trackWidth")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(1.5),
                position_x: n
                    .attribute("positionX")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                position_z: n
                    .attribute("positionZ")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.3),
            };

            Axles {
                front_axle: axles_node
                    .children()
                    .find(|n| n.tag_name().name() == "FrontAxle")
                    .map(|n| parse_axle(&n))
                    .unwrap_or_default(),
                rear_axle: axles_node
                    .children()
                    .find(|n| n.tag_name().name() == "RearAxle")
                    .map(|n| parse_axle(&n))
                    .unwrap_or_default(),
                additional_axles: axles_node
                    .children()
                    .filter(|n| n.tag_name().name() == "AdditionalAxle")
                    .map(|n| parse_axle(&n))
                    .collect(),
            }
        } else {
            Axles::default()
        }
    }

    fn parse_properties(&self, node: &roxmltree::Node) -> HashMap<String, String> {
        let mut props = HashMap::new();
        if let Some(properties) = node
            .children()
            .find(|n| n.tag_name().name() == "Properties")
        {
            for prop in properties
                .children()
                .filter(|n| n.tag_name().name() == "Property")
            {
                if let (Some(name), Some(value)) = (prop.attribute("name"), prop.attribute("value"))
                {
                    props.insert(name.to_string(), value.to_string());
                }
            }
        }
        props
    }

    fn parse_entity_controller(&self, node: &roxmltree::Node) -> Option<EntityController> {
        let controller = node
            .children()
            .find(|n| n.tag_name().name() == "Controller")?;

        Some(EntityController {
            name: controller.attribute("name").unwrap_or("").to_string(),
            controller_type: ControllerType::Default,
            properties: self.parse_properties(&controller),
        })
    }

    fn parse_storyboard(&self, node: &roxmltree::Node) -> Storyboard {
        let mut storyboard = Storyboard {
            init: Init::default(),
            stories: Vec::new(),
            stop_trigger: None,
        };

        // Parse Init
        if let Some(init) = node.children().find(|n| n.tag_name().name() == "Init") {
            storyboard.init = self.parse_init(&init);
        }

        // Parse Stories
        for story_node in node.children().filter(|n| n.tag_name().name() == "Story") {
            if let Some(story) = self.parse_story(&story_node) {
                storyboard.stories.push(story);
            }
        }

        // Parse StopTrigger
        if let Some(trigger) = node
            .children()
            .find(|n| n.tag_name().name() == "StopTrigger")
        {
            storyboard.stop_trigger = self.parse_trigger(&trigger);
        }

        storyboard
    }

    fn parse_init(&self, node: &roxmltree::Node) -> Init {
        let mut init = Init::default();

        if let Some(actions) = node.children().find(|n| n.tag_name().name() == "Actions") {
            for action_node in actions.children().filter(|n| n.is_element()) {
                if let Some(action) = self.parse_init_action(&action_node) {
                    init.actions.push(action);
                }
            }
        }

        init
    }

    fn parse_init_action(&self, node: &roxmltree::Node) -> Option<InitAction> {
        match node.tag_name().name() {
            "GlobalAction" => Some(InitAction {
                entity_ref: None,
                global_action: self.parse_global_action(node),
                private_action: None,
            }),
            "Private" => {
                let entity_ref = node.attribute("entityRef").map(String::from);
                let private_action = node
                    .children()
                    .find(|n| n.tag_name().name() == "PrivateAction")
                    .and_then(|pa| self.parse_private_action(&pa));
                Some(InitAction {
                    entity_ref,
                    global_action: None,
                    private_action,
                })
            }
            _ => None,
        }
    }

    fn parse_story(&self, node: &roxmltree::Node) -> Option<Story> {
        let name = node.attribute("name")?.to_string();

        let acts: Vec<Act> = node
            .children()
            .filter(|n| n.tag_name().name() == "Act")
            .filter_map(|a| self.parse_act(&a))
            .collect();

        Some(Story { name, acts })
    }

    fn parse_act(&self, node: &roxmltree::Node) -> Option<Act> {
        let name = node.attribute("name")?.to_string();

        let maneuver_groups: Vec<ManeuverGroup> = node
            .children()
            .filter(|n| n.tag_name().name() == "ManeuverGroup")
            .filter_map(|mg| self.parse_maneuver_group(&mg))
            .collect();

        let start_trigger = node
            .children()
            .find(|n| n.tag_name().name() == "StartTrigger")
            .and_then(|t| self.parse_trigger(&t));

        let stop_trigger = node
            .children()
            .find(|n| n.tag_name().name() == "StopTrigger")
            .and_then(|t| self.parse_trigger(&t));

        Some(Act {
            name,
            maneuver_groups,
            start_trigger,
            stop_trigger,
        })
    }

    fn parse_maneuver_group(&self, node: &roxmltree::Node) -> Option<ManeuverGroup> {
        let name = node.attribute("name")?.to_string();
        let maximum_execution_count = node
            .attribute("maximumExecutionCount")
            .and_then(|s| s.parse().ok())
            .unwrap_or(1);

        let actors: Vec<String> = node
            .children()
            .find(|n| n.tag_name().name() == "Actors")
            .map(|actors| {
                actors
                    .children()
                    .filter(|n| n.tag_name().name() == "EntityRef")
                    .filter_map(|e| e.attribute("entityRef").map(String::from))
                    .collect()
            })
            .unwrap_or_default();

        let maneuvers: Vec<Maneuver> = node
            .children()
            .filter(|n| n.tag_name().name() == "Maneuver")
            .filter_map(|m| self.parse_maneuver(&m))
            .collect();

        Some(ManeuverGroup {
            name,
            maximum_execution_count,
            actors,
            maneuvers,
        })
    }

    fn parse_maneuver(&self, node: &roxmltree::Node) -> Option<Maneuver> {
        let name = node.attribute("name")?.to_string();

        let events: Vec<Event> = node
            .children()
            .filter(|n| n.tag_name().name() == "Event")
            .filter_map(|e| self.parse_event(&e))
            .collect();

        Some(Maneuver { name, events })
    }

    fn parse_event(&self, node: &roxmltree::Node) -> Option<Event> {
        let name = node.attribute("name")?.to_string();
        let priority = match node.attribute("priority") {
            Some("skip") => EventPriority::Skip,
            Some("parallel") => EventPriority::Parallel,
            _ => EventPriority::Overwrite,
        };
        let maximum_execution_count = node
            .attribute("maximumExecutionCount")
            .and_then(|s| s.parse().ok())
            .unwrap_or(1);

        let actions: Vec<EventAction> = node
            .children()
            .filter(|n| n.tag_name().name() == "Action")
            .filter_map(|a| self.parse_event_action(&a))
            .collect();

        let start_trigger = node
            .children()
            .find(|n| n.tag_name().name() == "StartTrigger")
            .and_then(|t| self.parse_trigger(&t));

        Some(Event {
            name,
            priority,
            maximum_execution_count,
            actions,
            start_trigger,
        })
    }

    fn parse_event_action(&self, node: &roxmltree::Node) -> Option<EventAction> {
        let name = node.attribute("name")?.to_string();

        let global_action = node
            .children()
            .find(|n| n.tag_name().name() == "GlobalAction")
            .and_then(|ga| self.parse_global_action(&ga));

        let private_action = node
            .children()
            .find(|n| n.tag_name().name() == "PrivateAction")
            .and_then(|pa| self.parse_private_action(&pa));

        let user_defined_action = node
            .children()
            .find(|n| n.tag_name().name() == "UserDefinedAction")
            .and_then(|uda| self.parse_user_defined_action(&uda));

        Some(EventAction {
            name,
            global_action,
            private_action,
            user_defined_action,
        })
    }

    fn parse_global_action(&self, node: &roxmltree::Node) -> Option<GlobalAction> {
        // Parse environment action
        if let Some(env) = node
            .children()
            .find(|n| n.tag_name().name() == "EnvironmentAction")
        {
            return Some(GlobalAction::Environment(EnvironmentAction {
                environment: self.parse_environment(&env),
            }));
        }

        // Parse entity action
        if let Some(entity_action) = node
            .children()
            .find(|n| n.tag_name().name() == "EntityAction")
        {
            let entity_ref = entity_action
                .attribute("entityRef")
                .unwrap_or("")
                .to_string();
            let action_type = if entity_action
                .children()
                .any(|n| n.tag_name().name() == "DeleteEntityAction")
            {
                EntityActionType::Delete
            } else if let Some(add) = entity_action
                .children()
                .find(|n| n.tag_name().name() == "AddEntityAction")
            {
                let position = add
                    .children()
                    .find(|n| n.tag_name().name() == "Position")
                    .and_then(|p| self.parse_position(&p))
                    .unwrap_or(Position::World(WorldPosition {
                        x: 0.0,
                        y: 0.0,
                        z: None,
                        h: None,
                        p: None,
                        r: None,
                    }));
                EntityActionType::Add(AddEntityAction { position })
            } else {
                return None;
            };

            return Some(GlobalAction::EntityAction(EntityAction {
                entity_ref,
                action_type,
            }));
        }

        None
    }

    fn parse_private_action(&self, node: &roxmltree::Node) -> Option<PrivateAction> {
        // Parse teleport action
        if let Some(teleport) = node
            .children()
            .find(|n| n.tag_name().name() == "TeleportAction")
        {
            let position = teleport
                .children()
                .find(|n| n.tag_name().name() == "Position")
                .and_then(|p| self.parse_position(&p))?;
            return Some(PrivateAction::Teleport(TeleportAction { position }));
        }

        // Parse longitudinal action
        if let Some(long) = node
            .children()
            .find(|n| n.tag_name().name() == "LongitudinalAction")
        {
            return self
                .parse_longitudinal_action(&long)
                .map(PrivateAction::Longitudinal);
        }

        // Parse lateral action
        if let Some(lat) = node
            .children()
            .find(|n| n.tag_name().name() == "LateralAction")
        {
            return self.parse_lateral_action(&lat).map(PrivateAction::Lateral);
        }

        // Parse routing action
        if let Some(routing) = node
            .children()
            .find(|n| n.tag_name().name() == "RoutingAction")
        {
            return self
                .parse_routing_action(&routing)
                .map(PrivateAction::Routing);
        }

        None
    }

    fn parse_longitudinal_action(&self, node: &roxmltree::Node) -> Option<LongitudinalAction> {
        let speed_action = node
            .children()
            .find(|n| n.tag_name().name() == "SpeedAction")
            .and_then(|sa| self.parse_speed_action(&sa));

        Some(LongitudinalAction {
            speed_action,
            longitudinal_distance_action: None,
        })
    }

    fn parse_speed_action(&self, node: &roxmltree::Node) -> Option<SpeedAction> {
        let dynamics = node
            .children()
            .find(|n| n.tag_name().name() == "SpeedActionDynamics")
            .map(|d| self.parse_transition_dynamics(&d))
            .unwrap_or_else(|| TransitionDynamics {
                dynamics_shape: DynamicsShape::Step,
                value: 0.0,
                dynamics_dimension: DynamicsDimension::Time,
            });

        let target = node
            .children()
            .find(|n| n.tag_name().name() == "SpeedActionTarget")?;

        let speed_action_target = if let Some(abs) = target
            .children()
            .find(|n| n.tag_name().name() == "AbsoluteTargetSpeed")
        {
            SpeedActionTarget::Absolute(AbsoluteTargetSpeed {
                value: abs
                    .attribute("value")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
            })
        } else if let Some(rel) = target
            .children()
            .find(|n| n.tag_name().name() == "RelativeTargetSpeed")
        {
            SpeedActionTarget::Relative(RelativeTargetSpeed {
                entity_ref: rel.attribute("entityRef").unwrap_or("").to_string(),
                value: rel
                    .attribute("value")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                speed_target_value_type: match rel.attribute("speedTargetValueType") {
                    Some("factor") => SpeedTargetValueType::Factor,
                    _ => SpeedTargetValueType::Delta,
                },
                continuous: rel
                    .attribute("continuous")
                    .map(|s| s == "true")
                    .unwrap_or(false),
            })
        } else {
            return None;
        };

        Some(SpeedAction {
            speed_action_dynamics: dynamics,
            speed_action_target,
        })
    }

    fn parse_transition_dynamics(&self, node: &roxmltree::Node) -> TransitionDynamics {
        TransitionDynamics {
            dynamics_shape: match node.attribute("dynamicsShape") {
                Some("cubic") => DynamicsShape::Cubic,
                Some("sinusoidal") => DynamicsShape::Sinusoidal,
                Some("step") => DynamicsShape::Step,
                _ => DynamicsShape::Linear,
            },
            value: node
                .attribute("value")
                .and_then(|s| s.parse().ok())
                .unwrap_or(0.0),
            dynamics_dimension: match node.attribute("dynamicsDimension") {
                Some("time") => DynamicsDimension::Time,
                Some("distance") => DynamicsDimension::Distance,
                _ => DynamicsDimension::Rate,
            },
        }
    }

    fn parse_lateral_action(&self, node: &roxmltree::Node) -> Option<LateralAction> {
        let lane_change_action = node
            .children()
            .find(|n| n.tag_name().name() == "LaneChangeAction")
            .and_then(|lc| self.parse_lane_change_action(&lc));

        Some(LateralAction {
            lane_change_action,
            lane_offset_action: None,
            lateral_distance_action: None,
        })
    }

    fn parse_lane_change_action(&self, node: &roxmltree::Node) -> Option<LaneChangeAction> {
        let dynamics = node
            .children()
            .find(|n| n.tag_name().name() == "LaneChangeActionDynamics")
            .map(|d| self.parse_transition_dynamics(&d))
            .unwrap_or_else(|| TransitionDynamics {
                dynamics_shape: DynamicsShape::Sinusoidal,
                value: 3.0,
                dynamics_dimension: DynamicsDimension::Time,
            });

        let target = node
            .children()
            .find(|n| n.tag_name().name() == "LaneChangeTarget")?;

        let lane_change_target = if let Some(abs) = target
            .children()
            .find(|n| n.tag_name().name() == "AbsoluteTargetLane")
        {
            LaneChangeTarget::Absolute(AbsoluteTargetLane {
                value: abs.attribute("value").unwrap_or("0").to_string(),
            })
        } else if let Some(rel) = target
            .children()
            .find(|n| n.tag_name().name() == "RelativeTargetLane")
        {
            LaneChangeTarget::Relative(RelativeTargetLane {
                entity_ref: rel.attribute("entityRef").unwrap_or("").to_string(),
                value: rel
                    .attribute("value")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0),
            })
        } else {
            return None;
        };

        Some(LaneChangeAction {
            lane_change_action_dynamics: dynamics,
            lane_change_target,
        })
    }

    fn parse_routing_action(&self, node: &roxmltree::Node) -> Option<RoutingAction> {
        let acquire_position_action = node
            .children()
            .find(|n| n.tag_name().name() == "AcquirePositionAction")
            .and_then(|apa| {
                let position = apa
                    .children()
                    .find(|n| n.tag_name().name() == "Position")
                    .and_then(|p| self.parse_position(&p))?;
                Some(AcquirePositionAction { position })
            });

        Some(RoutingAction {
            assign_route_action: None,
            follow_trajectory_action: None,
            acquire_position_action,
        })
    }

    fn parse_user_defined_action(&self, node: &roxmltree::Node) -> Option<UserDefinedAction> {
        let command = node
            .children()
            .find(|n| n.tag_name().name() == "CustomCommandAction")?;

        Some(UserDefinedAction {
            custom_command_action: CustomCommandAction {
                command_type: command.attribute("type").unwrap_or("").to_string(),
                content: command.text().unwrap_or("").to_string(),
            },
        })
    }

    fn parse_environment(&self, node: &roxmltree::Node) -> Environment {
        let env_node = node
            .children()
            .find(|n| n.tag_name().name() == "Environment")
            .unwrap_or(*node);

        Environment {
            name: env_node.attribute("name").unwrap_or("").to_string(),
            time_of_day: env_node
                .children()
                .find(|n| n.tag_name().name() == "TimeOfDay")
                .map(|t| TimeOfDay {
                    animation: t
                        .attribute("animation")
                        .map(|s| s == "true")
                        .unwrap_or(false),
                    date_time: t.attribute("dateTime").unwrap_or("").to_string(),
                }),
            weather: env_node
                .children()
                .find(|n| n.tag_name().name() == "Weather")
                .map(|w| self.parse_weather(&w)),
            road_condition: env_node
                .children()
                .find(|n| n.tag_name().name() == "RoadCondition")
                .map(|r| RoadCondition {
                    friction_scale_factor: r
                        .attribute("frictionScaleFactor")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(1.0),
                }),
        }
    }

    fn parse_weather(&self, node: &roxmltree::Node) -> Weather {
        Weather {
            cloud_state: match node.attribute("cloudState") {
                Some("cloudy") => CloudState::Cloudy,
                Some("overcast") => CloudState::Overcast,
                Some("rainy") => CloudState::Rainy,
                Some("skyOff") => CloudState::SkyOff,
                _ => CloudState::Free,
            },
            sun: node
                .children()
                .find(|n| n.tag_name().name() == "Sun")
                .map(|s| Sun {
                    intensity: s
                        .attribute("intensity")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1.0),
                    azimuth: s
                        .attribute("azimuth")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0),
                    elevation: s
                        .attribute("elevation")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1.0),
                }),
            fog: node
                .children()
                .find(|n| n.tag_name().name() == "Fog")
                .map(|f| Fog {
                    visual_range: f
                        .attribute("visualRange")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(10000.0),
                }),
            precipitation: node
                .children()
                .find(|n| n.tag_name().name() == "Precipitation")
                .map(|p| Precipitation {
                    precipitation_type: match p.attribute("precipitationType") {
                        Some("rain") => PrecipitationType::Rain,
                        Some("snow") => PrecipitationType::Snow,
                        _ => PrecipitationType::Dry,
                    },
                    intensity: p
                        .attribute("intensity")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0),
                }),
            wind: node
                .children()
                .find(|n| n.tag_name().name() == "Wind")
                .map(|w| Wind {
                    direction: w
                        .attribute("direction")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0),
                    speed: w
                        .attribute("speed")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0),
                }),
        }
    }

    fn parse_trigger(&self, node: &roxmltree::Node) -> Option<Trigger> {
        let condition_groups: Vec<ConditionGroup> = node
            .children()
            .filter(|n| n.tag_name().name() == "ConditionGroup")
            .filter_map(|cg| self.parse_condition_group(&cg))
            .collect();

        if condition_groups.is_empty() {
            None
        } else {
            Some(Trigger { condition_groups })
        }
    }

    fn parse_condition_group(&self, node: &roxmltree::Node) -> Option<ConditionGroup> {
        let conditions: Vec<Condition> = node
            .children()
            .filter(|n| n.tag_name().name() == "Condition")
            .filter_map(|c| self.parse_condition(&c))
            .collect();

        if conditions.is_empty() {
            None
        } else {
            Some(ConditionGroup { conditions })
        }
    }

    fn parse_condition(&self, node: &roxmltree::Node) -> Option<Condition> {
        let name = node.attribute("name")?.to_string();
        let delay = node
            .attribute("delay")
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.0);
        let condition_edge = match node.attribute("conditionEdge") {
            Some("falling") => ConditionEdge::Falling,
            Some("risingOrFalling") => ConditionEdge::RisingOrFalling,
            Some("none") => ConditionEdge::None,
            _ => ConditionEdge::Rising,
        };

        let by_entity_condition = node
            .children()
            .find(|n| n.tag_name().name() == "ByEntityCondition")
            .and_then(|bec| self.parse_by_entity_condition(&bec));

        let by_value_condition = node
            .children()
            .find(|n| n.tag_name().name() == "ByValueCondition")
            .and_then(|bvc| self.parse_by_value_condition(&bvc));

        Some(Condition {
            name,
            delay,
            condition_edge,
            by_entity_condition,
            by_value_condition,
        })
    }

    fn parse_by_entity_condition(&self, node: &roxmltree::Node) -> Option<ByEntityCondition> {
        let triggering_entities = node
            .children()
            .find(|n| n.tag_name().name() == "TriggeringEntities")
            .map(|te| TriggeringEntities {
                rule: match te.attribute("triggeringEntitiesRule") {
                    Some("all") => TriggeringEntitiesRule::All,
                    _ => TriggeringEntitiesRule::Any,
                },
                entity_refs: te
                    .children()
                    .filter(|n| n.tag_name().name() == "EntityRef")
                    .filter_map(|e| e.attribute("entityRef").map(String::from))
                    .collect(),
            })?;

        let entity_condition = node
            .children()
            .find(|n| n.tag_name().name() == "EntityCondition")
            .and_then(|ec| self.parse_entity_condition(&ec))?;

        Some(ByEntityCondition {
            triggering_entities,
            entity_condition,
        })
    }

    fn parse_entity_condition(&self, node: &roxmltree::Node) -> Option<EntityCondition> {
        // Speed condition
        if let Some(speed) = node
            .children()
            .find(|n| n.tag_name().name() == "SpeedCondition")
        {
            return Some(EntityCondition::Speed(SpeedCondition {
                value: speed
                    .attribute("value")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                rule: self.parse_rule(speed.attribute("rule")),
            }));
        }

        // Reach position condition
        if let Some(reach) = node
            .children()
            .find(|n| n.tag_name().name() == "ReachPositionCondition")
        {
            let position = reach
                .children()
                .find(|n| n.tag_name().name() == "Position")
                .and_then(|p| self.parse_position(&p))?;
            return Some(EntityCondition::ReachPosition(ReachPositionCondition {
                tolerance: reach
                    .attribute("tolerance")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(1.0),
                position,
            }));
        }

        // Distance condition
        if let Some(dist) = node
            .children()
            .find(|n| n.tag_name().name() == "DistanceCondition")
        {
            let position = dist
                .children()
                .find(|n| n.tag_name().name() == "Position")
                .and_then(|p| self.parse_position(&p))?;
            return Some(EntityCondition::Distance(DistanceCondition {
                value: dist
                    .attribute("value")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                freespace: dist
                    .attribute("freespace")
                    .map(|s| s == "true")
                    .unwrap_or(false),
                along_route: dist
                    .attribute("alongRoute")
                    .map(|s| s == "true")
                    .unwrap_or(false),
                rule: self.parse_rule(dist.attribute("rule")),
                position,
            }));
        }

        // Stand still condition
        if let Some(stand) = node
            .children()
            .find(|n| n.tag_name().name() == "StandStillCondition")
        {
            return Some(EntityCondition::StandStill(StandStillCondition {
                duration: stand
                    .attribute("duration")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
            }));
        }

        None
    }

    fn parse_by_value_condition(&self, node: &roxmltree::Node) -> Option<ByValueCondition> {
        // Simulation time condition
        if let Some(sim_time) = node
            .children()
            .find(|n| n.tag_name().name() == "SimulationTimeCondition")
        {
            return Some(ByValueCondition::SimulationTimeCondition(
                SimulationTimeCondition {
                    value: sim_time
                        .attribute("value")
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(0.0),
                    rule: self.parse_rule(sim_time.attribute("rule")),
                },
            ));
        }

        // Storyboard element state condition
        if let Some(sb_state) = node
            .children()
            .find(|n| n.tag_name().name() == "StoryboardElementStateCondition")
        {
            return Some(ByValueCondition::StoryboardElementStateCondition(
                StoryboardElementStateCondition {
                    storyboard_element_ref: sb_state
                        .attribute("storyboardElementRef")
                        .unwrap_or("")
                        .to_string(),
                    storyboard_element_type: match sb_state.attribute("storyboardElementType") {
                        Some("story") => StoryboardElementType::Story,
                        Some("act") => StoryboardElementType::Act,
                        Some("maneuverGroup") => StoryboardElementType::ManeuverGroup,
                        Some("maneuver") => StoryboardElementType::Maneuver,
                        Some("action") => StoryboardElementType::Action,
                        _ => StoryboardElementType::Event,
                    },
                    state: match sb_state.attribute("state") {
                        Some("endTransition") => StoryboardElementState::EndTransition,
                        Some("stopTransition") => StoryboardElementState::StopTransition,
                        Some("skipTransition") => StoryboardElementState::SkipTransition,
                        Some("completeState") => StoryboardElementState::CompleteState,
                        Some("runningState") => StoryboardElementState::RunningState,
                        Some("standbyState") => StoryboardElementState::StandbyState,
                        _ => StoryboardElementState::StartTransition,
                    },
                },
            ));
        }

        None
    }

    fn parse_rule(&self, rule: Option<&str>) -> Rule {
        match rule {
            Some("lessThan") => Rule::LessThan,
            Some("equalTo") => Rule::EqualTo,
            Some("greaterOrEqual") => Rule::GreaterOrEqual,
            Some("lessOrEqual") => Rule::LessOrEqual,
            Some("notEqualTo") => Rule::NotEqualTo,
            _ => Rule::GreaterThan,
        }
    }

    fn parse_position(&self, node: &roxmltree::Node) -> Option<Position> {
        // World position
        if let Some(world) = node
            .children()
            .find(|n| n.tag_name().name() == "WorldPosition")
        {
            return Some(Position::World(WorldPosition {
                x: world
                    .attribute("x")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                y: world
                    .attribute("y")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                z: world.attribute("z").and_then(|s| s.parse().ok()),
                h: world.attribute("h").and_then(|s| s.parse().ok()),
                p: world.attribute("p").and_then(|s| s.parse().ok()),
                r: world.attribute("r").and_then(|s| s.parse().ok()),
            }));
        }

        // Lane position
        if let Some(lane) = node
            .children()
            .find(|n| n.tag_name().name() == "LanePosition")
        {
            return Some(Position::Lane(LanePosition {
                road_id: lane.attribute("roadId").unwrap_or("").to_string(),
                lane_id: lane.attribute("laneId").unwrap_or("").to_string(),
                s: lane
                    .attribute("s")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                offset: lane.attribute("offset").and_then(|s| s.parse().ok()),
                orientation: None,
            }));
        }

        // Road position
        if let Some(road) = node
            .children()
            .find(|n| n.tag_name().name() == "RoadPosition")
        {
            return Some(Position::Road(RoadPosition {
                road_id: road.attribute("roadId").unwrap_or("").to_string(),
                s: road
                    .attribute("s")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                t: road
                    .attribute("t")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                orientation: None,
            }));
        }

        // Relative world position
        if let Some(rel_world) = node
            .children()
            .find(|n| n.tag_name().name() == "RelativeWorldPosition")
        {
            return Some(Position::RelativeWorld(RelativeWorldPosition {
                entity_ref: rel_world.attribute("entityRef").unwrap_or("").to_string(),
                dx: rel_world
                    .attribute("dx")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                dy: rel_world
                    .attribute("dy")
                    .and_then(|s| s.parse().ok())
                    .unwrap_or(0.0),
                dz: rel_world.attribute("dz").and_then(|s| s.parse().ok()),
                orientation: None,
            }));
        }

        None
    }

    fn resolve_path(&self, path: &Path) -> PathBuf {
        if path.is_absolute() {
            path.to_path_buf()
        } else {
            self.base_path.join(path)
        }
    }
}

impl Default for OpenSCENARIOLoader {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Bevy Components and Plugin
// ============================================================================

/// Component for scenario entity
#[derive(Component)]
pub struct ScenarioEntityComponent {
    pub name: String,
    pub entity_type: EntityType,
}

/// Resource for active scenario
#[derive(Resource)]
pub struct ActiveScenario {
    pub scenario: OpenSCENARIOScenario,
    pub simulation_time: f64,
    pub state: ScenarioState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ScenarioState {
    #[default]
    Standby,
    Running,
    Paused,
    Completed,
}

/// Plugin for OpenSCENARIO support
pub struct OpenSCENARIOPlugin;

impl Plugin for OpenSCENARIOPlugin {
    fn build(&self, app: &mut App) {
        app.add_event::<LoadScenarioEvent>()
            .add_systems(Update, handle_load_scenario);
    }
}

/// Event to load scenario
#[derive(Event)]
pub struct LoadScenarioEvent {
    pub path: PathBuf,
}

fn handle_load_scenario(mut events: EventReader<LoadScenarioEvent>, mut commands: Commands) {
    for event in events.read() {
        let loader =
            OpenSCENARIOLoader::new().with_base_path(event.path.parent().unwrap_or(Path::new(".")));

        match loader.load(&event.path) {
            Ok(scenario) => {
                commands.insert_resource(ActiveScenario {
                    scenario,
                    simulation_time: 0.0,
                    state: ScenarioState::Standby,
                });
                info!("OpenSCENARIO loaded successfully");
            }
            Err(e) => {
                error!("Failed to load OpenSCENARIO: {}", e);
            }
        }
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_simple_scenario() {
        let xml = r#"<?xml version="1.0"?>
<OpenSCENARIO>
    <FileHeader revMajor="1" revMinor="0" date="2024-01-01" description="Test" author="HORUS"/>
    <ParameterDeclarations/>
    <CatalogLocations/>
    <RoadNetwork>
        <LogicFile filepath="town01.xodr"/>
    </RoadNetwork>
    <Entities>
        <ScenarioObject name="Ego">
            <Vehicle name="vehicle.tesla.model3" vehicleCategory="car">
                <BoundingBox>
                    <Center x="1.5" y="0.0" z="0.9"/>
                    <Dimensions width="2.1" length="4.5" height="1.8"/>
                </BoundingBox>
                <Performance maxSpeed="69.4" maxAcceleration="3.0" maxDeceleration="6.0"/>
                <Axles>
                    <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.8" positionX="3.1" positionZ="0.3"/>
                    <RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.8" positionX="0" positionZ="0.3"/>
                </Axles>
            </Vehicle>
        </ScenarioObject>
    </Entities>
    <Storyboard>
        <Init>
            <Actions>
                <Private entityRef="Ego">
                    <PrivateAction>
                        <TeleportAction>
                            <Position>
                                <WorldPosition x="10.0" y="20.0" z="0.0" h="0.0"/>
                            </Position>
                        </TeleportAction>
                    </PrivateAction>
                </Private>
            </Actions>
        </Init>
        <Story name="TestStory">
            <Act name="Act1">
                <ManeuverGroup name="MG1" maximumExecutionCount="1">
                    <Actors>
                        <EntityRef entityRef="Ego"/>
                    </Actors>
                    <Maneuver name="Accelerate">
                        <Event name="SpeedEvent" priority="overwrite">
                            <Action name="SpeedAction">
                                <PrivateAction>
                                    <LongitudinalAction>
                                        <SpeedAction>
                                            <SpeedActionDynamics dynamicsShape="linear" value="3.0" dynamicsDimension="time"/>
                                            <SpeedActionTarget>
                                                <AbsoluteTargetSpeed value="15.0"/>
                                            </SpeedActionTarget>
                                        </SpeedAction>
                                    </LongitudinalAction>
                                </PrivateAction>
                            </Action>
                            <StartTrigger>
                                <ConditionGroup>
                                    <Condition name="StartCondition" delay="0" conditionEdge="rising">
                                        <ByValueCondition>
                                            <SimulationTimeCondition value="0" rule="greaterThan"/>
                                        </ByValueCondition>
                                    </Condition>
                                </ConditionGroup>
                            </StartTrigger>
                        </Event>
                    </Maneuver>
                </ManeuverGroup>
                <StartTrigger>
                    <ConditionGroup>
                        <Condition name="ActStart" delay="0" conditionEdge="rising">
                            <ByValueCondition>
                                <SimulationTimeCondition value="0" rule="greaterThan"/>
                            </ByValueCondition>
                        </Condition>
                    </ConditionGroup>
                </StartTrigger>
            </Act>
        </Story>
    </Storyboard>
</OpenSCENARIO>"#;

        let loader = OpenSCENARIOLoader::new();
        let scenario = loader.parse(xml, Path::new("test.xosc")).unwrap();

        assert_eq!(scenario.file_header.description, "Test");
        assert_eq!(scenario.entities.len(), 1);
        assert_eq!(scenario.entities[0].name, "Ego");
        assert_eq!(scenario.storyboard.stories.len(), 1);
    }
}
