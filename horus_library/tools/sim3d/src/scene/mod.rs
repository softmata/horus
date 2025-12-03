pub mod composition;
pub mod gazebo_models;
pub mod heightmap_loader;
pub mod loader;
pub mod opendrive_loader;
pub mod openscenario_loader;
pub mod physics_materials;
pub mod sdf_importer;
pub mod spawner;
pub mod usd_importer;
pub mod validation;
pub mod world_loader;

// Re-export SDF types for external use
#[allow(unused_imports)]
pub use sdf_importer::{
    SDFCollision, SDFGeometry, SDFImporter, SDFInertial, SDFJoint, SDFLight, SDFLink, SDFMaterial,
    SDFModel, SDFPose, SDFVisual, SDFWorld,
};

// Re-export USD types for external use
#[allow(unused_imports)]
pub use usd_importer::{
    USDGeometry, USDImporter, USDPhysics, USDPrim, USDPrimType, USDSpawnData, USDStage,
    USDTransform, UpAxis,
};

// Re-export heightmap/terrain types
#[allow(unused_imports)]
pub use heightmap_loader::{
    CollisionHeightfield, HeightmapData, HeightmapLoader, LoadTerrainEvent, Terrain, TerrainConfig,
    TerrainMeshGenerator, TerrainPhysics, TerrainPlugin, TerrainSize,
};

// Re-export OpenDRIVE types
#[allow(unused_imports)]
pub use opendrive_loader::{
    Geometry, GeometryType, Junction, Lane, LaneType, LoadOpenDRIVEEvent, OpenDRIVELoader,
    OpenDRIVENetwork, OpenDRIVEPlugin, OpenDRIVERoad, OpenDRIVERoadNetwork, Road, RoadMesh,
    RoadMeshGenerator, RoadPoint,
};

// Re-export OpenSCENARIO types
#[allow(unused_imports)]
pub use openscenario_loader::{
    Act, ActiveScenario, Condition, EntityType, Event, LoadScenarioEvent, Maneuver,
    OpenSCENARIOLoader, OpenSCENARIOPlugin, OpenSCENARIOScenario, Pedestrian, Position,
    ScenarioEntity, ScenarioState, Story, Storyboard, Trigger, Vehicle,
};

// Re-export Gazebo model types
#[allow(unused_imports)]
pub use gazebo_models::{
    FuelClient, FuelUri, GazeboModelDatabase, GazeboModelLoader, GazeboModelsPlugin,
    LoadGazeboModelEvent, LoadedGazeboModel, ModelConfig, ModelDatabase, ScanModelsEvent,
};

// Re-export physics materials
#[allow(unused_imports)]
pub use physics_materials::{
    ContactProperties, FrictionCombineMode, MaterialAssignment, MaterialCategory,
    MaterialContactPair, PhysicsMaterial, PhysicsMaterialDatabase, PhysicsMaterialDatabaseResource,
    PhysicsMaterialsPlugin, RestitutionCombineMode,
};

// Re-export scene loader types
#[allow(unused_imports)]
pub use loader::{
    LoadedScene, SceneBuilder, SceneDefinition, SceneLighting, SceneLoader, SceneObject,
    SceneRobot, SceneShape,
};

// Re-export scene spawner types
#[allow(unused_imports)]
pub use spawner::{ObjectSpawnConfig, ObjectSpawner, SpawnShape, SpawnedObjects};

// Re-export scene composition types
#[allow(unused_imports)]
pub use composition::{ComposableScene, SceneComposer, SceneInclude};

// Re-export unified world loader types
#[allow(unused_imports)]
pub use world_loader::{
    CurrentWorld, LoadWorldEvent, LoadedWorld, UpAxis as WorldUpAxis, WorldLoadConfig,
    WorldLoadError, WorldLoadErrorEvent, WorldLoadResult, WorldLoadedEvent, WorldLoader,
    WorldLoaderPlugin, WorldMetadata,
};
