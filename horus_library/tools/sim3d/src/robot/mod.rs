pub mod articulated;
pub mod gazebo;
pub mod mjcf_loader;
#[allow(clippy::module_inception)] // Robot module provides Robot type - naming is intentional
pub mod robot;
pub mod srdf_loader;
pub mod state;
pub mod urdf_loader;
pub mod xacro_loader;

pub use robot::Robot;

// Re-export URDF loader
#[allow(unused_imports)]
pub use urdf_loader::URDFLoader;

// Re-export Xacro preprocessor
#[allow(unused_imports)]
pub use xacro_loader::XacroPreprocessor;

// Re-export MJCF (MuJoCo) loader types
#[allow(unused_imports)]
pub use mjcf_loader::{
    MJCFActuator, MJCFActuatorType, MJCFAssets, MJCFBody, MJCFCompiler, MJCFGeom, MJCFGeomType,
    MJCFJoint, MJCFJointType, MJCFLoader, MJCFModel, MJCFOption, MJCFSensor,
};

// Re-export SRDF (MoveIt semantic) loader types
#[allow(unused_imports)]
pub use srdf_loader::{
    DisabledCollision, DisabledCollisionReason, EndEffector, GroupState, KinematicChain,
    PlanningGroup, SRDFLoader, VirtualJoint, VirtualJointType, SRDF,
};

// Re-export articulated robot types

// Re-export joint state types

// Re-export Gazebo extension types
