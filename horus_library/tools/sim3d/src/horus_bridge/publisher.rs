use bevy::prelude::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use super::messages::*;
use crate::hframe::HFrameTree;
use crate::sensors::lidar3d::{LaserScan, Lidar2D, Lidar3D, PointCloud};

#[derive(Resource)]
pub struct HorusPublisher {
    tf_publisher: Arc<Mutex<HashMap<String, TransformStamped>>>,
    pointcloud_publisher: Arc<Mutex<HashMap<String, PointCloud2>>>,
    laserscan_publisher: Arc<Mutex<HashMap<String, LaserScanMessage>>>,
    odom_publisher: Arc<Mutex<HashMap<String, Odometry>>>,
    joint_state_publisher: Arc<Mutex<JointState>>,
    enabled: bool,
}

impl Default for HorusPublisher {
    fn default() -> Self {
        Self::new()
    }
}

impl HorusPublisher {
    pub fn new() -> Self {
        Self {
            tf_publisher: Arc::new(Mutex::new(HashMap::new())),
            pointcloud_publisher: Arc::new(Mutex::new(HashMap::new())),
            laserscan_publisher: Arc::new(Mutex::new(HashMap::new())),
            odom_publisher: Arc::new(Mutex::new(HashMap::new())),
            joint_state_publisher: Arc::new(Mutex::new(JointState {
                header: Header::new("", 0.0),
                name: Vec::new(),
                position: Vec::new(),
                velocity: Vec::new(),
                effort: Vec::new(),
            })),
            enabled: true,
        }
    }

    pub fn enable(&mut self) {
        self.enabled = true;
    }

    pub fn disable(&mut self) {
        self.enabled = false;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn publish_transform(
        &self,
        frame_id: impl Into<String>,
        child_frame: impl Into<String>,
        transform: &Transform,
        time: f64,
    ) {
        if !self.enabled {
            return;
        }

        let msg = TransformStamped {
            header: Header::new(frame_id, time),
            child_frame_id: child_frame.into(),
            transform: TransformMessage::from_bevy_transform(transform),
        };

        if let Ok(mut publisher) = self.tf_publisher.lock() {
            publisher.insert(msg.child_frame_id.clone(), msg);
        }
    }

    pub fn publish_pointcloud(
        &self,
        topic: impl Into<String>,
        pointcloud: &PointCloud,
        frame_id: impl Into<String>,
        time: f64,
    ) {
        if !self.enabled {
            return;
        }

        let msg = convert_pointcloud_to_msg(pointcloud, frame_id, time);

        if let Ok(mut publisher) = self.pointcloud_publisher.lock() {
            publisher.insert(topic.into(), msg);
        }
    }

    pub fn publish_laserscan(
        &self,
        topic: impl Into<String>,
        scan: &LaserScan,
        frame_id: impl Into<String>,
        time: f64,
    ) {
        if !self.enabled {
            return;
        }

        let msg = LaserScanMessage {
            header: Header::new(frame_id, time),
            angle_min: scan.angle_min,
            angle_max: scan.angle_max,
            angle_increment: scan.angle_increment,
            time_increment: 0.0,
            scan_time: 1.0 / 10.0,
            range_min: scan.range_min,
            range_max: scan.range_max,
            ranges: scan.ranges.clone(),
            intensities: scan.intensities.clone(),
        };

        if let Ok(mut publisher) = self.laserscan_publisher.lock() {
            publisher.insert(topic.into(), msg);
        }
    }

    pub fn publish_odometry(
        &self,
        topic: impl Into<String>,
        transform: &Transform,
        linear_vel: Vec3,
        angular_vel: Vec3,
        frame_id: impl Into<String>,
        child_frame_id: impl Into<String>,
        time: f64,
    ) {
        if !self.enabled {
            return;
        }

        let msg = Odometry {
            header: Header::new(frame_id, time),
            child_frame_id: child_frame_id.into(),
            pose: PoseWithCovariance {
                pose: Pose {
                    position: transform.translation.into(),
                    orientation: transform.rotation.into(),
                },
                covariance: vec![0.0; 36],
            },
            twist: TwistWithCovariance {
                twist: Twist {
                    linear: linear_vel.into(),
                    angular: angular_vel.into(),
                },
                covariance: vec![0.0; 36],
            },
        };

        if let Ok(mut publisher) = self.odom_publisher.lock() {
            publisher.insert(topic.into(), msg);
        }
    }

    pub fn publish_joint_state(
        &self,
        names: Vec<String>,
        positions: Vec<f32>,
        velocities: Vec<f32>,
        efforts: Vec<f32>,
        time: f64,
    ) {
        if !self.enabled {
            return;
        }

        let msg = JointState {
            header: Header::new("", time),
            name: names,
            position: positions,
            velocity: velocities,
            effort: efforts,
        };

        if let Ok(mut publisher) = self.joint_state_publisher.lock() {
            *publisher = msg;
        }
    }

    pub fn get_tf_messages(&self) -> Vec<TransformStamped> {
        if let Ok(publisher) = self.tf_publisher.lock() {
            publisher.values().cloned().collect()
        } else {
            Vec::new()
        }
    }

    pub fn get_pointcloud_messages(&self) -> HashMap<String, PointCloud2> {
        if let Ok(publisher) = self.pointcloud_publisher.lock() {
            publisher.clone()
        } else {
            HashMap::new()
        }
    }

    pub fn get_laserscan_messages(&self) -> HashMap<String, LaserScanMessage> {
        if let Ok(publisher) = self.laserscan_publisher.lock() {
            publisher.clone()
        } else {
            HashMap::new()
        }
    }

    pub fn clear(&self) {
        if let Ok(mut publisher) = self.tf_publisher.lock() {
            publisher.clear();
        }
        if let Ok(mut publisher) = self.pointcloud_publisher.lock() {
            publisher.clear();
        }
        if let Ok(mut publisher) = self.laserscan_publisher.lock() {
            publisher.clear();
        }
        if let Ok(mut publisher) = self.odom_publisher.lock() {
            publisher.clear();
        }
    }
}

fn convert_pointcloud_to_msg(
    pointcloud: &PointCloud,
    frame_id: impl Into<String>,
    time: f64,
) -> PointCloud2 {
    let num_points = pointcloud.points.len();

    let fields = vec![
        PointField::new("x", 0, PointField::FLOAT32, 1),
        PointField::new("y", 4, PointField::FLOAT32, 1),
        PointField::new("z", 8, PointField::FLOAT32, 1),
        PointField::new("intensity", 12, PointField::FLOAT32, 1),
    ];

    let point_step = 16;
    let mut data = Vec::with_capacity(num_points * point_step as usize);

    for (point, &intensity) in pointcloud.points.iter().zip(pointcloud.intensities.iter()) {
        data.extend_from_slice(&point.x.to_le_bytes());
        data.extend_from_slice(&point.y.to_le_bytes());
        data.extend_from_slice(&point.z.to_le_bytes());
        data.extend_from_slice(&intensity.to_le_bytes());
    }

    PointCloud2 {
        header: Header::new(frame_id, time),
        height: 1,
        width: num_points as u32,
        fields,
        is_bigendian: false,
        point_step,
        row_step: point_step * num_points as u32,
        data,
        is_dense: true,
    }
}

pub fn publish_hframe_system(
    time: Res<Time>,
    _hframe_tree: Res<HFrameTree>,
    publisher: Res<HorusPublisher>,
    query: Query<(&GlobalTransform, &Name)>,
) {
    if !publisher.is_enabled() {
        return;
    }

    let current_time = time.elapsed_secs_f64();

    for (transform, name) in query.iter() {
        let bevy_transform = transform.compute_transform();
        publisher.publish_transform("world", name.as_str(), &bevy_transform, current_time);
    }
}

pub fn publish_lidar3d_system(
    time: Res<Time>,
    publisher: Res<HorusPublisher>,
    query: Query<(&Lidar3D, &PointCloud, &Name)>,
) {
    if !publisher.is_enabled() {
        return;
    }

    let current_time = time.elapsed_secs_f64();

    for (_, pointcloud, name) in query.iter() {
        let topic = format!("{}.pointcloud", name.as_str());
        publisher.publish_pointcloud(&topic, pointcloud, name.as_str(), current_time);
    }
}

pub fn publish_lidar2d_system(
    time: Res<Time>,
    publisher: Res<HorusPublisher>,
    query: Query<(&Lidar2D, &LaserScan, &Name)>,
) {
    if !publisher.is_enabled() {
        return;
    }

    let current_time = time.elapsed_secs_f64();

    for (_, scan, name) in query.iter() {
        let topic = format!("{}.scan", name.as_str());
        publisher.publish_laserscan(&topic, scan, name.as_str(), current_time);
    }
}
