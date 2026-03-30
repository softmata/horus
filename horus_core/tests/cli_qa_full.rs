//! Quick 20s test for frame can retry fix
use horus_core::communication::Topic;
use horus_core::core::{DurationExt, Node};
use horus_core::scheduling::Scheduler;
use horus_robotics::CmdVel;
use horus_tf::*;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
mod common;
use common::cleanup_stale_shm;
struct B{name:String,t:Option<Topic<CmdVel>>,tf:Arc<TransformFrame>,c:Arc<AtomicU64>}
impl Node for B {
    fn name(&self)->&str{&self.name}
    fn init(&mut self)->horus_core::error::HorusResult<()>{self.t=Some(Topic::new("cmd_vel")?);Ok(())}
    fn tick(&mut self){
        let n=self.c.fetch_add(1,Ordering::SeqCst);
        if let Some(ref t)=self.t{t.send(CmdVel::new(1.0,0.0));}
        let ts=horus_library::transform_frame::timestamp_now();
        let _=self.tf.update_transform("base_link",&Transform::xyz(0.0,0.0,0.3+(n as f64)*0.001),ts);
    }
}
#[test]
fn test_frame_can() {
    cleanup_stale_shm();
    let c=Arc::new(AtomicU64::new(0));
    let tf=Arc::new(TransformFrame::medium());
    let _=tf.register_frame("world",None);
    let _=tf.register_frame("base_link",Some("world"));
    let _=tf.register_frame("camera_link",Some("base_link"));
    let ts=horus_library::transform_frame::timestamp_now();
    let _=tf.update_transform("base_link",&Transform::xyz(0.0,0.0,0.3),ts);
    let _=tf.update_transform("camera_link",&Transform::xyz(0.1,0.0,0.5),ts);
    let _tf_h=TransformFramePublisher::spawn(&tf,10.0).unwrap();
    let mut s=Scheduler::new().tick_rate(30_u64.hz()).deterministic(true).max_deadline_misses(99999);
    s.add(B{name:"bot".into(),t:None,tf:tf.clone(),c:c.clone()}).order(0).build().unwrap();
    s.run_for(20_u64.secs()).unwrap();
}
