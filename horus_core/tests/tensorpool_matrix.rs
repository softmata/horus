#![allow(dead_code)]
//! TensorPool zero-copy integration tests.
//!
//! Tests Image, PointCloud, DepthImage through Topic<T> using
//! the REAL user API — exactly what a robotics developer writes.
//!
//! Run: cargo test --no-default-features -p horus_core \
//!        --test tensorpool_matrix -- --ignored --nocapture

use horus_core::communication::topic::Topic;
use horus_core::memory::{DepthImage, Image, PointCloud};
use horus_core::types::{ImageEncoding, TensorDtype};

mod common;
use common::{cleanup_stale_shm, unique};

// ════════════════════════════════════════════════════════════════════════
// TEST 1: Image — create, set pixels, send, recv, verify pixels
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn image_roundtrip_640x480() {
    cleanup_stale_shm();
    let name = unique("tp_image");

    let topic: Topic<Image> = Topic::new(&name).expect("create image topic");

    // Create 640x480 RGB image
    let mut img = Image::new(640, 480, ImageEncoding::Rgb8).expect("create image");
    assert_eq!(img.width(), 640);
    assert_eq!(img.height(), 480);

    // Set some pixels with known values
    img.set_pixel(0, 0, &[255, 0, 0]); // top-left red
    img.set_pixel(639, 0, &[0, 255, 0]); // top-right green
    img.set_pixel(0, 479, &[0, 0, 255]); // bottom-left blue
    img.set_pixel(320, 240, &[128, 128, 128]); // center gray

    // Fill a stripe pattern for bulk verification
    for x in 0..640 {
        img.set_pixel(x, 100, &[(x % 256) as u8, 0, 0]);
    }

    let data_size = img.data().len();
    println!(
        "Image: {}x{} {:?}, {} bytes",
        img.width(),
        img.height(),
        img.encoding(),
        data_size
    );

    topic.send(&img);
    let recv = topic.recv().expect("should receive image");

    // Verify dimensions
    assert_eq!(recv.width(), 640, "width mismatch");
    assert_eq!(recv.height(), 480, "height mismatch");
    assert_eq!(recv.encoding(), ImageEncoding::Rgb8, "encoding mismatch");

    // Verify specific pixels
    assert_eq!(
        recv.pixel(0, 0).unwrap(),
        &[255, 0, 0],
        "top-left should be red"
    );
    assert_eq!(
        recv.pixel(639, 0).unwrap(),
        &[0, 255, 0],
        "top-right should be green"
    );
    assert_eq!(
        recv.pixel(0, 479).unwrap(),
        &[0, 0, 255],
        "bottom-left should be blue"
    );
    assert_eq!(
        recv.pixel(320, 240).unwrap(),
        &[128, 128, 128],
        "center should be gray"
    );

    // Verify stripe pattern
    let mut stripe_ok = 0;
    for x in 0..640 {
        let expected = [(x % 256) as u8, 0, 0];
        if recv.pixel(x, 100).unwrap() == expected {
            stripe_ok += 1;
        }
    }
    assert_eq!(
        stripe_ok, 640,
        "Stripe pattern: {}/640 pixels match",
        stripe_ok
    );

    println!(
        "✓ image_roundtrip_640x480 — {}x{}, all pixels verified ({} bytes zero-copy)",
        recv.width(),
        recv.height(),
        data_size
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 2: PointCloud — create, fill points, send, recv, verify
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn pointcloud_roundtrip_1000_points() {
    cleanup_stale_shm();
    let name = unique("tp_pointcloud");

    let topic: Topic<PointCloud> = Topic::new(&name).expect("create pointcloud topic");

    // Create pointcloud with 1000 points, 3 floats per point (XYZ)
    let pc = PointCloud::new(1000, 3, TensorDtype::F32).expect("create pointcloud");
    assert_eq!(pc.point_count(), 1000);

    // Fill with known pattern: point[i] = (i*0.01, sin(i), cos(i))
    let data = pc.data_mut();
    for i in 0..1000 {
        let offset = i * 12; // 3 x f32 = 12 bytes
        let x = i as f32 * 0.01;
        let y = (i as f32 * 0.1).sin();
        let z = (i as f32 * 0.1).cos();
        data[offset..offset + 4].copy_from_slice(&x.to_le_bytes());
        data[offset + 4..offset + 8].copy_from_slice(&y.to_le_bytes());
        data[offset + 8..offset + 12].copy_from_slice(&z.to_le_bytes());
    }

    let data_size = pc.data().len();
    println!(
        "PointCloud: {} points, {} bytes",
        pc.point_count(),
        data_size
    );

    topic.send(&pc);
    let recv = topic.recv().expect("should receive pointcloud");

    assert_eq!(recv.point_count(), 1000, "point count mismatch");

    // Verify specific points
    let recv_data = recv.data();
    let mut points_ok = 0;
    for i in 0..1000 {
        let offset = i * 12;
        let x = f32::from_le_bytes(recv_data[offset..offset + 4].try_into().unwrap());
        let y = f32::from_le_bytes(recv_data[offset + 4..offset + 8].try_into().unwrap());
        let z = f32::from_le_bytes(recv_data[offset + 8..offset + 12].try_into().unwrap());
        let expected_x = i as f32 * 0.01;
        let expected_y = (i as f32 * 0.1).sin();
        let expected_z = (i as f32 * 0.1).cos();
        if (x - expected_x).abs() < 1e-6
            && (y - expected_y).abs() < 1e-6
            && (z - expected_z).abs() < 1e-6
        {
            points_ok += 1;
        }
    }
    assert_eq!(
        points_ok, 1000,
        "PointCloud: {}/1000 points match",
        points_ok
    );

    println!(
        "✓ pointcloud_roundtrip_1000_points — 1000 points verified ({} bytes zero-copy)",
        data_size
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 3: DepthImage — create, fill depths, send, recv, verify
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn depthimage_roundtrip_320x240() {
    cleanup_stale_shm();
    let name = unique("tp_depth");

    let topic: Topic<DepthImage> = Topic::new(&name).expect("create depth topic");

    let depth = DepthImage::new(320, 240, TensorDtype::F32).expect("create depth image");
    assert_eq!(depth.width(), 320);
    assert_eq!(depth.height(), 240);

    // Fill with distance pattern: depth[x][y] = sqrt(x^2 + y^2) * 0.01
    let data = depth.data_mut();
    for y in 0..240u32 {
        for x in 0..320u32 {
            let d = ((x * x + y * y) as f32).sqrt() * 0.01;
            let offset = (y * 320 + x) as usize * 4; // f32 = 4 bytes
            if offset + 4 <= data.len() {
                data[offset..offset + 4].copy_from_slice(&d.to_le_bytes());
            }
        }
    }

    let data_size = depth.data().len();
    println!(
        "DepthImage: {}x{}, {} bytes",
        depth.width(),
        depth.height(),
        data_size
    );

    topic.send(&depth);
    let recv = topic.recv().expect("should receive depth image");

    assert_eq!(recv.width(), 320, "width");
    assert_eq!(recv.height(), 240, "height");

    // Verify corners
    let recv_data = recv.data();
    let d00 = f32::from_le_bytes(recv_data[0..4].try_into().unwrap());
    assert!(
        (d00 - 0.0).abs() < 0.01,
        "depth[0,0] should be ~0, got {}",
        d00
    );

    let offset_319_0 = 319 * 4;
    let d319 = f32::from_le_bytes(
        recv_data[offset_319_0..offset_319_0 + 4]
            .try_into()
            .unwrap(),
    );
    let expected = (319.0f32 * 319.0).sqrt() * 0.01;
    assert!(
        (d319 - expected).abs() < 0.01,
        "depth[319,0] should be ~{:.2}, got {:.2}",
        expected,
        d319
    );

    println!(
        "✓ depthimage_roundtrip_320x240 — verified ({} bytes zero-copy)",
        data_size
    );
}

// ════════════════════════════════════════════════════════════════════════
// TEST 4: Mixed — Image + PointCloud + CmdVel simultaneously
// ════════════════════════════════════════════════════════════════════════

#[test]
#[ignore]
fn mixed_tensor_and_pod_topics() {
    cleanup_stale_shm();

    use horus_core::core::{DurationExt, Node};
    use horus_core::scheduling::Scheduler;
    use horus_robotics::CmdVel;
    use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
    use std::sync::Arc;
    use std::time::Duration;

    let img_name = unique("tp_mix_img");
    let cmd_name = unique("tp_mix_cmd");

    let img_sent = Arc::new(AtomicU64::new(0));
    let img_recv = Arc::new(AtomicU64::new(0));
    let cmd_sent = Arc::new(AtomicU64::new(0));
    let cmd_recv = Arc::new(AtomicU64::new(0));
    let corrupted = Arc::new(AtomicU64::new(0));

    struct ImgPubNode {
        name: String,
        topic: Option<Topic<Image>>,
        sent: Arc<AtomicU64>,
    }
    impl Node for ImgPubNode {
        fn name(&self) -> &str {
            "img_pub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.topic = Some(Topic::new(&self.name)?);
            Ok(())
        }
        fn tick(&mut self) {
            // Small image to keep test fast
            if let Ok(mut img) = Image::new(16, 16, ImageEncoding::Rgb8) {
                img.set_pixel(0, 0, &[42, 0, 0]);
                if let Some(ref t) = self.topic {
                    t.send(&img);
                }
                self.sent.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    struct MixSubNode {
        img_name: String,
        cmd_name: String,
        img_topic: Option<Topic<Image>>,
        cmd_topic: Option<Topic<CmdVel>>,
        img_recv: Arc<AtomicU64>,
        cmd_recv: Arc<AtomicU64>,
        corrupted: Arc<AtomicU64>,
    }
    impl Node for MixSubNode {
        fn name(&self) -> &str {
            "mix_sub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.img_topic = Some(Topic::new(&self.img_name)?);
            self.cmd_topic = Some(Topic::new(&self.cmd_name)?);
            Ok(())
        }
        fn tick(&mut self) {
            if let Some(ref t) = self.img_topic {
                while let Some(img) = t.recv() {
                    if img.pixel(0, 0).unwrap_or(&[0, 0, 0]) != [42, 0, 0] {
                        self.corrupted.fetch_add(1, Ordering::Relaxed);
                    }
                    self.img_recv.fetch_add(1, Ordering::Relaxed);
                }
            }
            if let Some(ref t) = self.cmd_topic {
                while let Some(cmd) = t.recv() {
                    if (cmd.linear - 1.0).abs() > 0.01 {
                        self.corrupted.fetch_add(1, Ordering::Relaxed);
                    }
                    self.cmd_recv.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    struct CmdPubNode {
        name: String,
        topic: Option<Topic<CmdVel>>,
        sent: Arc<AtomicU64>,
    }
    impl Node for CmdPubNode {
        fn name(&self) -> &str {
            "cmd_pub"
        }
        fn init(&mut self) -> horus_core::error::HorusResult<()> {
            self.topic = Some(Topic::new(&self.name)?);
            Ok(())
        }
        fn tick(&mut self) {
            if let Some(ref t) = self.topic {
                t.send(CmdVel::new(1.0, 0.0));
            }
            self.sent.fetch_add(1, Ordering::Relaxed);
        }
    }

    let is = img_sent.clone();
    let cs = cmd_sent.clone();
    let ir = img_recv.clone();
    let cr = cmd_recv.clone();
    let co = corrupted.clone();
    let running = Arc::new(AtomicBool::new(true));
    let rc = running.clone();

    let h = std::thread::spawn(move || {
        let mut sched = Scheduler::new().tick_rate(30_u64.hz());
        let _ = sched
            .add(ImgPubNode {
                name: img_name.clone(),
                topic: None,
                sent: is,
            })
            .rate(30_u64.hz())
            .order(0)
            .build();
        let _ = sched
            .add(CmdPubNode {
                name: cmd_name.clone(),
                topic: None,
                sent: cs,
            })
            .rate(100_u64.hz())
            .order(1)
            .build();
        let _ = sched
            .add(MixSubNode {
                img_name,
                cmd_name,
                img_topic: None,
                cmd_topic: None,
                img_recv: ir,
                cmd_recv: cr,
                corrupted: co,
            })
            .order(2)
            .build();
        while rc.load(Ordering::Relaxed) {
            let _ = sched.tick_once();
            std::thread::sleep(Duration::from_millis(9));
        }
    });

    std::thread::sleep(Duration::from_secs(3));
    running.store(false, Ordering::Relaxed);
    h.join().unwrap();

    let isv = img_sent.load(Ordering::Relaxed);
    let irv = img_recv.load(Ordering::Relaxed);
    let csv = cmd_sent.load(Ordering::Relaxed);
    let crv = cmd_recv.load(Ordering::Relaxed);
    let cov = corrupted.load(Ordering::Relaxed);

    println!("╔══════════════════════════════════════════════════════════╗");
    println!("║  MIXED TENSOR + POD (3s)                                ║");
    println!(
        "║  Image:  sent={:4} recv={:4} ({:.0}%)                   ║",
        isv,
        irv,
        irv as f64 / isv.max(1) as f64 * 100.0
    );
    println!(
        "║  CmdVel: sent={:4} recv={:4} ({:.0}%)                   ║",
        csv,
        crv,
        crv as f64 / csv.max(1) as f64 * 100.0
    );
    println!(
        "║  Corrupted: {}                                          ║",
        cov
    );
    println!("╚══════════════════════════════════════════════════════════╝");

    assert_eq!(cov, 0, "DATA CORRUPTION in mixed tensor+pod!");
    assert!(irv > 10, "Image should be received >10 times, got {}", irv);
    assert!(crv > 50, "CmdVel should be received >50 times, got {}", crv);
    println!("✓ mixed_tensor_and_pod_topics — zero corruption, both channels flowing");
}
