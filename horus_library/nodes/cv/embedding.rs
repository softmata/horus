// Feature Embedding Node for HORUS
//
// Real-time feature extraction using neural networks like CLIP, DINOv2, ResNet, etc.
// Produces dense feature vectors useful for:
// - Image similarity/search
// - Zero-shot classification
// - Visual navigation
// - Place recognition
//
// # Features
// - CLIP vision encoder support (ViT-B/32, ViT-L/14)
// - DINOv2 support (small, base, large, giant)
// - ResNet feature extraction (pretrained ImageNet)
// - EfficientNet support
// - Batch processing
// - L2 normalization (optional)
//
// # Example
// ```rust,ignore
// use horus::prelude::*;
// use horus_library::nodes::cv::EmbeddingNode;
//
// fn main() -> Result<()> {
//     let mut scheduler = Scheduler::new();
//
//     let embedding_node = EmbeddingNode::new(
//         "models/clip_vit_b32_visual.onnx",
//         "camera/raw",
//         "vision/embeddings",
//         EmbeddingConfig::clip_vit_b32(),
//     )?;
//
//     scheduler.add(Box::new(embedding_node), 1, Some(true));
//     scheduler.run()?;
//     Ok(())
// }
// ```

#[cfg(feature = "onnx")]
use ort::session::{builder::GraphOptimizationLevel, Session};
#[cfg(feature = "onnx")]
use ort::value::Tensor;

use crate::messages::ml::{EmbeddingVector, InferenceMetrics};
use crate::messages::Image;
use horus_core::{HorusError, HorusResult, Node, NodeInfo, Topic};

#[cfg(feature = "onnx")]
use ndarray::{Array, ArrayD, IxDyn};

/// Embedding model type
#[derive(Clone, Debug, PartialEq)]
pub enum EmbeddingModelType {
    /// CLIP ViT-B/32 (512-dim embeddings)
    ClipVitB32,
    /// CLIP ViT-B/16 (512-dim embeddings)
    ClipVitB16,
    /// CLIP ViT-L/14 (768-dim embeddings)
    ClipVitL14,
    /// DINOv2 Small (384-dim embeddings)
    DINOv2Small,
    /// DINOv2 Base (768-dim embeddings)
    DINOv2Base,
    /// DINOv2 Large (1024-dim embeddings)
    DINOv2Large,
    /// ResNet50 features (2048-dim)
    ResNet50,
    /// EfficientNet-B0 features (1280-dim)
    EfficientNetB0,
    /// Custom model
    Custom {
        input_size: [usize; 2],
        embedding_dim: usize,
    },
}

/// Embedding configuration
#[derive(Clone, Debug)]
pub struct EmbeddingConfig {
    /// Model type
    pub model_type: EmbeddingModelType,
    /// Input image size [height, width]
    pub input_size: [usize; 2],
    /// Embedding dimension
    pub embedding_dim: usize,
    /// Use GPU for inference
    pub use_gpu: bool,
    /// GPU device ID
    pub device_id: u32,
    /// Number of threads for CPU inference
    pub num_threads: usize,
    /// Apply L2 normalization to output
    pub normalize: bool,
    /// ImageNet normalization parameters
    pub normalize_mean: [f32; 3],
    pub normalize_std: [f32; 3],
}

impl EmbeddingConfig {
    /// CLIP ViT-B/32 configuration
    pub fn clip_vit_b32() -> Self {
        Self {
            model_type: EmbeddingModelType::ClipVitB32,
            input_size: [224, 224],
            embedding_dim: 512,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.48145466, 0.4578275, 0.40821073],
            normalize_std: [0.26862954, 0.26130258, 0.27577711],
        }
    }

    /// CLIP ViT-B/16 configuration
    pub fn clip_vit_b16() -> Self {
        Self {
            model_type: EmbeddingModelType::ClipVitB16,
            input_size: [224, 224],
            embedding_dim: 512,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.48145466, 0.4578275, 0.40821073],
            normalize_std: [0.26862954, 0.26130258, 0.27577711],
        }
    }

    /// CLIP ViT-L/14 configuration (larger, more accurate)
    pub fn clip_vit_l14() -> Self {
        Self {
            model_type: EmbeddingModelType::ClipVitL14,
            input_size: [224, 224],
            embedding_dim: 768,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.48145466, 0.4578275, 0.40821073],
            normalize_std: [0.26862954, 0.26130258, 0.27577711],
        }
    }

    /// DINOv2 Small configuration (fast)
    pub fn dinov2_small() -> Self {
        Self {
            model_type: EmbeddingModelType::DINOv2Small,
            input_size: [224, 224],
            embedding_dim: 384,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.485, 0.456, 0.406],
            normalize_std: [0.229, 0.224, 0.225],
        }
    }

    /// DINOv2 Base configuration (balanced)
    pub fn dinov2_base() -> Self {
        Self {
            model_type: EmbeddingModelType::DINOv2Base,
            input_size: [224, 224],
            embedding_dim: 768,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.485, 0.456, 0.406],
            normalize_std: [0.229, 0.224, 0.225],
        }
    }

    /// DINOv2 Large configuration (high quality)
    pub fn dinov2_large() -> Self {
        Self {
            model_type: EmbeddingModelType::DINOv2Large,
            input_size: [224, 224],
            embedding_dim: 1024,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.485, 0.456, 0.406],
            normalize_std: [0.229, 0.224, 0.225],
        }
    }

    /// ResNet50 configuration
    pub fn resnet50() -> Self {
        Self {
            model_type: EmbeddingModelType::ResNet50,
            input_size: [224, 224],
            embedding_dim: 2048,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.485, 0.456, 0.406],
            normalize_std: [0.229, 0.224, 0.225],
        }
    }

    /// EfficientNet-B0 configuration
    pub fn efficientnet_b0() -> Self {
        Self {
            model_type: EmbeddingModelType::EfficientNetB0,
            input_size: [224, 224],
            embedding_dim: 1280,
            use_gpu: false,
            device_id: 0,
            num_threads: 4,
            normalize: true,
            normalize_mean: [0.485, 0.456, 0.406],
            normalize_std: [0.229, 0.224, 0.225],
        }
    }
}

impl Default for EmbeddingConfig {
    fn default() -> Self {
        Self::clip_vit_b32()
    }
}

/// Feature Embedding Node
///
/// Extracts dense feature vectors from images using neural networks.
#[cfg(feature = "onnx")]
pub struct EmbeddingNode {
    /// Image input subscriber
    image_sub: Topic<Image>,
    /// Embedding output publisher
    embedding_pub: Topic<EmbeddingVector>,
    /// Metrics publisher
    metrics_pub: Topic<InferenceMetrics>,
    /// ONNX Runtime session
    session: Session,
    /// Configuration
    config: EmbeddingConfig,
    /// Model name
    model_name: String,
    /// Frame counter
    frame_count: u64,
}

#[cfg(feature = "onnx")]
impl EmbeddingNode {
    /// Create a new embedding node
    pub fn new(
        model_path: &str,
        input_topic: &str,
        output_topic: &str,
        config: EmbeddingConfig,
    ) -> HorusResult<Self> {
        let session = Self::load_model(model_path, &config)?;

        let model_name = std::path::Path::new(model_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("embedding")
            .to_string();

        Ok(Self {
            image_sub: Topic::new(input_topic)?,
            embedding_pub: Topic::new(output_topic)?,
            metrics_pub: Topic::new(&format!("{}.metrics", output_topic))?,
            session,
            config,
            model_name,
            frame_count: 0,
        })
    }

    /// Load ONNX model
    fn load_model(model_path: &str, config: &EmbeddingConfig) -> HorusResult<Session> {
        let mut builder = Session::builder()
            .map_err(|e| HorusError::config(format!("Failed to create session builder: {}", e)))?;

        builder = builder
            .with_optimization_level(GraphOptimizationLevel::Level3)
            .map_err(|e| HorusError::config(format!("Failed to set optimization level: {}", e)))?;

        if !config.use_gpu {
            builder = builder
                .with_intra_threads(config.num_threads)
                .map_err(|e| HorusError::config(format!("Failed to set thread count: {}", e)))?;
        }

        #[cfg(feature = "cuda")]
        if config.use_gpu {
            use ort::ExecutionProvider;
            builder = builder
                .with_execution_providers([ExecutionProvider::CUDA(Default::default())])
                .map_err(|e| HorusError::config(format!("Failed to enable CUDA: {}", e)))?;
        }

        let session = builder.commit_from_file(model_path).map_err(|e| {
            HorusError::config(format!("Failed to load embedding model {}: {}", model_path, e))
        })?;

        Ok(session)
    }

    /// Preprocess image for embedding extraction
    fn preprocess(&self, image: &Image) -> HorusResult<ArrayD<f32>> {
        let img_data = &image.data;
        let width = image.width as usize;
        let height = image.height as usize;
        let channels = (img_data.len() / (width * height)).min(3);

        let target_h = self.config.input_size[0];
        let target_w = self.config.input_size[1];

        // Center crop and resize
        // First, resize maintaining aspect ratio, then center crop
        let scale = (target_h as f32 / height as f32).max(target_w as f32 / width as f32);
        let scaled_h = (height as f32 * scale) as usize;
        let scaled_w = (width as f32 * scale) as usize;

        // Resize
        let mut resized = vec![0u8; scaled_h * scaled_w * 3];
        let scale_h = height as f32 / scaled_h as f32;
        let scale_w = width as f32 / scaled_w as f32;

        for y in 0..scaled_h {
            for x in 0..scaled_w {
                let src_y = ((y as f32 + 0.5) * scale_h) as usize;
                let src_x = ((x as f32 + 0.5) * scale_w) as usize;
                let src_y = src_y.min(height - 1);
                let src_x = src_x.min(width - 1);

                for c in 0..3 {
                    let src_idx = (src_y * width + src_x) * channels + c.min(channels - 1);
                    let dst_idx = (y * scaled_w + x) * 3 + c;
                    resized[dst_idx] = img_data.get(src_idx).copied().unwrap_or(0);
                }
            }
        }

        // Center crop
        let crop_y = (scaled_h - target_h) / 2;
        let crop_x = (scaled_w - target_w) / 2;

        let mut cropped = vec![0u8; target_h * target_w * 3];
        for y in 0..target_h {
            for x in 0..target_w {
                let src_idx = ((y + crop_y) * scaled_w + (x + crop_x)) * 3;
                let dst_idx = (y * target_w + x) * 3;
                for c in 0..3 {
                    cropped[dst_idx + c] = resized.get(src_idx + c).copied().unwrap_or(0);
                }
            }
        }

        // Normalize with model-specific parameters
        let mean = &self.config.normalize_mean;
        let std = &self.config.normalize_std;

        let mut normalized = Array::zeros(IxDyn(&[1, 3, target_h, target_w]));
        for c in 0..3 {
            for h in 0..target_h {
                for w in 0..target_w {
                    let idx = (h * target_w + w) * 3 + c;
                    let value = cropped[idx] as f32 / 255.0;
                    normalized[[0, c, h, w]] = (value - mean[c]) / std[c];
                }
            }
        }

        Ok(normalized)
    }

    /// Run inference
    fn run_inference(&mut self, input: ArrayD<f32>) -> HorusResult<Vec<f32>> {
        let input_tensor = Tensor::from_array(input)
            .map_err(|e| HorusError::config(format!("Failed to create input tensor: {}", e)))?;

        let outputs = self
            .session
            .run(ort::inputs![input_tensor])
            .map_err(|e| HorusError::config(format!("Inference failed: {}", e)))?;

        let array_view: ndarray::ArrayViewD<f32> = outputs[0]
            .try_extract_array()
            .map_err(|e| HorusError::config(format!("Failed to extract tensor: {}", e)))?;

        // Flatten to 1D vector
        let embedding: Vec<f32> = array_view.iter().copied().collect();

        Ok(embedding)
    }

    /// L2 normalize embedding vector
    fn normalize_l2(embedding: &mut [f32]) {
        let norm: f32 = embedding.iter().map(|x| x * x).sum::<f32>().sqrt();
        if norm > 1e-8 {
            for x in embedding.iter_mut() {
                *x /= norm;
            }
        }
    }

    /// Compute cosine similarity between two embeddings
    #[allow(dead_code)]
    pub fn cosine_similarity(a: &[f32], b: &[f32]) -> f32 {
        if a.len() != b.len() {
            return 0.0;
        }

        let dot: f32 = a.iter().zip(b.iter()).map(|(x, y)| x * y).sum();
        let norm_a: f32 = a.iter().map(|x| x * x).sum::<f32>().sqrt();
        let norm_b: f32 = b.iter().map(|x| x * x).sum::<f32>().sqrt();

        if norm_a > 1e-8 && norm_b > 1e-8 {
            dot / (norm_a * norm_b)
        } else {
            0.0
        }
    }
}

#[cfg(feature = "onnx")]
impl Node for EmbeddingNode {
    fn name(&self) -> &'static str {
        "EmbeddingNode"
    }

    fn tick(&mut self, mut ctx: Option<&mut NodeInfo>) {
        while let Some(image) = self.image_sub.recv(&mut ctx) {
            let start = std::time::Instant::now();

            // Preprocess
            let input = match self.preprocess(&image) {
                Ok(inp) => inp,
                Err(e) => {
                    eprintln!("Embedding preprocessing failed: {}", e);
                    continue;
                }
            };

            // Inference
            let mut embedding = match self.run_inference(input) {
                Ok(emb) => emb,
                Err(e) => {
                    eprintln!("Embedding inference failed: {}", e);
                    continue;
                }
            };

            // L2 normalize if configured
            if self.config.normalize {
                Self::normalize_l2(&mut embedding);
            }

            let latency_ms = start.elapsed().as_secs_f32() * 1000.0;

            // Create embedding message
            let embedding_msg = EmbeddingVector {
                data: embedding,
                model_name: self.model_name.clone(),
                normalized: self.config.normalize,
                timestamp_ns: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
            };

            // Publish embedding
            let _ = self.embedding_pub.send(embedding_msg, &mut ctx);

            // Publish metrics
            self.frame_count += 1;
            let metrics = InferenceMetrics {
                latency_ms,
                throughput: 1000.0 / latency_ms,
                model_name: self.model_name.clone(),
                batch_size: 1,
                timestamp_ns: std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64,
            };
            let _ = self.metrics_pub.send(metrics, &mut ctx);
        }
    }
}

// Stub implementation when ONNX is not enabled
#[cfg(not(feature = "onnx"))]
pub struct EmbeddingNode;

#[cfg(not(feature = "onnx"))]
impl EmbeddingNode {
    pub fn new(
        _model_path: &str,
        _input_topic: &str,
        _output_topic: &str,
        _config: EmbeddingConfig,
    ) -> HorusResult<Self> {
        Err(HorusError::Config(
            "ONNX support not compiled. Enable 'onnx' feature for EmbeddingNode.".to_string(),
        ))
    }
}

#[cfg(not(feature = "onnx"))]
impl Node for EmbeddingNode {
    fn name(&self) -> &'static str {
        "EmbeddingNode (disabled)"
    }

    fn tick(&mut self, _ctx: Option<&mut NodeInfo>) {
        // No-op
    }
}
