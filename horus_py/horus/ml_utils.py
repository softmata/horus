"""
HORUS Machine Learning Utilities

Helper functions and base classes for integrating ML models with HORUS.
Supports PyTorch, TensorFlow, ONNX, and other frameworks.
"""

import numpy as np
from typing import Optional, Dict, Any, List, Tuple, Union
import time


class MLNodeBase:
    """
    Base class for ML inference nodes in HORUS.

    Provides common functionality for loading models, preprocessing,
    and postprocessing with performance tracking.
    """

    def __init__(self, model_path: str, input_topic: str, output_topic: str):
        """
        Initialize ML node base.

        Args:
            model_path: Path to the model file
            input_topic: Topic to subscribe to for inputs
            output_topic: Topic to publish predictions to
        """
        self.model_path = model_path
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.model = None
        self.inference_count = 0
        self.total_latency_ms = 0.0

    def load_model(self):
        """Load the model - override in subclass"""
        raise NotImplementedError("Subclass must implement load_model()")

    def preprocess(self, data: Any) -> Any:
        """Preprocess input data - override if needed"""
        return data

    def infer(self, data: Any) -> Any:
        """Run inference - override in subclass"""
        raise NotImplementedError("Subclass must implement infer()")

    def postprocess(self, output: Any) -> Any:
        """Postprocess model output - override if needed"""
        return output

    def run_inference(self, data: Any) -> Tuple[Any, float]:
        """
        Run full inference pipeline with timing.

        Returns:
            Tuple of (result, latency_ms)
        """
        start = time.perf_counter()

        # Preprocess
        preprocessed = self.preprocess(data)

        # Inference
        output = self.infer(preprocessed)

        # Postprocess
        result = self.postprocess(output)

        latency_ms = (time.perf_counter() - start) * 1000.0

        self.inference_count += 1
        self.total_latency_ms += latency_ms

        return result, latency_ms

    def get_stats(self) -> Dict[str, float]:
        """Get inference statistics"""
        avg_latency = (
            self.total_latency_ms / self.inference_count
            if self.inference_count > 0
            else 0.0
        )
        return {
            "inference_count": self.inference_count,
            "avg_latency_ms": avg_latency,
            "total_latency_ms": self.total_latency_ms,
        }


class PyTorchInferenceNode(MLNodeBase):
    """
    PyTorch model inference node for HORUS.

    Example:
        ```python
        import torch
        from horus import Node, Topic
        from horus.ml_utils import PyTorchInferenceNode

        class MyPyTorchNode(PyTorchInferenceNode):
            def load_model(self):
                self.model = torch.jit.load(self.model_path)
                self.model.eval()

            def preprocess(self, image_data):
                # Convert to tensor and normalize
                tensor = torch.from_numpy(image_data).float() / 255.0
                return tensor.unsqueeze(0)  # Add batch dimension

            def infer(self, tensor):
                with torch.no_grad():
                    return self.model(tensor)

        node = MyPyTorchNode(
            "models/resnet50.pt",
            "camera/raw",
            "predictions"
        )
        ```
    """

    def __init__(
        self,
        model_path: str,
        input_topic: str,
        output_topic: str,
        device: str = "cpu",
    ):
        """
        Initialize PyTorch inference node.

        Args:
            model_path: Path to PyTorch model (.pt or .pth)
            input_topic: Input topic name
            output_topic: Output topic name
            device: Device to run on ("cpu", "cuda", "cuda:0", etc.)
        """
        super().__init__(model_path, input_topic, output_topic)
        self.device = device

    def load_model(self):
        """Load PyTorch model"""
        try:
            import torch
        except ImportError:
            raise ImportError(
                "PyTorch not installed. Install with: pip install torch"
            )

        # Try TorchScript first, then standard checkpoint
        try:
            self.model = torch.jit.load(self.model_path, map_location=self.device)
        except RuntimeError:
            # Not a TorchScript model, try regular checkpoint
            self.model = torch.load(self.model_path, map_location=self.device)

        self.model.eval()
        self.model.to(self.device)

    def infer(self, data):
        """Run PyTorch inference"""
        import torch

        if not isinstance(data, torch.Tensor):
            data = torch.from_numpy(data).to(self.device)

        with torch.no_grad():
            output = self.model(data)

        # Convert back to numpy
        if isinstance(output, torch.Tensor):
            output = output.cpu().numpy()

        return output


class TensorFlowInferenceNode(MLNodeBase):
    """
    TensorFlow/Keras model inference node for HORUS.

    Example:
        ```python
        from horus import Node, Topic
        from horus.ml_utils import TensorFlowInferenceNode

        class MyTFNode(TensorFlowInferenceNode):
            def preprocess(self, image_data):
                # Resize and normalize
                import tensorflow as tf
                img = tf.image.resize(image_data, [224, 224])
                img = img / 255.0
                return tf.expand_dims(img, 0)  # Add batch dimension

        node = MyTFNode(
            "models/mobilenet_v2",
            "camera/raw",
            "predictions"
        )
        ```
    """

    def __init__(
        self,
        model_path: str,
        input_topic: str,
        output_topic: str,
        use_gpu: bool = False,
    ):
        """
        Initialize TensorFlow inference node.

        Args:
            model_path: Path to TensorFlow SavedModel or Keras .h5 file
            input_topic: Input topic name
            output_topic: Output topic name
            use_gpu: Whether to use GPU acceleration
        """
        super().__init__(model_path, input_topic, output_topic)
        self.use_gpu = use_gpu

    def load_model(self):
        """Load TensorFlow model"""
        try:
            import tensorflow as tf
        except ImportError:
            raise ImportError(
                "TensorFlow not installed. Install with: pip install tensorflow"
            )

        # Configure GPU
        if not self.use_gpu:
            import os
            os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

        # Load model (handles both SavedModel and .h5)
        if self.model_path.endswith(".h5") or self.model_path.endswith(".keras"):
            self.model = tf.keras.models.load_model(self.model_path)
        else:
            self.model = tf.saved_model.load(self.model_path)

    def infer(self, data):
        """Run TensorFlow inference"""
        import tensorflow as tf

        if not isinstance(data, tf.Tensor):
            data = tf.convert_to_tensor(data)

        output = self.model(data)

        # Convert to numpy
        if isinstance(output, tf.Tensor):
            output = output.numpy()

        return output


class ONNXInferenceNode(MLNodeBase):
    """
    ONNX model inference node for HORUS.

    Works with models exported from PyTorch, TensorFlow, scikit-learn, etc.

    Example:
        ```python
        from horus import Node, Topic
        from horus.ml_utils import ONNXInferenceNode

        class MyONNXNode(ONNXInferenceNode):
            def preprocess(self, image_data):
                # Normalize and reshape to NCHW
                img = image_data.astype(np.float32) / 255.0
                img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
                img = np.expand_dims(img, 0)  # Add batch dimension
                return img

        node = MyONNXNode(
            "models/yolov8n.onnx",
            "camera/raw",
            "detections"
        )
        ```
    """

    def __init__(
        self,
        model_path: str,
        input_topic: str,
        output_topic: str,
        use_gpu: bool = False,
    ):
        """
        Initialize ONNX inference node.

        Args:
            model_path: Path to ONNX model file
            input_topic: Input topic name
            output_topic: Output topic name
            use_gpu: Whether to use GPU (CUDA) acceleration
        """
        super().__init__(model_path, input_topic, output_topic)
        self.use_gpu = use_gpu

    def load_model(self):
        """Load ONNX model"""
        try:
            import onnxruntime as ort
        except ImportError:
            raise ImportError(
                "ONNX Runtime not installed. Install with: pip install onnxruntime"
            )

        # Configure providers
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if self.use_gpu else ["CPUExecutionProvider"]

        self.session = ort.InferenceSession(self.model_path, providers=providers)

        # Get input/output names
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

    def infer(self, data):
        """Run ONNX inference"""
        # Run inference
        outputs = self.session.run(self.output_names, {self.input_name: data})

        # Return first output if single output, else return all
        return outputs[0] if len(outputs) == 1 else outputs


def preprocess_image_imagenet(
    image: np.ndarray,
    target_size: Tuple[int, int] = (224, 224),
    mean: List[float] = [0.485, 0.456, 0.406],
    std: List[float] = [0.229, 0.224, 0.225],
) -> np.ndarray:
    """
    Standard ImageNet preprocessing for classification models.

    Args:
        image: Input image (H, W, C) in 0-255 range
        target_size: Target (height, width)
        mean: Normalization mean per channel
        std: Normalization std per channel

    Returns:
        Preprocessed image (1, C, H, W)
    """
    # Resize
    import cv2
    image = cv2.resize(image, (target_size[1], target_size[0]))

    # Normalize to 0-1
    image = image.astype(np.float32) / 255.0

    # Apply mean/std normalization
    image = (image - mean) / std

    # HWC -> CHW
    image = np.transpose(image, (2, 0, 1))

    # Add batch dimension
    image = np.expand_dims(image, 0)

    return image


def preprocess_image_yolo(
    image: np.ndarray,
    target_size: int = 640,
) -> Tuple[np.ndarray, float, float, float]:
    """
    YOLOv8-style preprocessing with letterbox padding.

    Args:
        image: Input image (H, W, C) in 0-255 range
        target_size: Target square size (640, 320, etc.)

    Returns:
        Tuple of (preprocessed_image, scale, pad_w, pad_h)
    """
    import cv2

    h, w = image.shape[:2]

    # Calculate scale and padding
    scale = min(target_size / w, target_size / h)
    new_w = int(w * scale)
    new_h = int(h * scale)
    pad_w = (target_size - new_w) / 2
    pad_h = (target_size - new_h) / 2

    # Resize
    image = cv2.resize(image, (new_w, new_h))

    # Create padded image
    padded = np.zeros((target_size, target_size, 3), dtype=np.uint8)
    pad_w_int = int(pad_w)
    pad_h_int = int(pad_h)
    padded[pad_h_int:pad_h_int + new_h, pad_w_int:pad_w_int + new_w] = image

    # Normalize to 0-1
    padded = padded.astype(np.float32) / 255.0

    # HWC -> CHW
    padded = np.transpose(padded, (2, 0, 1))

    # Add batch dimension
    padded = np.expand_dims(padded, 0)

    return padded, scale, pad_w, pad_h


def nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    iou_threshold: float = 0.45,
) -> List[int]:
    """
    Non-Maximum Suppression for bounding boxes.

    Args:
        boxes: Array of boxes (N, 4) as [x, y, w, h]
        scores: Array of scores (N,)
        iou_threshold: IoU threshold for suppression

    Returns:
        List of indices to keep
    """
    # Sort by score
    indices = np.argsort(scores)[::-1]

    keep = []
    while len(indices) > 0:
        current = indices[0]
        keep.append(current)

        if len(indices) == 1:
            break

        # Calculate IoU with remaining boxes
        current_box = boxes[current]
        remaining_boxes = boxes[indices[1:]]

        ious = calculate_iou(current_box, remaining_boxes)

        # Keep boxes with IoU below threshold
        indices = indices[1:][ious < iou_threshold]

    return keep


def calculate_iou(box1: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    """
    Calculate IoU between one box and multiple boxes.

    Args:
        box1: Single box [x, y, w, h]
        boxes: Multiple boxes (N, 4) as [x, y, w, h]

    Returns:
        Array of IoU values (N,)
    """
    # Convert to [x1, y1, x2, y2]
    box1_x2 = box1[0] + box1[2]
    box1_y2 = box1[1] + box1[3]

    boxes_x2 = boxes[:, 0] + boxes[:, 2]
    boxes_y2 = boxes[:, 1] + boxes[:, 3]

    # Calculate intersection
    x1_max = np.maximum(box1[0], boxes[:, 0])
    y1_max = np.maximum(box1[1], boxes[:, 1])
    x2_min = np.minimum(box1_x2, boxes_x2)
    y2_min = np.minimum(box1_y2, boxes_y2)

    intersection_w = np.maximum(0, x2_min - x1_max)
    intersection_h = np.maximum(0, y2_min - y1_max)
    intersection = intersection_w * intersection_h

    # Calculate union
    box1_area = box1[2] * box1[3]
    boxes_area = boxes[:, 2] * boxes[:, 3]
    union = box1_area + boxes_area - intersection

    # Calculate IoU
    iou = intersection / (union + 1e-6)

    return iou


class PerformanceMonitor:
    """
    Monitor ML inference performance in real-time.
    """

    def __init__(self, window_size: int = 100):
        """
        Initialize performance monitor.

        Args:
            window_size: Number of samples to keep for rolling statistics
        """
        self.window_size = window_size
        self.latencies = []
        self.throughputs = []
        self.reset()

    def reset(self):
        """Reset all statistics"""
        self.latencies = []
        self.throughputs = []
        self.start_time = time.time()

    def record(self, latency_ms: float):
        """Record a single inference"""
        self.latencies.append(latency_ms)

        # Keep only last window_size samples
        if len(self.latencies) > self.window_size:
            self.latencies.pop(0)

        # Calculate throughput
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            throughput = len(self.latencies) / elapsed
            self.throughputs.append(throughput)
            if len(self.throughputs) > self.window_size:
                self.throughputs.pop(0)

    def get_stats(self) -> Dict[str, float]:
        """Get current statistics"""
        if not self.latencies:
            return {
                "avg_latency_ms": 0.0,
                "min_latency_ms": 0.0,
                "max_latency_ms": 0.0,
                "fps": 0.0,
                "samples": 0,
            }

        return {
            "avg_latency_ms": np.mean(self.latencies),
            "min_latency_ms": np.min(self.latencies),
            "max_latency_ms": np.max(self.latencies),
            "std_latency_ms": np.std(self.latencies),
            "p50_latency_ms": np.percentile(self.latencies, 50),
            "p95_latency_ms": np.percentile(self.latencies, 95),
            "p99_latency_ms": np.percentile(self.latencies, 99),
            "fps": 1000.0 / np.mean(self.latencies),
            "samples": len(self.latencies),
        }

    def print_stats(self):
        """Print current statistics"""
        stats = self.get_stats()
        print(f"\n=== Performance Statistics ===")
        print(f"Samples: {stats['samples']}")
        print(f"Avg Latency: {stats['avg_latency_ms']:.2f} ms")
        print(f"Min Latency: {stats['min_latency_ms']:.2f} ms")
        print(f"Max Latency: {stats['max_latency_ms']:.2f} ms")
        print(f"P95 Latency: {stats['p95_latency_ms']:.2f} ms")
        print(f"FPS: {stats['fps']:.1f}")
        print(f"==============================\n")
