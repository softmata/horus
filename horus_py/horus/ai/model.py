"""
HORUS Model - Unified model loading and inference.

Supports PyTorch, TensorFlow, ONNX, and TFLite models with
automatic format detection and device placement.
"""

from typing import Optional, Union, List, Dict, Any, Callable, Tuple
from pathlib import Path
import time
import numpy as np

from .device import Device, get_default_device


class Model:
    """
    Unified model loader and inference engine.

    Automatically detects model format from file extension and loads
    with the appropriate backend.

    Example:
        # Auto-detect format
        model = Model.load("yolov8n.onnx")
        model = Model.load("resnet50.pt", device="cuda:0")

        # Run inference
        output = model(input_tensor)
        output = model.predict(input_tensor)

        # With preprocessing
        model.preprocess = my_preprocess_fn
        output = model(raw_image)
    """

    def __init__(
        self,
        model: Any,
        backend: str,
        device: str = "cpu",
        input_names: Optional[List[str]] = None,
        output_names: Optional[List[str]] = None,
    ):
        """
        Initialize model wrapper (use Model.load() instead).

        Args:
            model: The underlying model object
            backend: Backend type ("onnx", "pytorch", "tensorflow", "tflite")
            device: Device placement
            input_names: Input tensor names (for ONNX)
            output_names: Output tensor names (for ONNX)
        """
        self._model = model
        self._backend = backend
        self._device = device
        self._input_names = input_names or []
        self._output_names = output_names or []

        # Optional preprocessing/postprocessing
        self.preprocess: Optional[Callable[[Any], np.ndarray]] = None
        self.postprocess: Optional[Callable[[Any], Any]] = None

        # Performance tracking
        self._inference_count = 0
        self._total_latency_ms = 0.0

    # ══════════════════════════════════════════════════════════════
    # Factory Methods
    # ══════════════════════════════════════════════════════════════

    @classmethod
    def load(
        cls,
        path: Union[str, Path],
        device: Optional[str] = None,
        format: Optional[str] = None,
    ) -> 'Model':
        """
        Load a model from file with automatic format detection.

        Args:
            path: Path to model file
            device: Target device ("cpu", "cuda:0", etc.)
            format: Explicit format ("onnx", "pytorch", "tensorflow", "tflite")

        Returns:
            Loaded Model instance

        Example:
            model = Model.load("yolov8n.onnx")
            model = Model.load("resnet50.pt", device="cuda:0")
            model = Model.load("model.bin", format="onnx")
        """
        path = Path(path)
        device = device or get_default_device()

        # Auto-detect format from extension
        if format is None:
            ext = path.suffix.lower()
            format = {
                '.onnx': 'onnx',
                '.pt': 'pytorch',
                '.pth': 'pytorch',
                '.h5': 'tensorflow',
                '.keras': 'tensorflow',
                '.pb': 'tensorflow',
                '.tflite': 'tflite',
            }.get(ext)

            # Check for SavedModel directory
            if format is None and path.is_dir():
                if (path / "saved_model.pb").exists():
                    format = 'tensorflow'

            if format is None:
                raise ValueError(
                    f"Cannot determine model format from '{path}'. "
                    "Use format='onnx'/'pytorch'/'tensorflow'/'tflite'"
                )

        # Load based on format
        if format == 'onnx':
            return cls._load_onnx(path, device)
        elif format == 'pytorch':
            return cls._load_pytorch(path, device)
        elif format == 'tensorflow':
            return cls._load_tensorflow(path, device)
        elif format == 'tflite':
            return cls._load_tflite(path, device)
        else:
            raise ValueError(f"Unknown format: {format}")

    @classmethod
    def _load_onnx(cls, path: Path, device: str) -> 'Model':
        """Load ONNX model."""
        try:
            import onnxruntime as ort
        except ImportError:
            raise ImportError(
                "ONNX Runtime not installed. Install with: pip install onnxruntime"
            )

        # Configure providers based on device
        if device.startswith("cuda"):
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
        else:
            providers = ["CPUExecutionProvider"]

        session = ort.InferenceSession(str(path), providers=providers)

        # Get input/output names
        input_names = [inp.name for inp in session.get_inputs()]
        output_names = [out.name for out in session.get_outputs()]

        return cls(
            model=session,
            backend="onnx",
            device=device,
            input_names=input_names,
            output_names=output_names,
        )

    @classmethod
    def _load_pytorch(cls, path: Path, device: str) -> 'Model':
        """Load PyTorch model."""
        try:
            import torch
        except ImportError:
            raise ImportError(
                "PyTorch not installed. Install with: pip install torch"
            )

        # Map device string to torch device
        torch_device = torch.device(device)

        # Try TorchScript first
        try:
            model = torch.jit.load(str(path), map_location=torch_device)
        except RuntimeError:
            # Fall back to regular checkpoint
            model = torch.load(str(path), map_location=torch_device)

        model.eval()
        model.to(torch_device)

        return cls(
            model=model,
            backend="pytorch",
            device=device,
        )

    @classmethod
    def _load_tensorflow(cls, path: Path, device: str) -> 'Model':
        """Load TensorFlow/Keras model."""
        try:
            import tensorflow as tf
        except ImportError:
            raise ImportError(
                "TensorFlow not installed. Install with: pip install tensorflow"
            )

        # Configure device
        if not device.startswith("cuda"):
            import os
            os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

        # Load model
        path_str = str(path)
        if path_str.endswith(('.h5', '.keras')):
            model = tf.keras.models.load_model(path_str)
        else:
            model = tf.saved_model.load(path_str)

        return cls(
            model=model,
            backend="tensorflow",
            device=device,
        )

    @classmethod
    def _load_tflite(cls, path: Path, device: str) -> 'Model':
        """Load TFLite model."""
        try:
            import tensorflow as tf
        except ImportError:
            raise ImportError(
                "TensorFlow not installed. Install with: pip install tensorflow"
            )

        interpreter = tf.lite.Interpreter(model_path=str(path))
        interpreter.allocate_tensors()

        # Get input/output details
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        return cls(
            model=interpreter,
            backend="tflite",
            device=device,
            input_names=[d['name'] for d in input_details],
            output_names=[d['name'] for d in output_details],
        )

    # ══════════════════════════════════════════════════════════════
    # Inference
    # ══════════════════════════════════════════════════════════════

    def __call__(self, *inputs) -> Union[np.ndarray, List[np.ndarray]]:
        """
        Run inference on input(s).

        Args:
            *inputs: Input tensor(s)

        Returns:
            Output numpy array(s)
        """
        return self.predict(*inputs)

    def predict(self, *inputs) -> Union[np.ndarray, List[np.ndarray]]:
        """
        Run inference on input(s).

        Args:
            *inputs: Input tensor(s)

        Returns:
            Output tensor(s)
        """
        start = time.perf_counter()

        # Apply preprocessing if defined
        if self.preprocess is not None:
            inputs = tuple(self.preprocess(inp) for inp in inputs)

        # Convert to appropriate format
        inputs = self._prepare_inputs(inputs)

        # Run inference based on backend
        if self._backend == "onnx":
            outputs = self._infer_onnx(inputs)
        elif self._backend == "pytorch":
            outputs = self._infer_pytorch(inputs)
        elif self._backend == "tensorflow":
            outputs = self._infer_tensorflow(inputs)
        elif self._backend == "tflite":
            outputs = self._infer_tflite(inputs)
        else:
            raise RuntimeError(f"Unknown backend: {self._backend}")

        # Apply postprocessing if defined
        if self.postprocess is not None:
            outputs = self.postprocess(outputs)

        # Track performance
        latency_ms = (time.perf_counter() - start) * 1000.0
        self._inference_count += 1
        self._total_latency_ms += latency_ms

        return outputs

    def predict_batch(self, inputs: List[Any]) -> List[Any]:
        """
        Run inference on a batch of inputs.

        Args:
            inputs: List of inputs

        Returns:
            List of outputs
        """
        return [self.predict(inp) for inp in inputs]

    def _prepare_inputs(self, inputs: tuple) -> tuple:
        """Convert inputs to numpy arrays."""
        result = []
        for inp in inputs:
            if isinstance(inp, np.ndarray):
                result.append(inp)
            elif hasattr(inp, '__array__'):
                result.append(np.asarray(inp))
            else:
                result.append(np.array(inp))

        return tuple(result)

    def _infer_onnx(self, inputs: tuple) -> Union[np.ndarray, List[np.ndarray]]:
        """Run ONNX inference."""
        # Build input dict
        input_dict = {}
        for i, name in enumerate(self._input_names):
            if i < len(inputs):
                input_dict[name] = inputs[i]

        # Run inference
        outputs = self._model.run(self._output_names, input_dict)

        if len(outputs) == 1:
            return outputs[0]
        return list(outputs)

    def _infer_pytorch(self, inputs: tuple) -> Union[np.ndarray, List[np.ndarray]]:
        """Run PyTorch inference."""
        import torch

        torch_device = torch.device(self._device)

        # Convert to tensors
        torch_inputs = []
        for inp in inputs:
            t = torch.from_numpy(inp).to(torch_device)
            torch_inputs.append(t)

        # Run inference
        with torch.no_grad():
            if len(torch_inputs) == 1:
                output = self._model(torch_inputs[0])
            else:
                output = self._model(*torch_inputs)

        # Convert output to numpy
        if isinstance(output, torch.Tensor):
            return output.cpu().numpy()
        elif isinstance(output, (tuple, list)):
            return [o.cpu().numpy() for o in output]
        return output

    def _infer_tensorflow(self, inputs: tuple) -> Union[np.ndarray, List[np.ndarray]]:
        """Run TensorFlow inference."""
        import tensorflow as tf

        # Convert inputs
        tf_inputs = [tf.convert_to_tensor(inp) for inp in inputs]

        # Run inference
        if len(tf_inputs) == 1:
            output = self._model(tf_inputs[0])
        else:
            output = self._model(*tf_inputs)

        # Convert output to numpy
        if isinstance(output, tf.Tensor):
            return output.numpy()
        elif isinstance(output, (tuple, list)):
            return [o.numpy() for o in output]
        return output

    def _infer_tflite(self, inputs: tuple) -> Union[np.ndarray, List[np.ndarray]]:
        """Run TFLite inference."""
        # Get input/output details
        input_details = self._model.get_input_details()
        output_details = self._model.get_output_details()

        # Set inputs
        for i, detail in enumerate(input_details):
            if i < len(inputs):
                self._model.set_tensor(detail['index'], inputs[i].astype(np.float32))

        # Run inference
        self._model.invoke()

        # Get outputs
        outputs = [self._model.get_tensor(detail['index']) for detail in output_details]

        if len(outputs) == 1:
            return outputs[0]
        return outputs

    # ══════════════════════════════════════════════════════════════
    # Properties and Stats
    # ══════════════════════════════════════════════════════════════

    @property
    def backend(self) -> str:
        """Get the model backend type."""
        return self._backend

    @property
    def device(self) -> str:
        """Get the device this model is on."""
        return self._device

    @property
    def input_names(self) -> List[str]:
        """Get input tensor names."""
        return self._input_names

    @property
    def output_names(self) -> List[str]:
        """Get output tensor names."""
        return self._output_names

    def stats(self) -> Dict[str, float]:
        """Get inference statistics."""
        avg_latency = (
            self._total_latency_ms / self._inference_count
            if self._inference_count > 0
            else 0.0
        )
        fps = 1000.0 / avg_latency if avg_latency > 0 else 0.0

        return {
            "inference_count": self._inference_count,
            "avg_latency_ms": avg_latency,
            "total_latency_ms": self._total_latency_ms,
            "fps": fps,
        }

    def reset_stats(self) -> None:
        """Reset performance statistics."""
        self._inference_count = 0
        self._total_latency_ms = 0.0

    def to(self, device: str) -> 'Model':
        """
        Move model to a different device.

        Args:
            device: Target device

        Returns:
            Model on the new device (may be a new instance)
        """
        if device == self._device:
            return self

        # For PyTorch, we can move in-place
        if self._backend == "pytorch":
            import torch
            torch_device = torch.device(device)
            self._model.to(torch_device)
            self._device = device
            return self

        # For other backends, may need to reload
        raise NotImplementedError(
            f"Cannot move {self._backend} model to {device}. "
            "Reload the model with the target device."
        )

    def __repr__(self) -> str:
        return f"Model(backend='{self._backend}', device='{self._device}')"
