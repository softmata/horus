"""
HORUS ML Node - Decorator for AI-powered nodes.

Provides a clean decorator syntax for creating nodes that
run ML inference as part of the HORUS node graph.
"""

from typing import Optional, Union, Callable, Type, Any, Dict
from pathlib import Path
import numpy as np

from .model import Model


class MLNodeConfig:
    """Configuration for an ML node."""

    def __init__(
        self,
        model: Union[str, Path, Model],
        input_topic: str,
        output_topic: str,
        device: str = "cpu",
        rate: float = 30.0,
        preprocess: Optional[Callable] = None,
        postprocess: Optional[Callable] = None,
    ):
        self.model_path = model if isinstance(model, (str, Path)) else None
        self.model_instance = model if isinstance(model, Model) else None
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.device = device
        self.rate = rate
        self.preprocess = preprocess
        self.postprocess = postprocess


def mlnode(
    model: Optional[Union[str, Path, Model]] = None,
    input: Optional[str] = None,
    output: Optional[str] = None,
    device: str = "cpu",
    rate: float = 30.0,
):
    """
    Decorator to create an ML inference node.

    Can be used in two ways:

    1. As a class decorator with explicit preprocess/postprocess:

        @mlnode(
            model="models/detector.onnx",
            input="camera.image",
            output="detections",
            device="cuda:0",
        )
        class DetectorNode:
            def preprocess(self, image: np.ndarray) -> np.ndarray:
                return preprocess_fn(image)

            def postprocess(self, output: np.ndarray) -> list:
                return self.nms(output, threshold=0.5)

    2. As a factory function for simple nodes:

        DetectorNode = mlnode.from_model(
            "models/detector.onnx",
            input="camera.image",
            output="detections",
        )

    Args:
        model: Path to model file or Model instance
        input: Input topic name
        output: Output topic name
        device: Device for inference ("cpu", "cuda:0", etc.)
        rate: Node tick rate in Hz

    Returns:
        Decorated class or node factory
    """

    def decorator(cls: Type) -> Type:
        """Decorate a class to become an ML node."""

        # Import here to avoid circular imports
        from horus import Node, Topic

        # Store original __init__ if it exists
        original_init = cls.__init__ if hasattr(cls, '__init__') else None

        # Get preprocess/postprocess methods
        preprocess_fn = getattr(cls, 'preprocess', None)
        postprocess_fn = getattr(cls, 'postprocess', None)

        def new_init(self, *args, **kwargs):
            # Call original __init__ if it exists
            if original_init and original_init is not object.__init__:
                original_init(self, *args, **kwargs)

            # Load model
            if isinstance(model, Model):
                self._model = model
            else:
                self._model = Model.load(model, device=device)

            # Set up preprocess/postprocess
            if preprocess_fn:
                self._model.preprocess = lambda x: preprocess_fn(self, x)
            if postprocess_fn:
                self._model.postprocess = lambda x: postprocess_fn(self, x)

            # Store config
            self._input_topic = input
            self._output_topic = output
            self._device = device

        def tick(self, node: 'Node') -> None:
            """ML node tick function."""
            if node.has_msg(self._input_topic):
                data = node.get(self._input_topic)

                # Ensure numpy array
                if not isinstance(data, np.ndarray):
                    data = np.asarray(data)

                # Run inference (model returns np.ndarray)
                output = self._model(data)

                node.send(self._output_topic, output)

        # Create the wrapped class
        class WrappedMLNode(cls):
            def __init__(self, *args, **kwargs):
                new_init(self, *args, **kwargs)

            def create_node(self) -> 'Node':
                """Create a HORUS Node from this ML node."""
                return Node(
                    name=cls.__name__,
                    subs=[self._input_topic],
                    pubs=[self._output_topic],
                    tick=lambda n: tick(self, n),
                    rate=rate,
                )

        # Copy class attributes
        WrappedMLNode.__name__ = cls.__name__
        WrappedMLNode.__qualname__ = cls.__qualname__
        WrappedMLNode.__doc__ = cls.__doc__

        return WrappedMLNode

    # If called without arguments, return decorator
    if model is None:
        return decorator

    # If model is actually a class (decorator used without parens), apply directly
    if isinstance(model, type):
        cls = model
        model = None  # Reset to trigger error
        raise TypeError(
            "mlnode() requires model argument. Use @mlnode(model='path.onnx', ...)"
        )

    return decorator


def from_model(
    model: Union[str, Path, Model],
    input: str,
    output: str,
    device: str = "cpu",
    rate: float = 30.0,
    preprocess: Optional[Callable[[Any], np.ndarray]] = None,
    postprocess: Optional[Callable[[Any], Any]] = None,
) -> 'Node':
    """
    Create a simple ML inference node from a model.

    This is a convenience function for creating nodes that just run
    inference without custom logic.

    Example:
        detector = mlnode.from_model(
            "models/yolov8n.onnx",
            input="camera.image",
            output="detections",
            device="cuda:0",
        )

        run(detector)

    Args:
        model: Path to model file or Model instance
        input: Input topic name
        output: Output topic name
        device: Device for inference
        rate: Node tick rate in Hz
        preprocess: Optional preprocessing function
        postprocess: Optional postprocessing function

    Returns:
        Configured HORUS Node
    """
    from horus import Node

    # Load model
    if isinstance(model, (str, Path)):
        loaded_model = Model.load(model, device=device)
    else:
        loaded_model = model

    # Set preprocessing/postprocessing
    if preprocess:
        loaded_model.preprocess = preprocess
    if postprocess:
        loaded_model.postprocess = postprocess

    def tick(node: Node) -> None:
        if node.has_msg(input):
            data = node.get(input)

            # Ensure numpy array
            if not isinstance(data, np.ndarray):
                data = np.asarray(data)

            # Run inference (model returns np.ndarray)
            result = loaded_model(data)

            node.send(output, result)

    # Determine node name from model path
    if isinstance(model, (str, Path)):
        node_name = Path(model).stem + "_node"
    else:
        node_name = "ml_node"

    return Node(
        name=node_name,
        subs=[input],
        pubs=[output],
        tick=tick,
        rate=rate,
    )


# Attach from_model to mlnode for convenience
mlnode.from_model = from_model
