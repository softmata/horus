"""
HORUS Transforms - Image and data preprocessing utilities.

Provides composable transforms for ML preprocessing pipelines.
"""

from typing import Optional, Union, Tuple, List, Callable, Any
import numpy as np

# Dtype mapping for string -> numpy dtype conversion
DTYPE_MAP = {
    "float32": np.float32,
    "float64": np.float64,
    "float16": np.float16,
    "int8": np.int8,
    "int16": np.int16,
    "int32": np.int32,
    "int64": np.int64,
    "uint8": np.uint8,
    "uint16": np.uint16,
    "uint32": np.uint32,
    "uint64": np.uint64,
    "bool": np.bool_,
    "f32": np.float32,
    "f64": np.float64,
    "f16": np.float16,
    "i8": np.int8,
    "i16": np.int16,
    "i32": np.int32,
    "i64": np.int64,
    "u8": np.uint8,
    "u16": np.uint16,
    "u32": np.uint32,
    "u64": np.uint64,
}


class Transform:
    """Base class for transforms."""

    def __call__(self, data: np.ndarray) -> np.ndarray:
        """Apply transform to data."""
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}()"


class Compose(Transform):
    """
    Compose multiple transforms sequentially.

    Example:
        transform = Compose([
            Resize(640, 640),
            Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ToTensor(),
        ])
        output = transform(image)
    """

    def __init__(self, transforms: List[Transform]):
        self.transforms = transforms

    def __call__(self, data: np.ndarray) -> np.ndarray:
        for t in self.transforms:
            data = t(data)
        return data

    def __repr__(self) -> str:
        transforms_str = ", ".join(repr(t) for t in self.transforms)
        return f"Compose([{transforms_str}])"


class ToTensor(Transform):
    """
    Convert data to numpy array with specified dtype.

    Example:
        to_tensor = ToTensor()
        array = to_tensor(data)
    """

    def __init__(self, dtype: str = "float32"):
        self.dtype = dtype

    def __call__(self, data: np.ndarray) -> np.ndarray:
        np_dtype = DTYPE_MAP.get(self.dtype, np.float32)
        return np.asarray(data, dtype=np_dtype)

    def __repr__(self) -> str:
        return f"ToTensor(dtype='{self.dtype}')"


class Resize(Transform):
    """
    Resize image to target size.

    Example:
        resize = Resize(640, 640)
        resized = resize(image)

        # Keep aspect ratio
        resize = Resize(640, 640, keep_aspect=True)
    """

    def __init__(
        self,
        width: int,
        height: int,
        keep_aspect: bool = False,
        interpolation: str = "bilinear",
    ):
        self.width = width
        self.height = height
        self.keep_aspect = keep_aspect
        self.interpolation = interpolation

    def __call__(self, data: np.ndarray) -> np.ndarray:
        arr = data

        # Try using cv2 for resize
        try:
            import cv2

            interpolation_map = {
                "nearest": cv2.INTER_NEAREST,
                "bilinear": cv2.INTER_LINEAR,
                "bicubic": cv2.INTER_CUBIC,
                "area": cv2.INTER_AREA,
                "lanczos": cv2.INTER_LANCZOS4,
            }
            interp = interpolation_map.get(self.interpolation, cv2.INTER_LINEAR)

            if self.keep_aspect:
                h, w = arr.shape[:2]
                scale = min(self.width / w, self.height / h)
                new_w, new_h = int(w * scale), int(h * scale)
                resized = cv2.resize(arr, (new_w, new_h), interpolation=interp)

                # Pad to target size
                result = np.zeros((self.height, self.width) + arr.shape[2:], dtype=arr.dtype)
                y_offset = (self.height - new_h) // 2
                x_offset = (self.width - new_w) // 2
                result[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = resized
            else:
                result = cv2.resize(arr, (self.width, self.height), interpolation=interp)

        except ImportError:
            # Fallback to PIL
            try:
                from PIL import Image

                pil_interp = {
                    "nearest": Image.NEAREST,
                    "bilinear": Image.BILINEAR,
                    "bicubic": Image.BICUBIC,
                    "lanczos": Image.LANCZOS,
                }.get(self.interpolation, Image.BILINEAR)

                if arr.ndim == 3 and arr.shape[2] == 3:
                    img = Image.fromarray(arr.astype(np.uint8))
                else:
                    img = Image.fromarray(arr)

                if self.keep_aspect:
                    img.thumbnail((self.width, self.height), pil_interp)
                    # Pad to target size
                    result = np.zeros((self.height, self.width) + arr.shape[2:], dtype=arr.dtype)
                    y_offset = (self.height - img.height) // 2
                    x_offset = (self.width - img.width) // 2
                    result[y_offset:y_offset + img.height, x_offset:x_offset + img.width] = np.array(img)
                else:
                    img = img.resize((self.width, self.height), pil_interp)
                    result = np.array(img)

            except ImportError:
                # Last resort: simple numpy resize (nearest neighbor)
                h, w = arr.shape[:2]
                y_indices = (np.arange(self.height) * h / self.height).astype(int)
                x_indices = (np.arange(self.width) * w / self.width).astype(int)
                result = arr[y_indices[:, None], x_indices]

        return result

    def __repr__(self) -> str:
        return f"Resize({self.width}, {self.height}, keep_aspect={self.keep_aspect})"


class Normalize(Transform):
    """
    Normalize tensor with mean and std.

    Example:
        # ImageNet normalization
        normalize = Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225]
        )
        normalized = normalize(image)

        # Simple [0, 255] -> [0, 1]
        normalize = Normalize(scale=255.0)
    """

    def __init__(
        self,
        mean: Optional[List[float]] = None,
        std: Optional[List[float]] = None,
        scale: float = 1.0,
    ):
        self.mean = np.array(mean) if mean else None
        self.std = np.array(std) if std else None
        self.scale = scale

    def __call__(self, data: np.ndarray) -> np.ndarray:
        arr = data.astype(np.float32)

        # Scale first
        if self.scale != 1.0:
            arr = arr / self.scale

        # Apply mean/std normalization
        if self.mean is not None:
            arr = arr - self.mean
        if self.std is not None:
            arr = arr / self.std

        return arr

    def __repr__(self) -> str:
        if self.mean is not None:
            return f"Normalize(mean={list(self.mean)}, std={list(self.std)})"
        return f"Normalize(scale={self.scale})"


class CenterCrop(Transform):
    """
    Crop center of image.

    Example:
        crop = CenterCrop(224, 224)
        cropped = crop(image)
    """

    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height

    def __call__(self, data: np.ndarray) -> np.ndarray:
        h, w = data.shape[:2]
        y_start = max(0, (h - self.height) // 2)
        x_start = max(0, (w - self.width) // 2)

        return data[y_start:y_start + self.height, x_start:x_start + self.width]

    def __repr__(self) -> str:
        return f"CenterCrop({self.width}, {self.height})"


class Pad(Transform):
    """
    Pad image to target size.

    Example:
        pad = Pad(640, 640, fill=114)  # YOLO gray padding
        padded = pad(image)
    """

    def __init__(
        self,
        width: int,
        height: int,
        fill: Union[int, Tuple[int, int, int]] = 0,
        mode: str = "constant",
    ):
        self.width = width
        self.height = height
        self.fill = fill
        self.mode = mode

    def __call__(self, data: np.ndarray) -> np.ndarray:
        h, w = data.shape[:2]

        # Calculate padding
        pad_h = max(0, self.height - h)
        pad_w = max(0, self.width - w)

        pad_top = pad_h // 2
        pad_bottom = pad_h - pad_top
        pad_left = pad_w // 2
        pad_right = pad_w - pad_left

        # Pad
        if data.ndim == 3:
            if self.mode == "constant":
                padded = np.pad(
                    data,
                    ((pad_top, pad_bottom), (pad_left, pad_right), (0, 0)),
                    mode="constant",
                    constant_values=self.fill if isinstance(self.fill, int) else 0,
                )
            else:
                padded = np.pad(
                    data,
                    ((pad_top, pad_bottom), (pad_left, pad_right), (0, 0)),
                    mode=self.mode,
                )
        else:
            if self.mode == "constant":
                padded = np.pad(
                    data,
                    ((pad_top, pad_bottom), (pad_left, pad_right)),
                    mode="constant",
                    constant_values=self.fill if isinstance(self.fill, int) else 0,
                )
            else:
                padded = np.pad(
                    data,
                    ((pad_top, pad_bottom), (pad_left, pad_right)),
                    mode=self.mode,
                )

        return padded

    def __repr__(self) -> str:
        return f"Pad({self.width}, {self.height}, fill={self.fill})"


class HWC2CHW(Transform):
    """
    Convert HWC format to CHW format (for PyTorch).

    Example:
        convert = HWC2CHW()
        chw_tensor = convert(hwc_image)
    """

    def __call__(self, data: np.ndarray) -> np.ndarray:
        if data.ndim == 3:
            return data.transpose(2, 0, 1)
        return data

    def __repr__(self) -> str:
        return "HWC2CHW()"


class CHW2HWC(Transform):
    """
    Convert CHW format to HWC format (from PyTorch).

    Example:
        convert = CHW2HWC()
        hwc_tensor = convert(chw_image)
    """

    def __call__(self, data: np.ndarray) -> np.ndarray:
        if data.ndim == 3:
            return data.transpose(1, 2, 0)
        return data

    def __repr__(self) -> str:
        return "CHW2HWC()"


class BGR2RGB(Transform):
    """
    Convert BGR to RGB (OpenCV to standard).

    Example:
        convert = BGR2RGB()
        rgb = convert(bgr_image)
    """

    def __call__(self, data: np.ndarray) -> np.ndarray:
        if data.ndim == 3 and data.shape[2] == 3:
            return data[..., ::-1].copy()
        return data

    def __repr__(self) -> str:
        return "BGR2RGB()"


class RGB2BGR(Transform):
    """
    Convert RGB to BGR (standard to OpenCV).

    Example:
        convert = RGB2BGR()
        bgr = convert(rgb_image)
    """

    def __call__(self, data: np.ndarray) -> np.ndarray:
        if data.ndim == 3 and data.shape[2] == 3:
            return data[..., ::-1].copy()
        return data

    def __repr__(self) -> str:
        return "RGB2BGR()"


class AddBatch(Transform):
    """
    Add batch dimension.

    Example:
        add_batch = AddBatch()
        batched = add_batch(image)  # (H, W, C) -> (1, H, W, C)
    """

    def __call__(self, data: np.ndarray) -> np.ndarray:
        return np.expand_dims(data, axis=0)

    def __repr__(self) -> str:
        return "AddBatch()"


class RemoveBatch(Transform):
    """
    Remove batch dimension.

    Example:
        remove_batch = RemoveBatch()
        unbatched = remove_batch(batched)  # (1, H, W, C) -> (H, W, C)
    """

    def __call__(self, data: np.ndarray) -> np.ndarray:
        if data.ndim > 0 and data.shape[0] == 1:
            return np.squeeze(data, axis=0)
        return data

    def __repr__(self) -> str:
        return "RemoveBatch()"


# ══════════════════════════════════════════════════════════════
# Presets - Common preprocessing pipelines
# ══════════════════════════════════════════════════════════════

def imagenet_preprocess(size: int = 224) -> Compose:
    """
    ImageNet preprocessing pipeline.

    Example:
        preprocess = imagenet_preprocess()
        tensor = preprocess(image)
    """
    return Compose([
        Resize(size, size),
        Normalize(scale=255.0),
        Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        ),
        HWC2CHW(),
        AddBatch(),
    ])


def yolo_preprocess(size: int = 640) -> Compose:
    """
    YOLO preprocessing pipeline.

    Example:
        preprocess = yolo_preprocess(640)
        tensor = preprocess(image)
    """
    return Compose([
        Resize(size, size, keep_aspect=True),
        Normalize(scale=255.0),
        HWC2CHW(),
        AddBatch(),
    ])


def simple_preprocess(width: int, height: int) -> Compose:
    """
    Simple preprocessing: resize and normalize to [0, 1].

    Example:
        preprocess = simple_preprocess(224, 224)
        tensor = preprocess(image)
    """
    return Compose([
        Resize(width, height),
        Normalize(scale=255.0),
        ToTensor(),
    ])
