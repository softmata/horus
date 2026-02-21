"""
HORUS Model Registry - Lightweight model catalog.

Manages a local registry of ML models at ~/.horus/models.yaml,
tracking model metadata, versions, and file locations.

Example:
    from horus.ai import ModelRegistry, ModelEntry

    registry = ModelRegistry()

    # Register a model
    registry.register(ModelEntry(
        name="yolov8n",
        version="1.0.0",
        format="onnx",
        path="/models/yolov8n.onnx",
    ))

    # Look it up
    entry = registry.get("yolov8n")
    model = Model.load(entry.path)
"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]


def _default_registry_path() -> Path:
    return Path.home() / ".horus" / "models.yaml"


@dataclass
class ModelEntry:
    """A single model in the registry."""

    name: str
    version: str
    format: str
    path: str
    hash: Optional[str] = None
    deprecated: bool = False
    metadata: Dict[str, str] = field(default_factory=dict)

    def compute_hash(self) -> str:
        """Compute SHA-256 hash of the model file."""
        p = Path(self.path)
        if not p.exists():
            raise FileNotFoundError(f"Model file not found: {self.path}")
        h = hashlib.sha256()
        with open(p, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
        self.hash = h.hexdigest()
        return self.hash


class ModelRegistry:
    """
    Local model catalog backed by a YAML file.

    Default location: ~/.horus/models.yaml
    """

    def __init__(self, path: Optional[str] = None) -> None:
        self._path = Path(path) if path else _default_registry_path()
        self._entries: Dict[str, ModelEntry] = {}
        if self._path.exists():
            self._load()

    # ── Public API ────────────────────────────────────────────────

    def register(self, entry: ModelEntry) -> None:
        """Add or update a model entry."""
        self._entries[entry.name] = entry
        self._save()

    def get(self, name: str) -> Optional[ModelEntry]:
        """Look up a model by name. Returns None if not found."""
        return self._entries.get(name)

    def list(self, include_deprecated: bool = False) -> List[ModelEntry]:
        """List all registered models."""
        entries = list(self._entries.values())
        if not include_deprecated:
            entries = [e for e in entries if not e.deprecated]
        return entries

    def deprecate(self, name: str) -> None:
        """Mark a model as deprecated."""
        entry = self._entries.get(name)
        if entry is None:
            raise KeyError(f"Model not found: {name}")
        entry.deprecated = True
        self._save()

    def remove(self, name: str) -> None:
        """Remove a model from the registry (does not delete the file)."""
        if name not in self._entries:
            raise KeyError(f"Model not found: {name}")
        del self._entries[name]
        self._save()

    # ── Persistence ───────────────────────────────────────────────

    def _load(self) -> None:
        if yaml is None:
            raise ImportError("pyyaml is required: pip install pyyaml")
        with open(self._path) as f:
            data = yaml.safe_load(f) or {}
        for name, raw in data.items():
            self._entries[name] = ModelEntry(**raw)

    def _save(self) -> None:
        if yaml is None:
            raise ImportError("pyyaml is required: pip install pyyaml")
        self._path.parent.mkdir(parents=True, exist_ok=True)
        data = {name: asdict(entry) for name, entry in self._entries.items()}
        with open(self._path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    def __repr__(self) -> str:
        count = len(self._entries)
        return f"ModelRegistry({count} models, path='{self._path}')"
