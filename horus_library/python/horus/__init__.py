# Namespace package - allows multiple packages to share the 'horus' namespace
__path__ = __import__('pkgutil').extend_path(__path__, __name__)

# Re-export everything from horus.library for convenience
# This allows: from horus import Pose2D, Image, PointCloud, etc.
try:
    from horus.library import *
    from horus.library import __all__ as _library_all
    __all__ = list(_library_all)
except ImportError:
    # Library not installed yet
    __all__ = []
