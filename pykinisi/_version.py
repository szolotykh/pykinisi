# Filename: _version.py
# Description: Single source for package identity and INIT SDK version fields.
"""Package identity; protocol compatibility is versioned separately in the schema."""

__version__ = "2.1.0"
VERSION = tuple(int(part) for part in __version__.split("."))
