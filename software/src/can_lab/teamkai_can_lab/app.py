"""Compatibility alias for the migrated kaiev26_hil bench runtime module."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "kaiev26_hil"))
from kaiev26_hil.bench import app as _implementation
sys.modules[__name__] = _implementation
