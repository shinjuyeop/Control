#!/usr/bin/env python3
"""Deprecated source-tree entry point for kaiev26_hil.bench.can_sim."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "kaiev26_hil"))
from kaiev26_hil.bench.can_sim import main

if __name__ == "__main__":
    raise SystemExit(main())
