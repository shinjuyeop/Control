from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEAMKAI_CAN_LAB = ROOT / "teamkai_can_lab"

if str(TEAMKAI_CAN_LAB) not in sys.path:
    sys.path.insert(0, str(TEAMKAI_CAN_LAB))
