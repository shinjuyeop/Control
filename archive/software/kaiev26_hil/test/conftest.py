from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

SHARED_ROOT = PACKAGE_ROOT.parent / "vehicle_console"
sys.path.insert(0, str(SHARED_ROOT))
sys.path.insert(0, str(SHARED_ROOT / "test"))
