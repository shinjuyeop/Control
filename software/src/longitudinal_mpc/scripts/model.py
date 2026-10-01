from dataclasses import dataclass
from pathlib import Path
import numpy as np
import yaml
import os
import tempfile


@dataclass
class Model:
    dt: float
    A: np.ndarray
    B: np.ndarray
    c: np.ndarray
    speed_max_mps: float

    @classmethod
    def load(cls, path):
        data = yaml.safe_load(Path(path).read_text())
        if (data['input'] != '/vehicle/drive_current_command.phase_current_a' or
                data['states'] != ['speed_mps', 'phase_current_a']):
            raise ValueError('Model input/state coordinates must match the PC current interface')
        m = cls(float(data['dt']), np.asarray(data['A'],float), np.asarray(data['B'],float),
                np.asarray(data['c'],float), float(data['speed_max_mps']))
        if (m.A.shape!=(2,2) or m.B.shape!=(2,) or m.c.shape!=(2,) or
                not all(np.isfinite(a).all() for a in (m.A,m.B,m.c)) or
                not np.isfinite([m.dt,m.speed_max_mps]).all() or m.speed_max_mps<=0 or abs(m.dt-.05)>1e-9):
            raise ValueError('Expected finite two-state dynamics sampled at 20 Hz')
        return m

    def save(self, path, **details):
        document = yaml.safe_dump(dict(dt=self.dt, states=['speed_mps', 'phase_current_a'],
            input='/vehicle/drive_current_command.phase_current_a', A=self.A.tolist(), B=self.B.tolist(),
            c=self.c.tolist(), speed_max_mps=self.speed_max_mps, **details), sort_keys=False)
        path = Path(path)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
                temporary = stream.name
                stream.write(document)
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
