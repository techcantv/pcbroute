"""The GPU build hands searches to the GPU. From 2026-10-19 to 2026-10-24 it never did: the gate tested `!sx` for
"not in shove mode", but inside the search sx is the grid step, so the gate was always closed and build-gpu routed
exactly like the CPU build. With --gpu-pops 1 every search past its first pop must go to the GPU.

usage: python3 -I tests/test_gpu_gate.py   (PCBROUTE_GPU=path/to/pcbroute, default build-gpu/pcbroute; skipped
without that build or a CUDA device)
"""
import os
import re
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTER = os.environ.get("PCBROUTE_GPU", os.path.join(HERE, "..", "build-gpu", "pcbroute"))

# 20 x 10 mm, two layers, one net from one end to the other
BOARD = """(pcb test.dsn
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (path pcb 0  0 0  20000 0  20000 10000  0 10000  0 0))
    (via V)
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P (place A1 2000 2000 front 0) (place B1 18000 8000 front 0))
  )
  (library
    (image P (pin SMD 1 0 0))
    (padstack SMD (shape (rect F.Cu -300 -300 300 300)) (attach off))
    (padstack V (shape (circle F.Cu 600)) (shape (circle B.Cu 600)) (attach off))
  )
  (network
    (net N1 (pins A1-1 B1-1))
    (class default N1 (circuit (use_via V)) (rule (width 200) (clearance 200)))
  )
)
"""


def has_gpu():
    try:
        return subprocess.run(["nvidia-smi", "-L"], capture_output=True, timeout=20).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


@unittest.skipUnless(os.path.exists(ROUTER) and has_gpu(), "needs build-gpu/pcbroute and a CUDA device")
class GpuGate(unittest.TestCase):
    def route(self, *args):
        with tempfile.TemporaryDirectory() as d:
            p, s = os.path.join(d, "t.dsn"), os.path.join(d, "t.ses")
            with open(p, "w") as f:
                f.write(BOARD)
            r = subprocess.run([ROUTER, "-de", p, "-do", s, "-mt", "1", "-v", *args], capture_output=True, text=True)
            self.assertIn("0 unrouted", r.stderr, r.stderr)
            return r.stderr

    def test_searches_go_to_the_gpu(self):
        m = re.search(r"(\d+) searches finished on the GPU", self.route("--gpu-pops", "1"))
        self.assertIsNotNone(m, "no search reached the GPU")
        self.assertGreater(int(m.group(1)), 0)

    def test_gpu_pops_0_keeps_it_off(self):
        self.assertNotIn("on the GPU", self.route("--gpu-pops", "0"))


if __name__ == "__main__":
    unittest.main()
