"""pcbroute reads the keepouts inside footprint images (KiCad writes one per copper layer for every bare NPTH hole,
and the footprint's rule areas there): placed, rotated and mirrored with their component, and obeyed by routing and
by --eval. --no-image-keepouts reads only the structure's, as before.

usage: python3 -I tests/test_image_keepouts.py   (PCBROUTE=path/to/pcbroute, default build/pcbroute;
       PCBROUTE_OLD=build-old/pcbroute, a build from before the fix, adds the A/B case)
"""
import json
import os
import re
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTER = os.environ.get("PCBROUTE", os.path.join(HERE, "..", "build", "pcbroute"))
OLD = os.environ.get("PCBROUTE_OLD", os.path.join(HERE, "..", "build-old", "pcbroute"))

HEAD = """(pcb test.dsn
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (path pcb 0  0 0  20000 0  20000 10000  0 10000  0 0))
    (via V)
    (rule (width 200) (clearance 200))
  )
"""

LIB = """  (library
    (image P (pin TH 1 0 0))
    (image M (keepout "" (circle F.Cu 3000)) (keepout "" (circle B.Cu 3000)))
    (image K
      (keepout "" (polygon F.Cu 0  1000 0  2000 0  2000 500  1000 500))
      (via_keepout "" (circle signal 400 0 1500))
    )
    (padstack TH (shape (circle F.Cu 800)) (shape (circle B.Cu 800)) (attach off))
    (padstack V (shape (circle F.Cu 600)) (shape (circle B.Cu 600)) (attach off))
  )
  (network
    (net N1 (pins A1-1 B1-1))
    (class default N1 (circuit (use_via V)) (rule (width 200) (clearance 200)))
  )
)
"""


def dsn(places):
    """places: (image, ref, x, y, side, rotation)"""
    comps = {}
    for img, ref, x, y, side, rot in places:
        comps.setdefault(img, []).append(f"(place {ref} {x} {y} {side} {rot})")
    pl = "".join(f"    (component {img} {' '.join(p)})\n" for img, p in comps.items())
    return HEAD + "  (placement\n" + pl + "  )\n" + LIB


def run(router, text, *args):
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "test.dsn")
        with open(p, "w") as f:
            f.write(text)
        out = {}
        if "--dump-board" in args:
            j = os.path.join(d, "dump.json")
            rest = [a for a in args if a != "--dump-board"]
            subprocess.run([router, "-de", p, *rest, "--dump-board", j], check=True, capture_output=True)
            with open(j) as f:
                out["dump"] = json.load(f)
        else:
            s = os.path.join(d, "test.ses")
            r = subprocess.run([router, "-de", p, "-do", s, "-mt", "1", *args], capture_output=True, text=True)
            out["route"] = r
            ev = subprocess.run([ROUTER, "-de", p, "--eval", s], capture_output=True, text=True)
            out["eval"] = ev.stdout + ev.stderr
        return out


def boxes(dump):
    res = []
    for k in dump["keepouts"]:
        xs = [v for s in k["segs"] for v in (s[0], s[2])]
        ys = [v for s in k["segs"] for v in (s[1], s[3])]
        res.append((k["layer"], k["wires"], k["vias"], round(min(xs)), round(min(ys)), round(max(xs)), round(max(ys)),
                    round(k["r"])))
    return sorted(res)


def violations(ev):
    m = re.search(r"(\d+) violations?", ev)
    return int(m.group(1)) if m else None


class ImageKeepouts(unittest.TestCase):
    def test_placed_rotated_mirrored(self):
        d = run(ROUTER, dsn([("P", "A1", 1000, 1000, "front", 0), ("P", "B1", 19000, 1000, "front", 0),
                             ("K", "K1", 5000, 5000, "front", 0), ("K", "K2", 5000, 5000, "front", 90),
                             ("K", "K3", 5000, 5000, "back", 0)]), "--dump-board")["dump"]
        F, B = 0, 1
        self.assertEqual(boxes(d), sorted([
            # K1: as drawn, on F.Cu (wires and vias); its via keepout at (0, 1500) on every layer
            (F, 1, 1, 6000, 5000, 7000, 5500, 0), (-1, 0, 1, 5000, 6500, 5000, 6500, 200),
            # K2: turned 90 degrees counter-clockwise about the part's origin
            (F, 1, 1, 4500, 6000, 5000, 7000, 0), (-1, 0, 1, 3500, 5000, 3500, 5000, 200),
            # K3: on the back: x mirrored, F.Cu becomes B.Cu
            (B, 1, 1, 3000, 5000, 4000, 5500, 0), (-1, 0, 1, 5000, 6500, 5000, 6500, 200),
        ]))

    def test_switch_off(self):
        d = run(ROUTER, dsn([("P", "A1", 1000, 1000, "front", 0), ("P", "B1", 19000, 1000, "front", 0),
                             ("K", "K1", 5000, 5000, "back", 90)]), "--dump-board", "--no-image-keepouts")["dump"]
        self.assertEqual(d["keepouts"], [])

    def test_route_goes_around_a_hole(self):
        board = dsn([("P", "A1", 2000, 5000, "front", 0), ("P", "B1", 18000, 5000, "front", 0),
                     ("M", "H1", 10000, 5000, "front", 0)])
        r = run(ROUTER, board)
        self.assertEqual(r["route"].returncode, 0, r["route"].stderr)
        self.assertIn("0 unrouted", r["route"].stderr + r["route"].stdout)
        self.assertEqual(violations(r["eval"]), 0, r["eval"])

    @unittest.skipUnless(os.path.exists(OLD), "no pre-fix build (PCBROUTE_OLD)")
    def test_old_build_crosses_the_hole_and_eval_sees_it(self):
        board = dsn([("P", "A1", 2000, 5000, "front", 0), ("P", "B1", 18000, 5000, "front", 0),
                     ("M", "H1", 10000, 5000, "front", 0)])
        r = run(OLD, board)
        self.assertGreater(violations(r["eval"]), 0, r["eval"])


if __name__ == "__main__":
    unittest.main()
