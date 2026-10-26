"""--plane-mode refill: a DSN plane (a copper pour) no longer takes its whole layer for its net. Other nets route
through it with their clearance, as through a refilled KiCad zone; the pour's net connects through the islands
left, and --eval and the self-check judge it by those islands. The default (reserve) keeps the old behaviour.

usage: python3 -I tests/test_plane_refill.py   (PCBROUTE=path/to/pcbroute, default build/pcbroute)
"""
import os
import re
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTER = os.environ.get("PCBROUTE", os.path.join(HERE, "..", "build", "pcbroute"))

# 20 x 10 mm, two layers. A GND pour covers B.Cu between y = 2 and 8 mm. A keepout wall across F.Cu at y = 6.5-7 mm
# makes N1 (pads at y = 1 and 9 mm) cross the pour on B.Cu. GND's two pads (y = 4 mm, x = 3 and 17 mm) sit in it.
BOARD = """(pcb test.dsn
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal))
    (layer B.Cu (type signal))
    (boundary (path pcb 0  0 0  20000 0  20000 10000  0 10000  0 0))
    (keepout "" (polygon F.Cu 0  0 6500  20000 6500  20000 7000  0 7000))
    (plane GND (polygon B.Cu 0  0 2000  20000 2000  20000 8000  0 8000))
    (via V)
    (rule (width 200) (clearance 200))
  )
  (placement
    (component P (place A1 10000 1000 front 0) (place B1 10000 9000 front 0)
                 (place G1 3000 4000 front 0) (place G2 17000 4000 front 0))
  )
  (library
    (image P (pin TH 1 0 0))
    (padstack TH (shape (circle F.Cu 800)) (shape (circle B.Cu 800)) (attach off))
    (padstack V (shape (circle F.Cu 600)) (shape (circle B.Cu 600)) (attach off))
  )
  (network
    (net N1 (pins A1-1 B1-1))
    (net GND (pins G1-1 G2-1))
    (class default N1 GND (circuit (use_via V)) (rule (width 200) (clearance 200)))
  )
)
"""


def run(*args, board=BOARD):
    with tempfile.TemporaryDirectory() as d:
        p, s = os.path.join(d, "test.dsn"), os.path.join(d, "test.ses")
        with open(p, "w") as f:
            f.write(board)
        r = subprocess.run([ROUTER, "-de", p, "-do", s, "-mt", "1", *args], capture_output=True, text=True)
        ev = {m: subprocess.run([ROUTER, "-de", p, "--eval", s, "--plane-mode", m], capture_output=True, text=True).stdout
              for m in ("reserve", "refill")}
        ses = ""
        if os.path.exists(s):
            with open(s) as f:
                ses = f.read()
        return r, ev, ses


def num(pattern, text):
    m = re.search(pattern, text)
    return int(m.group(1)) if m else None


# Four layers: a keepout wall across F.Cu and B.Cu makes N1 go inside, where In1.Cu is +3V3's pour and In2.Cu GND's
# (the reference plane of B.Cu). Left alone, N1 takes In2.Cu.
BOARD4 = (BOARD.replace("(layer B.Cu (type signal))", "(layer In1.Cu (type signal))\n    (layer In2.Cu (type signal))\n"
                        "    (layer B.Cu (type signal))")
          .replace('(keepout "" (polygon F.Cu 0  0 6500  20000 6500  20000 7000  0 7000))',
                   '(keepout "" (polygon F.Cu 0  0 4500  20000 4500  20000 5500  0 5500))\n'
                   '    (keepout "" (polygon B.Cu 0  0 4500  20000 4500  20000 5500  0 5500))')
          .replace("(plane GND (polygon B.Cu 0  0 2000  20000 2000  20000 8000  0 8000))",
                   "(plane GND (polygon In2.Cu 0  0 0  20000 0  20000 10000  0 10000))\n"
                   "    (plane +3V3 (polygon In1.Cu 0  0 0  20000 0  20000 10000  0 10000))")
          .replace("(net GND (pins G1-1 G2-1))", "(net GND (pins G1-1 G2-1))\n    (net +3V3 (pins))")
          .replace("(circle B.Cu 800)", "(circle In1.Cu 800)) (shape (circle In2.Cu 800)) (shape (circle B.Cu 800)")
          .replace("(circle B.Cu 600)", "(circle In1.Cu 600)) (shape (circle In2.Cu 600)) (shape (circle B.Cu 600)"))


def inner_layers(ses, net):
    blk = re.search(r"\(net " + re.escape(net) + r"\b.*?\n      \)", ses, re.S).group(0)
    return set(re.findall(r"\(path (In\d\.Cu)", blk))


class PlaneRefill(unittest.TestCase):
    def test_gnd_cost_keeps_the_reference_plane(self):
        r, ev, ses = run("--plane-mode", "refill", "--pour-gnd-cost", "10", "-v", board=BOARD4)
        self.assertIn("ground pour cost x10.00 on In2.Cu\n", r.stderr)
        self.assertEqual(num(r"(\d+) unrouted", r.stderr), 0, r.stderr)
        self.assertEqual(inner_layers(ses, "N1"), {"In1.Cu"}, ses)
        self.assertEqual(inner_layers(run("--plane-mode", "refill", board=BOARD4)[2], "N1"), {"In2.Cu"})
        self.assertEqual(num(r"(\d+) open", ev["refill"]), 0, ev["refill"])

    def test_gnd_cost_off_byte_identical(self):
        a = run("--plane-mode", "refill", board=BOARD4)[2]
        b = run("--plane-mode", "refill", "--pour-gnd-cost", "0", board=BOARD4)[2]
        self.assertTrue(a)
        self.assertEqual(a, b)

    def test_reserve_keeps_the_layer(self):
        r, ev, _ = run()
        self.assertEqual(num(r"(\d+) unrouted", r.stderr), 1, r.stderr)  # N1 can't cross the pour

    def test_refill_routes_through_the_pour(self):
        r, ev, ses = run("--plane-mode", "refill")
        self.assertEqual(num(r"(\d+) unrouted", r.stderr), 0, r.stderr)
        self.assertEqual(num(r"(\d+) violations", ev["refill"]), 0, ev["refill"])
        self.assertIn("B.Cu", re.search(r"\(net N1.*?\n      \)", ses, re.S).group(0))
        # N1 cuts the pour in two: the repair pass joins GND's islands with a track of its own (F.Cu, above the wall),
        # and the self-check agrees with --eval
        self.assertIn("refilled pours: 1 nets cut into islands", r.stderr)
        self.assertEqual(num(r"(\d+) open", ev["refill"]), 0, ev["refill"])
        self.assertEqual(num(r"refilled pours: (\d+) of", r.stderr), 0, r.stderr)
        self.assertIn("(net GND", ses)

    def test_refill_eval_sees_the_cut(self):
        # N1 routed straight across the pour, GND left alone: reserve --eval counts GND joined by the pour, refill
        # --eval sees two islands
        with tempfile.TemporaryDirectory() as d:
            p, s = os.path.join(d, "test.dsn"), os.path.join(d, "test.ses")
            with open(p, "w") as f:
                f.write(BOARD)
            with open(s, "w") as f:
                f.write("(session test (routes (resolution um 10) (network_out (net N1"
                        " (wire (path B.Cu 2000 100000 10000 100000 90000))))))")
            ev = {m: subprocess.run([ROUTER, "-de", p, "--eval", s, "--plane-mode", m], capture_output=True, text=True).stdout
                  for m in ("reserve", "refill")}
        self.assertEqual(num(r"(\d+) open", ev["reserve"]), 0, ev["reserve"])
        self.assertEqual(num(r"(\d+) open", ev["refill"]), 1, ev["refill"])

    def test_no_planes_byte_identical(self):
        board = re.sub(r"    \(plane .*\n", "", BOARD)
        a = run(board=board)[2]
        b = run("--plane-mode", "refill", board=board)[2]
        self.assertTrue(a)
        self.assertEqual(a, b)


def evaluate(board, wires=""):
    """GND's open connections by --eval --plane-mode refill of a session with `wires` (in N1) on `board`, and the
    output (stdout + stderr)."""
    with tempfile.TemporaryDirectory() as d:
        p, s = os.path.join(d, "test.dsn"), os.path.join(d, "test.ses")
        with open(p, "w") as f:
            f.write(board)
        with open(s, "w") as f:
            f.write(f"(session test (routes (resolution um 10) (network_out (net N1 {wires}))))")
        r = subprocess.run([ROUTER, "-de", p, "--eval", s, "--plane-mode", "refill", "-v"], capture_output=True, text=True)
        return num(r"open: GND, (\d+)", r.stderr) or 0, r.stdout + r.stderr


def with_planes(planes, zones=""):
    """BOARD with its GND plane replaced by `planes`, a VCC net with a pad at (17, 9) mm (above GND's pour: a pour
    keeps only the islands its net's copper touches), and `zones` as its (pcbroute_zones)."""
    board = re.sub(r"    \(plane .*\n", planes + (f"    (pcbroute_zones {zones})\n" if zones else ""), BOARD)
    board = board.replace("(place G2 17000 4000 front 0)", "(place G2 17000 4000 front 0) (place V1 17000 9000 front 0)")
    return board.replace("(net GND (pins G1-1 G2-1))", "(net GND (pins G1-1 G2-1))\n    (net VCC (pins V1-1))")


GND = "    (plane GND (polygon B.Cu 0  0 2000  20000 2000  20000 8000  0 8000))\n"
VCC = "    (plane VCC (polygon B.Cu 0  0 0  20000 0  20000 10000  0 10000))\n"  # the whole layer, larger than GND's


class ZoneRules(unittest.TestCase):
    """make_dsn.py's (pcbroute_zones): each plane's KiCad zone priority, clearance, minimum width and fillet."""

    def test_priority_decides_the_overlap(self):
        # without rules the smaller pour (GND) wins; with VCC's priority higher, GND is knocked out and its pads
        # are left apart
        self.assertEqual(evaluate(with_planes(GND + VCC))[0], 0)
        self.assertEqual(evaluate(with_planes(GND + VCC, "(zone GND B.Cu (priority 0)) (zone VCC B.Cu (priority 2))"))[0], 1)

    def test_a_winner_knocks_out_only_its_fill(self):
        # an N1 track across B.Cu at y = 8.3 mm cuts VCC's pour below it off VCC's pad: KiCad removes that island, and
        # GND, of lower priority, fills there (VCC's outline over GND would leave GND's pads apart)
        cut = "(wire (path B.Cu 2000 0 83000 200000 83000))"
        self.assertEqual(evaluate(with_planes(GND + VCC, "(zone GND B.Cu (priority 0)) (zone VCC B.Cu (priority 2))"), cut)[0], 0)

    def test_zone_clearance_cuts_a_neck(self):
        # an N1 stub up B.Cu from the bottom edge to y = 5.5 mm leaves GND a 2.4 mm neck above it: the class
        # clearance keeps it, a 2.3 mm zone clearance doesn't (100 um left, under the 250 um minimum width)
        stub = "(wire (path B.Cu 2000 100000 10000 100000 55000))"
        self.assertEqual(evaluate(with_planes(GND, "(zone GND B.Cu (priority 0) (clearance 1200) (min_width 250))"), stub)[0], 0)
        self.assertEqual(evaluate(with_planes(GND, "(zone GND B.Cu (priority 0) (clearance 2300) (min_width 250))"), stub)[0], 1)

    def test_rules_that_dont_match_are_ignored(self):
        gnd, out = evaluate(with_planes(GND + VCC, "(zone VCC B.Cu (priority 0)) (zone GND B.Cu (priority 2))"))
        self.assertIn("doesn't match the planes", out)
        self.assertEqual(gnd, 0, out)

    def test_touching_pours_of_a_net_join(self):
        # two GND pours that overlap, each under one GND pad and no copper of GND's in both: KiCad joins their fills
        halves = ("    (plane GND (polygon B.Cu 0  0 2000  10200 2000  10200 8000  0 8000))\n"
                  "    (plane GND (polygon B.Cu 0  9800 2000  20000 2000  20000 8000  9800 8000))\n")
        self.assertEqual(evaluate(with_planes(halves))[0], 0)


if __name__ == "__main__":
    unittest.main()
