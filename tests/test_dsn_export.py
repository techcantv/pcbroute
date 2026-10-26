"""tools/make_dsn.py's export, and pcbroute reading it.

- A pour on outer and inner layers (one zone on F.Cu In1.Cu In2.Cu B.Cu, as on the PCIe debifurcator) keeps its
  inner layers as planes, and its net keeps its pins; a pour on the outer layers only is dropped and its net is done.
- The board's copper-to-edge clearance goes in (pcbroute_rules (edge_clearance UM)): the .kicad_pro's
  min_copper_edge_clearance (KiCad 6+), an unconditional .kicad_dru edge_clearance rule over it, a KiCad 5.99
  board's setup (edge_clearance), a KiCad 5 board's Default netclass clearance; nothing for a KiCad 6+ board without
  its project. pcbroute takes it as the --edge default, and --edge still overrides it.

usage: python3 -I tests/test_dsn_export.py   (PCBROUTE=path/to/pcbroute, default build/pcbroute; the make_dsn
tests need KiCad's python and are skipped without it)
"""
import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTER = os.environ.get("PCBROUTE", os.path.join(HERE, "..", "build", "pcbroute"))
MAKE_DSN = os.path.join(HERE, "..", "tools", "make_dsn.py")
try:
    import pcbnew  # noqa: F401
    HAVE_KICAD = True
except ImportError:
    HAVE_KICAD = False

# 30 x 20 mm, 4 layers, two 2-pin THT parts: GND (pads A1, B1) and SIG (A2, B2). GroundFill is one zone on all four
# copper layers; TopFill (net SIG) is on F.Cu only.
PCB = """(kicad_pcb (version 20240108) (generator "pcbnew") (generator_version "8.0")
  (general (thickness 1.6) (legacy_teardrops no))
  (paper "A4")
  (layers
    (0 "F.Cu" signal) (1 "In1.Cu" power) (2 "In2.Cu" power) (31 "B.Cu" signal)
    (44 "Edge.Cuts" user) (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen")
  )
  (setup (pad_to_mask_clearance 0))
  (net 0 "") (net 1 "GND") (net 2 "SIG")
  (footprint "T:P2" (layer "F.Cu") (at 5 10)
    (property "Reference" "A" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu") (net 1 "GND"))
    (pad "2" thru_hole circle (at 0 3) (size 1.6 1.6) (drill 0.8) (layers "*.Cu") (net 2 "SIG"))
  )
  (footprint "T:P2" (layer "F.Cu") (at 25 10)
    (property "Reference" "B" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers "*.Cu") (net 1 "GND"))
    (pad "2" thru_hole circle (at 0 3) (size 1.6 1.6) (drill 0.8) (layers "*.Cu") (net 2 "SIG"))
  )
  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (zone (net 1) (net_name "GND") (layers "F.Cu" "In1.Cu" "In2.Cu" "B.Cu") (name "GroundFill") (hatch edge 0.5)
    (connect_pads (clearance 0.3)) (min_thickness 0.25) (filled_areas_thickness no)
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 1 1) (xy 29 1) (xy 29 19) (xy 1 19)))
  )
  (zone (net 2) (net_name "SIG") (layer "F.Cu") (name "TopFill") (hatch edge 0.5)
    (connect_pads (clearance 0.3)) (min_thickness 0.25) (filled_areas_thickness no)
    (fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
    (polygon (pts (xy 10 12) (xy 20 12) (xy 20 15) (xy 10 15)))
  )
)
"""

# The same board in KiCad 5's format: the rules are in the board, the Default netclass clearance 0.3 mm.
PCB5 = """(kicad_pcb (version 20171130) (host pcbnew 5.1.9)
  (general (thickness 1.6))
  (page A4)
  (layers (0 F.Cu signal) (31 B.Cu signal) (44 Edge.Cuts user))
  (setup (last_trace_width 0.25) (trace_clearance 0.3) (zone_clearance 0.5) (trace_min 0.2) (via_size 0.8)
    (via_drill 0.4) (via_min_size 0.4) (via_min_drill 0.3) (edge_width 0.05) (aux_axis_origin 0 0))
  (net 0 "") (net 1 SIG)
  (net_class Default "This is the default net class."
    (clearance 0.3) (trace_width 0.25) (via_dia 0.8) (via_drill 0.4) (uvia_dia 0.3) (uvia_drill 0.1)
    (add_net SIG)
  )
  (module T:P1 (layer F.Cu) (tedit 0) (tstamp 1) (at 5 10)
    (fp_text reference A (at 0 -2) (layer F.SilkS) (effects (font (size 1 1) (thickness 0.15))))
    (pad 1 thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers *.Cu) (net 1 SIG))
  )
  (module T:P1 (layer F.Cu) (tedit 0) (tstamp 2) (at 25 10)
    (fp_text reference B (at 0 -2) (layer F.SilkS) (effects (font (size 1 1) (thickness 0.15))))
    (pad 1 thru_hole circle (at 0 0) (size 1.6 1.6) (drill 0.8) (layers *.Cu) (net 1 SIG))
  )
  (gr_line (start 0 0) (end 30 0) (layer Edge.Cuts) (width 0.05))
  (gr_line (start 30 0) (end 30 20) (layer Edge.Cuts) (width 0.05))
  (gr_line (start 30 20) (end 0 20) (layer Edge.Cuts) (width 0.05))
  (gr_line (start 0 20) (end 0 0) (layer Edge.Cuts) (width 0.05))
)
"""

# KiCad 5.99 (2019-2020 nightlies): quoted names, the setup's (defaults (edge_clearance)).
PCB599 = (PCB5.replace("(version 20171130) (host pcbnew 5.1.9)", '(version 20200119) (host pcbnew "5.99")')
          .replace("(aux_axis_origin 0 0))", "(aux_axis_origin 0 0) (defaults (edge_clearance 0.15)))")
          .replace("(0 F.Cu signal) (31 B.Cu signal) (44 Edge.Cuts user)", '(0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user)')
          .replace("(layer F.Cu)", '(layer "F.Cu")').replace("(layer F.SilkS)", '(layer "F.SilkS")')
          .replace("(layer Edge.Cuts)", '(layer "Edge.Cuts")').replace("(layers *.Cu)", '(layers "*.Cu")')
          .replace("(net 1 SIG)", '(net 1 "SIG")').replace("(add_net SIG)", '(add_net "SIG")')
          .replace("net_class Default", 'net_class "Default"').replace("(module T:P1", '(module "T:P1"'))


def project(edge_mm):
    return json.dumps({"board": {"design_settings": {"rules": {"min_copper_edge_clearance": edge_mm}}},
                       "meta": {"filename": "t.kicad_pro", "version": 1}})


def export(pcb, pro=None, dru=None):
    with tempfile.TemporaryDirectory() as d:
        for ext, text in (("kicad_pcb", pcb), ("kicad_pro", pro), ("kicad_dru", dru)):
            if text is not None:
                with open(os.path.join(d, "t." + ext), "w") as f:
                    f.write(text)
        r = subprocess.run(["python3", "-I", MAKE_DSN, os.path.join(d, "t.kicad_pcb"), os.path.join(d, "t.dsn")],
                           capture_output=True, text=True)
        if r.returncode:
            raise AssertionError(r.stderr)
        with open(os.path.join(d, "t.dsn")) as f:
            return f.read()


def edge(dsn):
    m = re.search(r"\(pcbroute_rules \(edge_clearance (\d+)\)\)", dsn)
    return int(m.group(1)) if m else None


@unittest.skipUnless(HAVE_KICAD, "needs KiCad's python (pcbnew)")
class Export(unittest.TestCase):
    def test_outer_and_inner_pour_keeps_inner_planes(self):
        dsn = export(PCB, project(0.25))
        planes = re.findall(r"\(plane (\S+) \(\w+ (\S+)", dsn)
        self.assertEqual(sorted(planes), [("GND", "In1.Cu"), ("GND", "In2.Cu")])
        self.assertRegex(dsn, r"\(pcbroute_zones\s+\(zone GND In1\.Cu .*\)\s+\(zone GND In2\.Cu .*\)\s+\)")
        nets = dict(re.findall(r"\(net (\S+)\s*\(pins([^)]*)\)", dsn))
        self.assertEqual(nets["GND"].split(), ["A-1", "B-1"])  # routed to its planes, not done
        self.assertEqual(nets["SIG"].split(), [])  # F.Cu-only pour: the net counts as done

    def test_edge_clearance_from_project(self):
        self.assertEqual(edge(export(PCB, project(0.25))), 250)
        self.assertEqual(edge(export(PCB, project(0))), 0)

    def test_edge_clearance_dru_rule(self):
        dru = ('(version 1)\n(rule "edge" (constraint edge_clearance (min 0.4mm)))\n'
               '(rule "pins" (constraint edge_clearance (min 0.1mm)) (condition "A.Type == \'Pad\'"))\n')
        self.assertEqual(edge(export(PCB, project(0.25), dru)), 400)  # the conditional rule is not the board's

    def test_no_project_writes_nothing(self):
        self.assertIsNone(edge(export(PCB)))

    def test_kicad5_default_netclass(self):
        dsn = export(PCB5)
        self.assertEqual(edge(dsn), 300)
        self.assertEqual(edge(export(PCB599.replace(" (defaults (edge_clearance 0.15))", ""))), 300)  # quoted "Default"

    def test_kicad599_setup(self):
        self.assertEqual(edge(export(PCB599)), 150)


# 20 x 6 mm, one layer. N1's pads sit 0.9 mm from the top edge; a keepout leaves only a lane along the edge between
# them: its far side 0.45 mm from the edge, a 0.2 mm track fits there at an edge clearance of 0.15 mm, not 0.5 mm.
DSN = """(pcb test.dsn
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal))
    (boundary (path pcb 0  0 0  20000 0  20000 6000  0 6000  0 0))
    (keepout "" (polygon F.Cu 0  5000 0  15000 0  15000 5550  5000 5550))
    (via V)
    (rule (width 200) (clearance 100))
    EDGE
  )
  (placement
    (component P (place A1 2000 5100 front 0) (place B1 18000 5100 front 0))
  )
  (library
    (image P (pin S 1 0 0))
    (padstack S (shape (rect F.Cu -300 -300 300 300)) (attach off))
    (padstack V (shape (circle F.Cu 600)) (attach off))
  )
  (network
    (net N1 (pins A1-1 B1-1))
    (class default N1 (circuit (use_via V)) (rule (width 200) (clearance 100)))
  )
)
"""


def route(edge_scope, *args):
    with tempfile.TemporaryDirectory() as d:
        p, s = os.path.join(d, "test.dsn"), os.path.join(d, "test.ses")
        with open(p, "w") as f:
            f.write(DSN.replace("EDGE", edge_scope))
        subprocess.run([ROUTER, "-de", p, "-do", s, "-mt", "1", "-q", *args], capture_output=True, text=True)
        ev = subprocess.run([ROUTER, "-de", p, "--eval", s], capture_output=True, text=True).stdout
        m = re.search(r"(\d+) open", ev)
        return int(m.group(1)) if m else None, ev


@unittest.skipUnless(os.path.exists(ROUTER), "no pcbroute build")
class EdgeDefault(unittest.TestCase):
    def test_board_rule_is_the_default(self):
        self.assertEqual(route("")[0], 1)  # pcbroute's 0.5 mm: the lane is closed
        self.assertEqual(route("(pcbroute_rules (edge_clearance 150))")[0], 0)

    def test_flag_overrides(self):
        self.assertEqual(route("(pcbroute_rules (edge_clearance 150))", "--edge", "500")[0], 1)
        self.assertEqual(route("", "--edge", "150")[0], 0)


if __name__ == "__main__":
    unittest.main()
