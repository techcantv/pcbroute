"""Strip the tracks and vias of a KiCad board and export the routing problem as a Specctra DSN (KiCad's python).

usage: python3 -I make_dsn.py IN.kicad_pcb OUT.dsn [--drawn-widths]

Like gen_pcb.py's passes: pours on the outer layers are dropped (pcbroute, like Freerouting, takes a plane as its
net's layer), and the nets they connected lose their pin lists, so they count as done (in the original they are
connected by those pours). Pours on inner layers stay: those nets reach them through vias. A pour on outer and inner
layers (one zone on F.Cu In1.Cu In2.Cu B.Cu) keeps its inner layers, and its net keeps its pins.

The board's copper-to-edge clearance goes in a (pcbroute_rules (edge_clearance UM)) scope at the end of (structure),
which pcbroute takes as its --edge default: KiCad 6+'s min_copper_edge_clearance (from the .kicad_pro), or the last
unconditional edge_clearance rule of the .kicad_dru; a KiCad 5.99 board's (edge_clearance) in its setup; a KiCad 5
board's Default netclass clearance (KiCad 5's DRC held copper off the edge by the copper's clearance). Nothing for a
KiCad 6+ board without its project.

The DSN's (plane) has only a net and an outline: KiCad's export drops each zone's priority, its own clearance, its
minimum width and its corner fillet, which decide where a refilled pour reaches (pcbroute --plane-mode refill). They
go in a (pcbroute_zones (zone NET LAYER (priority N) (clearance UM) (min_width UM) [(fillet UM) | (chamfer UM)]) ...)
scope after the planes in (structure), one (zone) per (plane), in the same order. Freerouting skips scopes it doesn't
know.

--drawn-widths: a net the designer drew narrower than its netclass's width (a custom width from the board's track
width list) gets that width: the narrowest track drawn on it, never below the board's minimum track width (none
without the project). Its clearance and via stay the netclass's. KiCad's DRC enforces the clearances and the minimum
width, not the netclass width, which is only the default for new tracks (seen: a 0.6 mm default class on a board
drawn mostly at 0.3 mm).
"""
import os
import re
import sys

import pcbnew

drawn_widths = "--drawn-widths" in sys.argv[3:]
b = pcbnew.LoadBoard(sys.argv[1])
version = int(re.search(r"\(version (\d+)", open(sys.argv[1]).read(200)).group(1))
no_project = version >= 20211014 and not os.path.exists(os.path.splitext(sys.argv[1])[0] + ".kicad_pro")  # KiCad 6+
if no_project:
    print("make_dsn: no .kicad_pro beside the board: KiCad 6+ keeps the netclasses there, so the DSN gets KiCad's "
          "default netclass (0.2 / 0.2 mm), not the designer's rules", file=sys.stderr)


def hole_keepout_radii():
    """Each pad KiCad may write as an image keepout: board position (DSN um, y up) -> [(drill, radius its keepout
    circle needs)], um.

    KiCad writes a bare hole (an NPTH pad, an unnumbered PTH, a pad on no copper layer) as a keepout circle: the drill
    plus 0.25 mm. pcbroute keeps no clearance from a keepout, so the circle must hold everything KiCad's DRC asks of
    the hole: the board's hole clearance or an unconditional / NPTH `hole_clearance` rule of the .kicad_dru, from the
    hole's edge, and the pad's own clearance (its override, or the footprint's), from the pad's copper if it has
    any. A pad without a drill (a marker on a non-copper layer, written as a 0.5 mm circle) needs nothing."""
    hole_clr = b.GetDesignSettings().m_HoleClearance
    dru = os.path.splitext(sys.argv[1])[0] + ".kicad_dru"
    if os.path.exists(dru):
        # the rule's condition only when it names NPTH (or there is none): other hole rules are for plated holes
        for rule in re.split(r"\(rule\b", open(dru).read())[1:]:
            m = re.search(r"\(constraint\s+hole_clearance\s+\(min\s+([\d.]+)\s*mm\)", rule)
            cond = re.search(r"\(condition\s+\"([^\"]*)\"", rule)
            if m and (not cond or "NPTH" in cond.group(1)):
                hole_clr = max(hole_clr, round(float(m.group(1)) * 1e6))
    radii = {}
    for fp in b.GetFootprints():
        for p in fp.Pads():
            drill = max(p.GetDrillSize().x, p.GetDrillSize().y)
            if not drill and p.IsOnCopperLayer():
                continue
            copper = max(drill, p.GetSize(pcbnew.F_Cu).x, p.GetSize(pcbnew.F_Cu).y) if p.IsOnCopperLayer() else drill
            own = p.GetOwnClearance(pcbnew.F_Cu)  # the DRC engine's: override, footprint's, else the hole clearance
            r = max(drill / 2 + hole_clr, copper / 2 + own) / 1000 if drill else 0
            pos = p.GetPosition()
            radii.setdefault((pos.x / 1000, -pos.y / 1000), []).append((drill / 1000, r))
    return radii


def edge_clearance():
    """The board's copper-to-edge clearance, um (None: unknown, pcbroute keeps its default)."""
    src = open(sys.argv[1]).read()
    setup = re.search(r"\(setup\b.*?\(edge_clearance ([\d.]+)\)", src, re.S)
    if setup:  # KiCad 5.99 (2019-2020) boards keep it in the setup
        return round(float(setup.group(1)) * 1000)
    if version < 20211014 and not os.path.exists(os.path.splitext(sys.argv[1])[0] + ".kicad_pro"):
        # KiCad 5: no edge setting, its DRC kept copper off the edge by the copper's clearance (the Default class's)
        m = re.search(r'\(net_class "?Default"?\s[^()]*\(clearance ([\d.]+)\)', src)
        return round(float(m.group(1)) * 1000) if m else None
    if no_project:
        return None
    clr = b.GetDesignSettings().m_CopperEdgeClearance / 1000
    dru = os.path.splitext(sys.argv[1])[0] + ".kicad_dru"
    if os.path.exists(dru):
        # KiCad's custom rules override the board setup, a later rule an earlier one; a conditional rule only some copper
        for rule in re.split(r"\(rule\b", open(dru).read())[1:]:
            m = re.search(r"\(constraint\s+edge_clearance\s+\(min\s+([\d.]+)\s*mm\)", rule)
            if m and not re.search(r"\(condition\b", rule):
                clr = float(m.group(1)) * 1000
    return round(clr)


edge_um = edge_clearance()
hole_radii = hole_keepout_radii()  # before anything is deleted from the board
narrowest = {}  # net name -> narrowest drawn track, um
if drawn_widths:
    # the board's minimum track width (unknown without the project: KiCad's default would be a guess)
    wmin = 0 if no_project else b.GetDesignSettings().m_TrackMinWidth / 1000
    for t in b.GetTracks():
        if t.GetClass() in ("PCB_TRACK", "PCB_ARC") and t.GetNetname():
            w = max(t.GetWidth() / 1000, wmin)
            narrowest[t.GetNetname()] = min(narrowest.get(t.GetNetname(), w), w)
for t in list(b.GetTracks()):
    b.Delete(t)
# A rule area that forbids neither tracks nor vias (only pours, pads or footprints: OtterCam's BGA, tiny-engineer) is
# no obstacle to routing, but KiCad's Specctra export writes every rule area that isn't a wire or via keepout as a
# full (keepout): drop those first.
def harmless(z):
    return z.GetIsRuleArea() and not z.GetDoNotAllowTracks() and not z.GetDoNotAllowVias()
dropped = 0
for z in list(b.Zones()):
    if harmless(z):
        b.Delete(z); dropped += 1
for fp in b.GetFootprints():
    for z in list(fp.Zones()):
        if harmless(z):
            fp.Delete(z); dropped += 1
if dropped:
    print(f"make_dsn: {dropped} rule area(s) that allow tracks and vias left out (not keepouts)", file=sys.stderr)
outer = {pcbnew.F_Cu, pcbnew.B_Cu}
inner_nets, pour_nets = set(), set()
for z in list(b.Zones()):
    if z.GetIsRuleArea():
        continue
    layers = set(z.GetLayerSet().CuStack())
    name = z.GetNetname()
    if layers - outer:
        inner_nets.add(name)
    if layers & outer:
        pour_nets.add(name)
        if layers - outer:
            # a pour on outer and inner layers keeps its inner layers (they are planes, as an inner-only pour)
            keep = pcbnew.LSET()
            for l in layers - outer:
                keep.AddLayer(l)
            z.SetLayerSet(keep)
        else:
            b.Delete(z)
done = {n for n in pour_nets - inner_nets if n}
# the zones the export writes as planes, in its order (zones in board order, each zone's copper layers top down)
zone_rules = []
for z in b.Zones():
    if z.GetIsRuleArea():
        continue
    clr = z.GetLocalClearance()
    props = [("priority", z.GetAssignedPriority()), ("clearance", round((clr or 0) / 1000)),
             ("min_width", round(z.GetMinThickness() / 1000))]
    corner = {pcbnew.ZONE_SETTINGS.CORNER_SMOOTHING_FILLET: "fillet", pcbnew.ZONE_SETTINGS.CORNER_SMOOTHING_CHAMFER: "chamfer"}
    if z.GetCornerSmoothingType() in corner and z.GetCornerRadius():
        props.append((corner[z.GetCornerSmoothingType()], round(z.GetCornerRadius() / 1000)))
    for l in z.GetLayerSet().CuStack():
        zone_rules.append((z.GetNetname(), b.GetLayerName(l), props))
if not pcbnew.ExportSpecctraDSN(b, sys.argv[2]):
    sys.exit("DSN export failed")


def span(text, i):
    depth, q = 0, False
    for k in range(i, len(text)):
        c = text[k]
        if c == '"':
            q = not q
        elif not q and c == "(":
            depth += 1
        elif not q and c == ")":
            depth -= 1
            if depth == 0:
                return k + 1
    raise ValueError("unbalanced")


text = open(sys.argv[2]).read()
out, k = [], 0
for m in re.finditer(r"\(net (\"[^\"]*\"|\S+)", text):
    if m.start() < k:
        continue
    name = m.group(1).strip('"')
    if name in done:
        e = span(text, m.start())
        block = text[m.start():e]
        if "(pins" in block:
            p0 = block.index("(pins")
            block = block[:p0] + "(pins)" + block[span(block, p0):]
        out.append(text[k:m.start()] + block)
        k = e
out.append(text[k:])
text = "".join(out)


def narrow_classes(text):
    """Move each net drawn narrower than its class into a copy of the class with that width (see --drawn-widths)."""
    out, k, moved = [], 0, 0
    for m in re.finditer(r"\(class (\"[^\"]*\"|[^\s()]+)", text):
        if m.start() < k:
            continue
        e = span(text, m.start())
        block = text[m.start():e]
        c = block.index("(circuit") if "(circuit" in block else block.index("(rule")
        head, body = block[:c], block[c:]
        names = re.findall(r'"[^"]*"|[^\s()]+', head[len("(class"):])
        cls, nets = names[0], names[1:]
        wm = re.search(r"\(width ([\d.]+)\)", body)
        if not wm:
            continue
        width = float(wm.group(1))
        keep, groups = [], {}
        for n in nets:
            w = narrowest.get(n.strip('"'))
            if w is not None and w < width - 0.5:
                groups.setdefault(round(w, 1), []).append(n)
            else:
                keep.append(n)
        if not groups:
            continue
        ind = text[text.rfind("\n", 0, m.start()) + 1:m.start()]
        blocks = []
        if keep:
            blocks.append(f"(class {cls} " + " ".join(keep) + "\n" + ind + "  " + body.lstrip())
        for w, ns in sorted(groups.items()):
            name = cls.strip('"') + f"_w{w:g}"
            blocks.append(f'(class "{name}" ' + " ".join(ns) + "\n" + ind + "  " +
                          re.sub(r"\(width [\d.]+\)", f"(width {w:g})", body, count=1).lstrip())
            moved += len(ns)
        out.append(text[k:m.start()] + ("\n" + ind).join(blocks))
        k = e
    out.append(text[k:])
    if moved:
        print(f"make_dsn: {moved} nets narrowed to the widths drawn on them", file=sys.stderr)
    return "".join(out)


def grow_hole_keepouts(text):
    """Grow each image keepout circle that is a hole to the radius hole_keepout_radii() asks of it. The circle
    is matched to its pads through every placement of its image (an image shared by several parts takes the
    largest need)."""
    import math
    assert re.search(r"\(unit um\)", text), "make_dsn: expected a DSN in um"
    tok = r'("[^"]*"|[^\s()]+)'
    places = {}  # image -> [(x, y, back, rot)]
    p0 = text.index("(placement")
    pl = text[p0:span(text, p0)]
    for m in re.finditer(r"\(component " + tok, pl):
        comp = pl[m.start():span(pl, m.start())]
        places[m.group(1).strip('"')] = [(float(x), float(y), side == "back", float(r)) for _, x, y, side, r in
                                         re.findall(r"\(place " + tok + r" ([-\d.]+) ([-\d.]+) (\w+) ([-\d.]+)", comp)]
    out, k, grown, unmatched = [], 0, 0, 0
    for m in re.finditer(r"\(image " + tok, text):
        e = span(text, m.start())
        block = text[m.start():e]
        pls = places.get(m.group(1).strip('"'), [])

        def grow(c):
            nonlocal grown, unmatched
            d, x, y = float(c.group(2)), float(c.group(3) or 0), float(c.group(4) or 0)
            need = 0
            for px, py, back, rot in pls:  # as pcbroute places an image's keepouts
                qx = -x if back else x
                a = math.radians(rot)
                ax, ay = px + qx * math.cos(a) - y * math.sin(a), py + qx * math.sin(a) + y * math.cos(a)
                # the pads there whose drill the circle covers (a marker pad may sit on another part's hole)
                fit = [r for (hx, hy), pads in hole_radii.items() if abs(ax - hx) + abs(ay - hy) <= 2
                      for drill, r in pads if drill <= d + 1]
                if not fit:
                    unmatched += 1
                    return c.group(0)
                need = max(need, *fit)
            if 2 * need <= d + 0.05:
                return c.group(0)
            grown += 1
            at = f" {c.group(3)} {c.group(4)}" if c.group(3) else ""
            return f"(circle {c.group(1)} {math.ceil(2 * need * 10) / 10:g}{at})"

        block = re.sub(r"(?<=\(keepout \"\" )\(circle (\S+) ([\d.]+)(?: ([-\d.]+) ([-\d.]+))?\)", grow, block)
        out.append(text[k:m.start()] + block)
        k = e
    out.append(text[k:])
    if grown or unmatched:
        print(f"make_dsn: {grown} hole keepout circles grown to the hole / pad clearance"
              + (f", {unmatched} keepout circles match no hole (left as they are)" if unmatched else ""),
              file=sys.stderr)
    return "".join(out)


def add_zone_rules(text):
    """Write zone_rules after the last (plane) of (structure), each checked against its plane's net and layer."""
    tok = r'("[^"]*"|[^\s()]+)'
    planes = [(m.group(1).strip('"'), m.group(2).strip('"'), m.start())
              for m in re.finditer(r"\(plane " + tok + r"\s*\(\w+ " + tok, text)]
    if not planes:
        return text
    got = [(n, l) for n, l, _ in zone_rules]
    if got != [(n, l) for n, l, _ in planes]:
        print(f"make_dsn: the DSN's planes {[(n, l) for n, l, _ in planes]} aren't the zones {got}: "
              "no zone rules written", file=sys.stderr)
        return text
    q = lambda n: n if re.fullmatch(r"[^\s()\"]+", n) else '"' + n + '"'
    s = planes[-1][2]
    e = span(text, s)
    ind = text[text.rfind("\n", 0, s) + 1:s]
    body = "".join(f"\n{ind}  (zone {q(n)} {l} " + " ".join(f"({k} {v})" for k, v in props) + ")"
                   for n, l, props in zone_rules)
    return text[:e] + f"\n{ind}(pcbroute_zones{body}\n{ind})" + text[e:]


def add_board_rules(text):
    """Write (pcbroute_rules (edge_clearance UM)) at the end of (structure)."""
    if edge_um is None:
        return text
    s = text.find("(structure")
    e = span(text, s) - 1
    ind = text[text.rfind("\n", 0, s) + 1:s]
    return text[:e] + f"  (pcbroute_rules (edge_clearance {edge_um}))\n{ind}" + text[e:]


text = grow_hole_keepouts(text)
text = add_board_rules(text)
text = add_zone_rules(text)
if drawn_widths:
    text = narrow_classes(text)
open(sys.argv[2], "w").write(text)
