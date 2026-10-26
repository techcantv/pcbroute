"""Routing-guideline metrics that the totals of benchmark.py hide, per net type (pcbroute --nt-dump classifies).

usage: python3 nt_metrics.py RESULTS.json [--configs a,b] [--ref a] [--boards-out FILE]

For every board and config of a benchmark.py results file (its sessions are in ses/ next to it), from the session:
  fast nets (differential, high-speed, crystal): vias, track length
  differential pairs: skew |L1 - L2| and the coupled share (the longer share of one net's track within --couple mm
  of the other's, same layer), over the pairs with both nets routed in every config
  ground / power nets with a plane that the session routes: each SMD pad's distance to the nearest point the net reaches its pour (a via, a through-hole
  pad; 0 for a pad on a pour layer), and their track length
  supplies without a plane: track width (length-weighted) and length
  crystals: other nets' vias within 1 mm of a crystal net's pads
"""
import argparse
import json
import math
import os
import re
import statistics
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PCBROUTE = os.path.join(HERE, os.environ.get("ROUTER_DIR", "build"), "pcbroute")
FAST = {"diff", "highspeed", "crystal"}


def read_ses(path):
    """{net: {"segs": [(layer, w, x0, y0, x1, y1)], "vias": [(x, y)]}} in um."""
    text = open(path).read()
    m = re.search(r"\(resolution (\S+) (\d+)\)", text)
    unit = {"um": 1.0, "mm": 1000.0, "mil": 25.4, "inch": 25400.0}[m.group(1)] / float(m.group(2)) if m else 0.1
    nets = {}
    # nets are "(net NAME" followed by wires and vias until the next net
    parts = re.split(r"\n\s*\(net ", text)
    for part in parts[1:]:
        nm = re.match(r'("(?:[^"\\]|\\.)*"|\S+)', part).group(1)
        nm = nm[1:-1] if nm.startswith('"') else nm
        d = nets.setdefault(nm, {"segs": [], "vias": []})
        for p in re.finditer(r"\(path (\S+) (\S+)((?:\s+-?[\d.]+)+)\s*\)", part):
            v = [float(x) * unit for x in p.group(3).split()]
            w = float(p.group(2)) * unit
            pts = list(zip(v[0::2], v[1::2]))
            for a, b in zip(pts, pts[1:]):
                d["segs"].append((p.group(1), w, a[0], a[1], b[0], b[1]))
        for p in re.finditer(r"\(via \S+ (-?[\d.]+) (-?[\d.]+)", part):
            d["vias"].append((float(p.group(1)) * unit, float(p.group(2)) * unit))
    return nets


def seg_len(s):
    return math.hypot(s[4] - s[2], s[5] - s[3])


def pt_seg(px, py, s):
    ax, ay, bx, by = s[2], s[3], s[4], s[5]
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0 if l2 == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


def coupled(a_segs, b_segs, dmax):
    """Share of b's track length within dmax of a's track on the same layer (sampled every 0.1 mm)."""
    tot = near = 0.0
    by_layer = {}
    for s in a_segs:
        by_layer.setdefault(s[0], []).append(s)
    for s in b_segs:
        L = seg_len(s)
        n = max(1, int(L / 100))
        cand = by_layer.get(s[0], [])
        for k in range(n):
            t = (k + 0.5) / n
            px, py = s[2] + t * (s[4] - s[2]), s[3] + t * (s[5] - s[3])
            tot += L / n
            if any(pt_seg(px, py, c) <= dmax for c in cand):
                near += L / n
    return near / tot if tot else 0.0


def net_types(dsn, cache):
    if not os.path.exists(cache):
        subprocess.run([PCBROUTE, "-q", "-de", dsn, "--nt-dump", cache], check=True)
    return json.load(open(cache))


def board_metrics(nt, ses, couple_um):
    R = read_ses(ses)
    layers = nt["layers"]
    nets = {n["name"]: n for n in nt["nets"]}
    out = dict(fast_nets=0, fast_vias=0, fast_len=0.0, pairs={}, plane_pads=[], plane_len=0.0,
               supply_len=0.0, supply_w=0.0, xtal_vias_near=0)
    for name, n in nets.items():
        r = R.get(name, {"segs": [], "vias": []})
        L = sum(seg_len(s) for s in r["segs"])
        if n["type"] in FAST and len(n["pads"]) >= 2:
            out["fast_nets"] += 1
            out["fast_vias"] += len(r["vias"])
            out["fast_len"] += L
        if n["plane"] and n["type"] in ("ground", "power") and r["segs"] + r["vias"]:  # a supply with a plane that this session routes
            out["plane_len"] += L
            # where the net reaches its pours: its vias (new and existing), its through-hole pads
            access = list(r["vias"]) + [tuple(v) for v in n.get("vias", [])] + [(p[0], p[1]) for p in n["pads"] if not p[3]]
            for p in n["pads"]:
                if not p[3]:
                    continue
                if p[2] in n["plane"]:
                    out["plane_pads"].append(0.0)
                elif access:
                    out["plane_pads"].append(min(math.hypot(p[0] - a[0], p[1] - a[1]) for a in access))
        elif n["type"] in ("ground", "power") and r["segs"]:
            out["supply_len"] += L
            out["supply_w"] += sum(s[1] * seg_len(s) for s in r["segs"])
    seen = set()
    for name, n in nets.items():
        p = n["pair"]
        if not p or n["type"] not in ("diff", "analog") or (p, name) in seen:
            continue
        seen.add((name, p))
        a, b = R.get(name), R.get(p)
        if not a or not b or not a["segs"] or not b["segs"]:
            continue
        la, lb = sum(seg_len(s) for s in a["segs"]), sum(seg_len(s) for s in b["segs"])
        out["pairs"][name + " / " + p] = (abs(la - lb), max(coupled(a["segs"], b["segs"], couple_um), coupled(b["segs"], a["segs"], couple_um)))
    xt = [(p[0], p[1]) for n in nets.values() if n["type"] == "crystal" for p in n["pads"]]
    if xt:
        for name, r in R.items():
            if nets.get(name, {}).get("type") == "crystal":
                continue
            for v in r["vias"]:
                if any(math.hypot(v[0] - x, v[1] - y) < 1000 for x, y in xt):
                    out["xtal_vias_near"] += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results")
    ap.add_argument("--configs")
    ap.add_argument("--couple", type=float, default=1.0, help="mm: a pair's tracks this close count as coupled")
    ap.add_argument("--json", help="write the per-board metrics here")
    a = ap.parse_args()
    res = json.load(open(a.results))
    sesdir = os.path.join(os.path.dirname(os.path.abspath(a.results)), "ses")
    cfgs = a.configs.split(",") if a.configs else sorted({c for r in res.values() for c in r if c != "dsn"})
    per = {}
    for bid, r in sorted(res.items()):
        nt = net_types(r["dsn"], os.path.join(sesdir, f"{bid[:16]}_nettypes.json"))
        for c in cfgs:
            for k, row in enumerate(r.get(c, [])):
                ses = os.path.join(sesdir, f"{bid[:16]}_{c}_{k}.ses")
                if os.path.exists(ses):
                    per.setdefault(c, {})[bid] = dict(board_metrics(nt, ses, a.couple * 1000), open=row.get("open"),
                                                      violations=row.get("violations"), vias=row.get("vias"),
                                                      length=row.get("length"), route_s=row.get("route_s"))
                    break
    if a.json:
        json.dump(per, open(a.json, "w"), indent=1)
    # totals over the boards every config has
    common = set.intersection(*(set(per.get(c, {})) for c in cfgs)) if cfgs else set()
    hdr = ["config", "boards", "open", "viol", "vias", "len m", "route s", "fast vias", "fast len m", "pairs",
           "skew mean mm", "coupled", "plane pads", "pad-plane mean mm", "pad-plane >1mm", "plane len m",
           "supply w um", "supply len m", "xtal vias"]
    print("\t".join(hdr))
    for c in cfgs:
        B = [per[c][b] for b in sorted(common)]
        S = lambda k: sum((x[k] or 0) for x in B)
        # pairs routed (both nets) in every config, so that skew and coupling compare the same pairs
        both = [(b, k) for b in sorted(common) for k in per[cfgs[0]][b]["pairs"] if all(k in per[c2][b]["pairs"] for c2 in cfgs)]
        skew = [per[c][b]["pairs"][k][0] for b, k in both]
        cp = [per[c][b]["pairs"][k][1] for b, k in both]
        pp = [s for x in B for s in x["plane_pads"]]
        row = [c, len(B), S("open"), S("violations"), S("vias"), round(S("length") / 1000, 2), round(S("route_s"), 1),
               S("fast_vias"), round(S("fast_len") / 1e6, 2), len(skew),
               round(statistics.mean(skew) / 1000, 2) if skew else "-", round(statistics.mean(cp), 3) if cp else "-",
               len(pp), round(statistics.mean(pp) / 1000, 3) if pp else "-", sum(1 for x in pp if x > 1000),
               round(S("plane_len") / 1e6, 2), round(S("supply_w") / S("supply_len"), 0) if S("supply_len") else "-",
               round(S("supply_len") / 1e6, 2), S("xtal_vias_near")]
        print("\t".join(str(x) for x in row))


if __name__ == "__main__":
    sys.exit(main())
