"""Throwaway: one-step L-corner extension of the strip test, on top of map_check.rings."""
import json, sys
import numpy as np
from pathlib import Path
from map_check.rings import classify_rings

def runs_of(mask, horiz):
    """yield lists of (r,c) for maximal straight runs of mask cells"""
    H, W = mask.shape
    out = []
    rng = range(H) if horiz else range(W)
    for a in rng:
        cur = []
        for b in range(W if horiz else H):
            r, c = (a, b) if horiz else (b, a)
            if mask[r, c]:
                cur.append((r, c))
            else:
                if cur: out.append(cur); cur = []
        if cur: out.append(cur)
    return out

def extend(occ, owner, strip):
    ring = owner > 0
    H, W = occ.shape
    def o(r, c): return 0 <= r < H and 0 <= c < W and occ[r, c]
    new = strip.copy()
    for horiz in (True, False):
        for run in runs_of(ring, horiz):
            if all(strip[p] for p in run): continue
            for e in (run[0], run[-1]):
                if not strip[e]: continue
                r, c = e
                # occupied line through e perpendicular to the run
                a, b = ((r - 1, c), (r + 1, c)) if horiz else ((r, c - 1), (r, c + 1))
                if not (o(*a) and o(*b)):          # L corner (line ends at e), not a T
                    for p in run: new[p] = True
                    break
    return new

def show(sk, heights):
    occ = np.array([[ch == '#' for ch in r] for r in sk])
    labels = np.array([[1 if ch == 'i' else 0 for ch in r] for r in sk], np.int32)
    rr = classify_rings(occ, labels, heights)
    s2 = extend(occ, rr.owner, rr.strip)
    for i, row in enumerate(sk):
        print('  ' + ''.join('.' if not occ[i, j] and row[j] != 'i' else 'i' if row[j] == 'i' else ('S' if s2[i, j] else ('f' if rr.owner[i, j] else '#')) for j in range(len(row))))

print("door case (reviewer):")
show(["#########.....####",
      "#iiiiii#..........",
      "#iiiiii#..........",
      "########..........",
      "#.................",
      "#................."], {1: 0.3})
print("wall ends exactly at the table corner (no jamb):")
show(["########......####",
      "#iiiiii#..........",
      "#iiiiii#..........",
      "########..........",
      "#.................",
      "#................."], {1: 0.3})

# fixture cases must be unchanged
fx = json.loads(Path("tools/map_check/tests/fixtures/outline_golden.json").read_text())
bad = []
for case in fx:
    cells = np.array(case["cells"])
    occ = cells == 255
    for i in case["erased"]:
        occ.flat[i] = False
    labels = np.where((cells > 0) & (cells < 255), cells, 0).astype(np.int32)
    hs = {int(k): v["height"] for k, v in case["shapes"].items() if v["fill"]}
    rr = classify_rings(occ, labels, hs)
    if (extend(occ, rr.owner, rr.strip) != rr.strip).any():
        bad.append(case["name"])
print("fixture cases changed by the extension:", bad)

# real maps
from map_check.occupancy import load_map_dir, closed_outlines
D = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps")
tot = 0
for y in sorted(D.glob("*.yaml")):
    if y.name.endswith(".poses.yaml"): continue
    m = load_map_dir(y)
    lab, outs = closed_outlines(m)
    rr = classify_rings(m.occupied, lab, {o.id: 0.3 for o in outs if o.area_m2 <= 4.0})
    d = int((extend(m.occupied, rr.owner, rr.strip) & ~rr.strip).sum())
    if d:
        tot += d; print(f"  {y.stem}: +{d} strip cells")
print("total cells moved free-edge -> strip:", tot)
