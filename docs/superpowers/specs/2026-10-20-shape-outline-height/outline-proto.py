"""Throwaway: classify filled-shape ring cells into free edge vs shared wall strip."""
import numpy as np
from scipy import ndimage

N8 = np.ones((3, 3), bool)

def runs(W, r, c):
    H, Wd = W.shape
    def run(dr, dc):
        n, rr, cc = 0, r + dr, c + dc
        while 0 <= rr < H and 0 <= cc < Wd and W[rr, cc]:
            n += 1; rr += dr; cc += dc
        return n
    return 1 + run(0, -1) + run(0, 1), 1 + run(-1, 0) + run(1, 0)  # horizontal, vertical

def classify(occ, labels, filled):
    ring = np.zeros_like(occ)
    for k in filled:
        ring |= ndimage.binary_dilation(labels == k, structure=N8) & occ
    W = occ & ~ring
    H, Wd = occ.shape
    strip = np.zeros_like(occ)
    for r, c in zip(*np.nonzero(ring)):
        for dr, dc, horiz in ((0, -1, True), (0, 1, True), (-1, 0, False), (1, 0, False)):
            rr, cc = r + dr, c + dc
            while 0 <= rr < H and 0 <= cc < Wd and ring[rr, cc]:
                rr += dr; cc += dc
            if 0 <= rr < H and 0 <= cc < Wd and W[rr, cc]:
                h, v = runs(W, rr, cc)
                if (horiz and h > v) or (not horiz and v > h):
                    strip[r, c] = True; break
    return ring, strip

def grid(rows):
    return np.array([[ch == "#" for ch in row] for row in rows])

def show(name, occ):
    labels, n = ndimage.label(ndimage.binary_fill_holes(occ) & ~occ)
    ring, strip = classify(occ, labels, range(1, n + 1))
    print(name)
    for r in range(occ.shape[0]):
        print("  " + "".join("S" if strip[r, c] else "f" if ring[r, c] else "#" if occ[r, c] else "." for c in range(occ.shape[1])))

show("C: table against 1-cell wall", grid([
    "###############",
    "....#.....#....",
    "....#.....#....",
    "....#######....",
    "...............",]))
show("E: table against 2-cell wall", grid([
    "###############",
    "###############",
    "....#.....#....",
    "....#######....",
    "...............",]))
show("F: table in a corner", grid([
    "##########",
    "#....#....",
    "#....#....",
    "######....",
    "#.........",
    "#.........",]))
show("A: free-standing", grid([
    "..........",
    "..######..",
    "..#....#..",
    "..######..",
    "..........",]))
