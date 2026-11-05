"""Outline rings of filled shapes: free edges versus shared wall strip.

Implements "The rule, exactly" of docs/superpowers/specs/2026-10-20-shape-outline-height-design.md.
The strip test and its corner extension (step 3b) are vectorised; they equal the prototypes'
per-cell loops (outline-proto.py and corner-proto.py's ``extend()``).
"""
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

_ROW = np.array([[0, 0, 0], [1, 1, 1], [0, 0, 0]], bool)  # horizontal runs
_COL = _ROW.T  # vertical runs


@dataclass(frozen=True)
class Rings:
    owner: np.ndarray  # int32: 0 = in no filled ring, else the owning (tallest, lowest-id) shape
    strip: np.ndarray  # bool: ring cells that continue a wall line


def _shift(a: np.ndarray, dr: int, dc: int, fill=0) -> np.ndarray:
    """out[r, c] = a[r + dr, c + dc]; cells whose source is off the map get ``fill``."""
    h, w = a.shape
    out = np.full_like(a, fill)
    out[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)] = \
        a[max(0, dr):h + min(0, dr), max(0, dc):w + min(0, dc)]
    return out


def _run_length(mask: np.ndarray, structure: np.ndarray) -> np.ndarray:
    """Per cell: length of the run of ``mask`` through it along ``structure`` (0 off the mask)."""
    lab, n = ndimage.label(mask, structure=structure)
    sizes = np.bincount(lab.ravel(), minlength=n + 1)
    sizes[0] = 0
    return sizes[lab]


def _walk_hits(ring: np.ndarray, target: np.ndarray, structure: np.ndarray, steps) -> np.ndarray:
    """Ring cells whose walk (through ring cells, either way along ``structure``) first leaves
    the ring onto a ``target`` cell.

    Every cell of a straight ring run leaves it at the same two cells, one past each end, so a
    whole run qualifies when either end's outer neighbour is a target. A ring cell whose
    neighbour in a step is a target is necessarily that run's end (targets lie outside the ring).
    """
    lab, n = ndimage.label(ring, structure=structure)
    end = np.zeros_like(ring)
    for dr, dc in steps:
        end |= _shift(target, dr, dc, False)
    good = np.zeros(n + 1, bool)
    good[lab[ring & end]] = True
    good[0] = False
    return good[lab]


def _corner_extension(occ: np.ndarray, ring: np.ndarray, strip: np.ndarray) -> np.ndarray:
    """Step 3b, applied once to the step-3 ``strip``: a straight ring run becomes all strip when
    one of its end cells is strip and the occupied line across the run at that cell does not
    continue on both sides (the wall turns a corner there: an L, not a T).

    Only step-3 strip cells trigger it, in both directions, so cells it adds trigger nothing
    (no chaining). A run that is already all strip is unchanged by the union.
    """
    out = strip.copy()
    for structure, along, across in ((_ROW, ((0, -1), (0, 1)), ((-1, 0), (1, 0))),
                                     (_COL, ((-1, 0), (1, 0)), ((0, -1), (0, 1)))):
        lab, n = ndimage.label(ring, structure=structure)
        end = ring & ~(_shift(ring, *along[0], False) & _shift(ring, *along[1], False))
        tee = _shift(occ, *across[0], False) & _shift(occ, *across[1], False)
        good = np.zeros(n + 1, bool)
        good[lab[end & strip & ~tee]] = True
        good[0] = False
        out |= good[lab]
    return out


def classify_rings(occ: np.ndarray, labels: np.ndarray, heights: dict) -> Rings:
    """Classify the rings of filled shapes.

    occ: occupied mask after erasing. labels: interior ids (0 = none). heights: filled shape
    id -> height. Ring of k = cells of occ 8-adjacent to an interior cell of k; a cell's owner is
    the tallest such filled shape, the lowest id on a tie. A ring cell is strip when walking from
    it left, right, up or down through ring cells first leaves the ring onto a remaining wall
    cell w whose wall run along the walk is strictly longer than its run across it. Then, once,
    a ring run with a strip end cell where the wall turns a corner becomes all strip (step 3b).
    """
    occ = np.asarray(occ, bool)
    labels = np.asarray(labels)
    top = max([int(labels.max()) if labels.size else 0, *heights])
    by_id = np.full(top + 1, -np.inf)
    for k, h in heights.items():
        by_id[k] = h
    owner = np.zeros(occ.shape, np.int32)
    best = np.full(occ.shape, -np.inf)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == dc == 0:
                continue
            nid = _shift(labels, dr, dc, 0)
            nh = by_id[nid]
            take = (nh > best) | ((nh == best) & (nh > -np.inf) & (nid < owner))
            owner = np.where(take, nid, owner).astype(np.int32)
            best = np.where(take, nh, best)
    owner[~occ] = 0
    ring = owner > 0
    wall = occ & ~ring
    hrun = _run_length(wall, _ROW)
    vrun = _run_length(wall, _COL)
    strip = (_walk_hits(ring, wall & (hrun > vrun), _ROW, ((0, 1), (0, -1)))
             | _walk_hits(ring, wall & (vrun > hrun), _COL, ((1, 0), (-1, 0))))
    return Rings(owner=owner, strip=_corner_extension(occ, ring, strip & ring))
