"""Geometry helpers: sampled reference paths, Frenet coordinates, frames, boxes.

Copied (with small simplifications) from the original research simulator.

Coordinate conventions used EVERYWHERE in this project
------------------------------------------------------
World (global) frame:  X = north, Y = east, heading measured CLOCKWISE from
north.  Ego frame:     x = forward, y = RIGHT, heading positive toward +y
(a right turn).  Both frames have the same handedness, so the same formulas
work in either one.

Frenet coordinates on a path:  s = distance along the path [m],
d = sideways offset, POSITIVE TO THE RIGHT of the path [m].
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


MIN_SEGMENT_LENGTH_M = 1e-6


# ---------------------------------------------------------------------------
# Reference path (a polyline with arc length, heading and curvature)
# ---------------------------------------------------------------------------
@dataclass(frozen=True, eq=False)
class ReferencePath:
    """Immutable sampled path.  Build it with ``ReferencePath.from_xy(points)``.

    * position between samples: linear interpolation in arc length s;
    * heading: linear interpolation of the per-sample tangent headings;
    * beyond either end the path continues straight along its end tangent.
    """

    xy_m: np.ndarray                  # (N, 2) samples
    s_m: np.ndarray                   # (N,) arc length, s_m[0] == 0
    heading_rad: np.ndarray           # (N,) tangent heading at each sample
    segment_length_m: np.ndarray      # (N-1,)
    segment_heading_rad: np.ndarray   # (N-1,)
    segment_curvature_1pm: np.ndarray  # (N-1,) d(heading)/ds, + = right turn

    @classmethod
    def from_xy(cls, xy_m) -> "ReferencePath":
        xy = np.array(xy_m, dtype=float, copy=True)
        if xy.ndim != 2 or xy.shape[1] != 2:
            raise ValueError(f"xy_m must have shape (N, 2), got {xy.shape}")
        if not np.all(np.isfinite(xy)):
            raise ValueError("path samples must be finite")
        if xy.shape[0] >= 2:
            step = np.hypot(*np.diff(xy, axis=0).T)
            keep = np.concatenate(([True], step > MIN_SEGMENT_LENGTH_M))
            xy = xy[keep]
        if xy.shape[0] < 2:
            raise ValueError("a path needs at least two distinct points")

        delta = np.diff(xy, axis=0)
        seg_len = np.hypot(delta[:, 0], delta[:, 1])
        seg_heading = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
        s = np.concatenate(([0.0], np.cumsum(seg_len)))
        heading = np.empty(xy.shape[0])
        heading[0] = seg_heading[0]
        heading[-1] = seg_heading[-1]
        if xy.shape[0] > 2:
            l_prev, l_next = seg_len[:-1], seg_len[1:]
            heading[1:-1] = (seg_heading[:-1] * l_next + seg_heading[1:] * l_prev) / (l_prev + l_next)
        curvature = np.diff(heading) / seg_len
        for array in (xy, s, heading, seg_len, seg_heading, curvature):
            array.setflags(write=False)
        return cls(xy, s, heading, seg_len, seg_heading, curvature)

    @property
    def length_m(self) -> float:
        return float(self.s_m[-1])

    @property
    def num_points(self) -> int:
        return int(self.xy_m.shape[0])

    def __repr__(self) -> str:
        return f"ReferencePath(points={self.num_points}, length_m={self.length_m:.1f})"


def _segment_index(path: ReferencePath, s: np.ndarray) -> np.ndarray:
    index = np.searchsorted(path.s_m, s, side="right") - 1
    return np.clip(index, 0, path.num_points - 2)


def reference_state(path: ReferencePath, s_m):
    """``(x, y, heading, curvature, segment_heading)`` at arc length(s) ``s_m``."""
    s = np.asarray(s_m, dtype=float)
    i = _segment_index(path, s)
    u = (s - path.s_m[i]) / path.segment_length_m[i]
    p0, p1 = path.xy_m[i], path.xy_m[i + 1]
    x = p0[..., 0] + u * (p1[..., 0] - p0[..., 0])
    y = p0[..., 1] + u * (p1[..., 1] - p0[..., 1])
    uc = np.clip(u, 0.0, 1.0)
    heading = path.heading_rad[i] + uc * (path.heading_rad[i + 1] - path.heading_rad[i])
    inside = (u >= 0.0) & (u <= 1.0)
    curvature = np.where(inside, path.segment_curvature_1pm[i], 0.0)
    return x, y, heading, curvature, path.segment_heading_rad[i]


def frenet_to_cartesian(path: ReferencePath, s_m, d_m) -> Tuple[np.ndarray, np.ndarray]:
    """Frenet (s, d) -> (x, y) in the frame of ``path``.  Works on arrays."""
    s = np.asarray(s_m, dtype=float)
    d = np.asarray(d_m, dtype=float)
    cx, cy, heading, _, _ = reference_state(path, s)
    return cx - d * np.sin(heading), cy + d * np.cos(heading)


def frenet_heading_to_cartesian(path: ReferencePath, s_m, d_m, dd_ds) -> np.ndarray:
    """Heading of a curve d(s) with slope dd/ds drawn along ``path``."""
    _, _, heading, curvature, _ = reference_state(path, np.asarray(s_m, dtype=float))
    return heading + np.arctan2(np.asarray(dd_ds, dtype=float), 1.0 - curvature * np.asarray(d_m, dtype=float))


def cartesian_to_frenet(path: ReferencePath, x_m: float, y_m: float) -> Tuple[float, float]:
    """One point (x, y) -> Frenet (s, d) on ``path`` (nearest segment + Newton)."""
    px, py = float(x_m), float(y_m)
    p0 = path.xy_m[:-1]
    seg = path.xy_m[1:] - p0
    rel = np.array([px, py]) - p0
    u = np.clip(np.einsum("ij,ij->i", rel, seg) / path.segment_length_m ** 2, 0.0, 1.0)
    proj = p0 + seg * u[:, None]
    k = int(np.argmin(np.sum((proj - np.array([px, py])) ** 2, axis=1)))
    s = float(path.s_m[k] + u[k] * path.segment_length_m[k])
    if k == 0 and u[k] == 0.0:
        s += float(np.dot(rel[0], seg[0]) / path.segment_length_m[0])
    elif k == path.num_points - 2 and u[k] == 1.0:
        s = float(path.s_m[-1] + np.dot(np.array([px, py]) - path.xy_m[-1], seg[-1]) / path.segment_length_m[-1])
    for _ in range(30):
        cx, cy, heading, curvature, seg_heading = (float(v) for v in reference_state(path, s))
        tx, ty = math.cos(heading), math.sin(heading)
        rx, ry = px - cx, py - cy
        f = rx * tx + ry * ty
        d = -rx * ty + ry * tx
        slope = -(math.cos(seg_heading) * tx + math.sin(seg_heading) * ty) + curvature * d
        if abs(slope) < 1e-6:
            break
        step = -f / slope
        s += step
        if abs(step) < 1e-10:
            break
    cx, cy, heading, _, _ = (float(v) for v in reference_state(path, s))
    d = -(px - cx) * math.sin(heading) + (py - cy) * math.cos(heading)
    return float(s), float(d)


def cartesian_to_frenet_many(path: ReferencePath, x_m, y_m, iterations: int = 8):
    """Many points (arrays) -> Frenet (s, d) arrays on ``path``."""
    px = np.atleast_1d(np.asarray(x_m, dtype=float))
    py = np.atleast_1d(np.asarray(y_m, dtype=float))
    if px.size == 0:
        return np.zeros(0), np.zeros(0)
    p0 = path.xy_m[:-1]
    seg = path.xy_m[1:] - p0
    rel_x = px[:, None] - p0[None, :, 0]
    rel_y = py[:, None] - p0[None, :, 1]
    u = np.clip((rel_x * seg[None, :, 0] + rel_y * seg[None, :, 1]) / path.segment_length_m ** 2, 0.0, 1.0)
    dx = rel_x - u * seg[None, :, 0]
    dy = rel_y - u * seg[None, :, 1]
    k = np.argmin(dx * dx + dy * dy, axis=1)
    rows = np.arange(px.size)
    s = path.s_m[k] + u[rows, k] * path.segment_length_m[k]
    first = (k == 0) & (u[rows, k] == 0.0)
    s = np.where(first, (rel_x[:, 0] * seg[0, 0] + rel_y[:, 0] * seg[0, 1]) / path.segment_length_m[0], s)
    last = (k == path.num_points - 2) & (u[rows, k] == 1.0)
    end_proj = ((px - path.xy_m[-1, 0]) * seg[-1, 0] + (py - path.xy_m[-1, 1]) * seg[-1, 1]) / path.segment_length_m[-1]
    s = np.where(last, path.s_m[-1] + end_proj, s)
    for _ in range(iterations):
        cx, cy, heading, curvature, seg_heading = reference_state(path, s)
        tx, ty = np.cos(heading), np.sin(heading)
        rx, ry = px - cx, py - cy
        f = rx * tx + ry * ty
        d = -rx * ty + ry * tx
        slope = -(np.cos(seg_heading) * tx + np.sin(seg_heading) * ty) + curvature * d
        slope = np.where(np.abs(slope) < 1e-6, -1.0, slope)
        s = s - f / slope
    cx, cy, heading, _, _ = reference_state(path, s)
    d = -(px - cx) * np.sin(heading) + (py - cy) * np.cos(heading)
    return s, d


# ---------------------------------------------------------------------------
# Frame transforms (world <-> ego)
# ---------------------------------------------------------------------------
def world_to_ego(ego_x_m: float, ego_y_m: float, ego_yaw_rad: float, x_m, y_m):
    """World (north, east) point(s) -> ego (x forward, y right)."""
    dx = np.asarray(x_m, dtype=float) - ego_x_m
    dy = np.asarray(y_m, dtype=float) - ego_y_m
    c, s = math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)
    return c * dx + s * dy, -s * dx + c * dy


def world_vector_to_ego(ego_yaw_rad: float, vx, vy):
    """Rotate a world vector into the ego axes (no translation)."""
    c, s = math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)
    vx, vy = np.asarray(vx, dtype=float), np.asarray(vy, dtype=float)
    return c * vx + s * vy, -s * vx + c * vy


def ego_to_world(ego_x_m: float, ego_y_m: float, ego_yaw_rad: float, x_m, y_m):
    """Ego (x forward, y right) point(s) -> world (north, east)."""
    c, s = math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)
    x, y = np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
    return ego_x_m + c * x - s * y, ego_y_m + s * x + c * y


def wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Oriented boxes (vehicle footprints)
# ---------------------------------------------------------------------------
def box_corners(center_x, center_y, heading_rad, length_m, width_m) -> np.ndarray:
    """Four corners (4, 2) of a rectangle centred at (x, y) with a heading."""
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    hl, hw = length_m / 2.0, width_m / 2.0
    local = np.array([(hl, -hw), (hl, hw), (-hl, hw), (-hl, -hw)])
    return np.column_stack((center_x + local[:, 0] * c - local[:, 1] * s,
                            center_y + local[:, 0] * s + local[:, 1] * c))


def boxes_overlap(c1, h1, l1, w1, c2, h2, l2, w2) -> bool:
    """Separating-axis test for two oriented rectangles (centre, heading, length, width)."""
    axes = [(math.cos(h1), math.sin(h1)), (-math.sin(h1), math.cos(h1)),
            (math.cos(h2), math.sin(h2)), (-math.sin(h2), math.cos(h2))]
    d = (c2[0] - c1[0], c2[1] - c1[1])
    for ax, ay in axes:
        r1 = 0.5 * l1 * abs(math.cos(h1) * ax + math.sin(h1) * ay) + 0.5 * w1 * abs(-math.sin(h1) * ax + math.cos(h1) * ay)
        r2 = 0.5 * l2 * abs(math.cos(h2) * ax + math.sin(h2) * ay) + 0.5 * w2 * abs(-math.sin(h2) * ax + math.cos(h2) * ay)
        if abs(d[0] * ax + d[1] * ay) > r1 + r2:
            return False
    return True


def boxes_overlap_many(c1x, c1y, h1, l1, w1, c2x, c2y, h2, l2, w2) -> np.ndarray:
    """Vectorized ``boxes_overlap`` over arrays of box pairs (same length)."""
    c1x, c1y, h1 = (np.asarray(v, dtype=float) for v in (c1x, c1y, h1))
    c2x, c2y, h2 = (np.asarray(v, dtype=float) for v in (c2x, c2y, h2))
    dx, dy = c2x - c1x, c2y - c1y
    separated = np.zeros(np.broadcast(c1x, c2x).shape, dtype=bool)
    for hx in (h1, h2):
        for ax, ay in ((np.cos(hx), np.sin(hx)), (-np.sin(hx), np.cos(hx))):
            r1 = 0.5 * l1 * np.abs(np.cos(h1) * ax + np.sin(h1) * ay) + 0.5 * w1 * np.abs(-np.sin(h1) * ax + np.cos(h1) * ay)
            r2 = 0.5 * l2 * np.abs(np.cos(h2) * ax + np.sin(h2) * ay) + 0.5 * w2 * np.abs(-np.sin(h2) * ax + np.cos(h2) * ay)
            separated |= np.abs(dx * ax + dy * ay) > r1 + r2
    return ~separated


def point_in_polygon(x, y, polygon: np.ndarray) -> np.ndarray:
    """Even-odd ray-casting test; ``x``, ``y`` may be arrays.

    Vectorized over the polygon edges (a road surface has thousands of
    vertices and this runs every simulation sub-step)."""
    x = np.atleast_1d(np.asarray(x, dtype=float))
    y = np.atleast_1d(np.asarray(y, dtype=float))
    shape = x.shape
    x, y = x.ravel(), y.ravel()
    ax, ay = polygon[:, 0], polygon[:, 1]
    bx, by = np.roll(ax, -1), np.roll(ay, -1)
    inside = np.zeros(x.size, dtype=bool)
    chunk = max(1, 200_000 // max(ax.size, 1))               # bound memory for many query points
    for i in range(0, x.size, chunk):
        xc, yc = x[i:i + chunk, None], y[i:i + chunk, None]
        crosses = (ay > yc) != (by > yc)
        with np.errstate(divide="ignore", invalid="ignore"):
            x_cross = ax + (yc - ay) * (bx - ax) / (by - ay)
        inside[i:i + chunk] = np.count_nonzero(crosses & (xc < x_cross), axis=1) % 2 == 1
    return inside.reshape(shape)
