"""Road network model: lanes, markings, stop lines, crosswalks, signals.

Copied from the original research simulator and trimmed to the road types
used in this challenge (the geometry code itself is unchanged, so roads look
exactly like the original).

World frame: X north, Y east, heading clockwise from north.  Right-hand
traffic.  On a straight road the reference line runs "forward"; forward
lanes lie to its RIGHT (+d), opposing lanes to its LEFT.

Lane IDs
--------
Straight road named ``R`` with n lanes per direction:
    ``R:F1`` (inner forward lane, next to the centre line) ... ``R:Fn`` (outer)
    ``R:O1`` ... ``R:On``  (opposing lanes, travelling the other way)
Four-way intersection (one lane per direction, arms A0 south, A1 west,
A2 north, A3 east):
    approach lanes ``A0:in`` .. ``A3:in``, exit lanes ``A0:out`` .. ``A3:out``,
    connectors ``A0>A2:S`` (straight), ``A0>A1:L`` (left), ``A0>A3:R`` (right).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from simulator.geometry import ReferencePath, frenet_to_cartesian, reference_state


LANE_WIDTH_M = 3.5
SAMPLE_M = 0.5


class MarkingStyle(str, Enum):
    DASHED_WHITE = "DASHED_WHITE"
    SOLID_WHITE = "SOLID_WHITE"
    DOUBLE_YELLOW = "DOUBLE_YELLOW"
    ROAD_EDGE = "ROAD_EDGE"


class SignalState(str, Enum):
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"


class BoundaryType(str, Enum):
    """What the paint between two lanes means."""

    DASHED = "DASHED"                  # lane change permitted
    SOLID = "SOLID"                    # lane change prohibited
    DOUBLE_YELLOW = "DOUBLE_YELLOW"    # centre line: never cross
    EDGE = "EDGE"                      # road edge
    MERGE = "MERGE"                    # lane-drop taper: merging permitted
    NONE = "NONE"                      # unmarked (junction box): no lane change


CROSSABLE_BOUNDARIES = frozenset({BoundaryType.DASHED, BoundaryType.MERGE})

BoundarySpans = Tuple[Tuple[float, float, BoundaryType], ...]


@dataclass(frozen=True, eq=False)
class LaneMarking:
    style: MarkingStyle
    points_m: np.ndarray            # (N, 2) world polyline
    arc_m: np.ndarray = field(init=False, repr=False)   # arc length (anchors dash patterns)

    def __post_init__(self) -> None:
        step = np.hypot(*np.diff(self.points_m, axis=0).T) if len(self.points_m) > 1 else np.zeros(0)
        object.__setattr__(self, "arc_m", np.concatenate(([0.0], np.cumsum(step))))


@dataclass(frozen=True, eq=False)
class Lane:
    lane_id: str
    road_id: str
    centerline: ReferencePath       # world frame, in the direction of travel
    width_m: float
    left_lane_id: Optional[str] = None     # neighbouring lanes in the SAME direction
    right_lane_id: Optional[str] = None
    left_boundary: BoundarySpans = ()
    right_boundary: BoundarySpans = ()
    junction_id: Optional[str] = None      # set on junction connector lanes
    movement: Optional[str] = None         # STRAIGHT / LEFT / RIGHT on connectors

    def boundary_at(self, side: str, s_m: float) -> BoundaryType:
        spans = self.left_boundary if side == "left" else self.right_boundary
        for s0, s1, kind in spans:
            if s0 - 1e-9 <= s_m <= s1 + 1e-9:
                return kind
        return BoundaryType.NONE


@dataclass(frozen=True, eq=False)
class RoadReference:
    road_id: str
    reference: ReferencePath        # road reference line (used to place actors)


@dataclass(frozen=True)
class StopLine:
    stop_line_id: str
    lane_id: str
    p0_m: Tuple[float, float]
    p1_m: Tuple[float, float]
    s_on_lane_m: float

    @property
    def midpoint_m(self) -> Tuple[float, float]:
        return (0.5 * (self.p0_m[0] + self.p1_m[0]), 0.5 * (self.p0_m[1] + self.p1_m[1]))


@dataclass(frozen=True, eq=False)
class Crosswalk:
    crosswalk_id: str
    polygon_m: np.ndarray           # (4, 2); first edge runs ACROSS the road
    signal_id: Optional[str] = None


@dataclass(frozen=True, eq=False)
class Sidewalk:
    sidewalk_id: str
    centerline_m: np.ndarray        # (N, 2) world polyline
    width_m: float


@dataclass(frozen=True)
class StopSign:
    sign_id: str
    position_m: Tuple[float, float]
    stop_line_id: str


@dataclass(frozen=True)
class TrafficSignal:
    signal_id: str
    position_m: Tuple[float, float]
    stop_line_id: str


@dataclass(frozen=True, eq=False)
class RoadNetwork:
    name: str
    main_road_id: str
    roads: Mapping[str, RoadReference]
    lanes: Mapping[str, Lane]
    markings: Tuple[LaneMarking, ...]
    surfaces: Tuple[np.ndarray, ...]          # asphalt polygons (world)
    stop_lines: Mapping[str, StopLine] = field(default_factory=dict)
    crosswalks: Tuple[Crosswalk, ...] = ()
    stop_signs: Tuple[StopSign, ...] = ()
    signals: Mapping[str, TrafficSignal] = field(default_factory=dict)
    sidewalks: Tuple[Sidewalk, ...] = ()
    junction_boxes: Mapping[str, np.ndarray] = field(default_factory=dict)
    _route_cache: Dict[Tuple[str, ...], ReferencePath] = field(default_factory=dict, repr=False)

    def route_path(self, lane_ids: Sequence[str]) -> ReferencePath:
        """Join lane centrelines into one world-frame path."""
        key = tuple(lane_ids)
        if key not in self._route_cache:
            if not key:
                raise ValueError("a route needs at least one lane")
            for lane_id in key:
                if lane_id not in self.lanes:
                    raise KeyError(f"unknown lane '{lane_id}'. Known lanes: {sorted(self.lanes)}")
            pts = np.vstack([self.lanes[lane_id].centerline.xy_m for lane_id in key])
            self._route_cache[key] = ReferencePath.from_xy(pts)
        return self._route_cache[key]

    def lane_start_on_route(self, lane_ids: Sequence[str], lane_id: str) -> float:
        """Route distance s at which ``lane_id`` begins (lane must be on the route)."""
        offset = 0.0
        for lid in lane_ids:
            if lid == lane_id:
                return offset
            offset += self.lanes[lid].centerline.length_m
        raise KeyError(f"lane {lane_id} is not on route {tuple(lane_ids)}")


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class CurvatureSegment:
    length_m: float
    curvature_start_1pm: float
    curvature_end_1pm: float   # linear ramp (clothoid) between start and end


def integrate_curvature(segments: Sequence[CurvatureSegment], x0=0.0, y0=0.0, h0=0.0, step_m: float = 0.05):
    """Integrate a piecewise-linear curvature profile -> (x, y, heading) arrays."""
    total = sum(seg.length_m for seg in segments)
    n = max(1, int(math.ceil(total / step_m)))
    ds = total / n
    s_mid = (np.arange(n) + 0.5) * ds
    bounds = np.cumsum([0.0] + [seg.length_m for seg in segments])
    kappa = np.empty(n)
    for i, seg in enumerate(segments):
        mask = (s_mid >= bounds[i]) & (s_mid < bounds[i + 1] + (1e-9 if i == len(segments) - 1 else 0.0))
        frac = (s_mid[mask] - bounds[i]) / seg.length_m
        kappa[mask] = seg.curvature_start_1pm + frac * (seg.curvature_end_1pm - seg.curvature_start_1pm)
    heading = h0 + np.concatenate(([0.0], np.cumsum(kappa * ds)))
    h_mid = 0.5 * (heading[:-1] + heading[1:])
    x = x0 + np.concatenate(([0.0], np.cumsum(np.cos(h_mid) * ds)))
    y = y0 + np.concatenate(([0.0], np.cumsum(np.sin(h_mid) * ds)))
    return x, y, heading


def _resample(x: np.ndarray, y: np.ndarray, spacing_m: float = SAMPLE_M) -> np.ndarray:
    path = ReferencePath.from_xy(np.column_stack((x, y)))
    s = np.linspace(0.0, path.length_m, max(2, int(round(path.length_m / spacing_m)) + 1))
    px, py = frenet_to_cartesian(path, s, np.zeros_like(s))
    return np.column_stack((px, py))


def reference_from_curvature(segments: Sequence[CurvatureSegment], x0: float = 0.0,
                             y0: float = 0.0, h0: float = 0.0) -> ReferencePath:
    x, y, _ = integrate_curvature(segments, x0, y0, h0)
    return ReferencePath.from_xy(_resample(x, y))


def _smoothstep(s: np.ndarray, s0: float, s1: float) -> np.ndarray:
    u = np.clip((np.asarray(s, float) - s0) / max(s1 - s0, 1e-9), 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def _offset_profile(reference: ReferencePath, s: np.ndarray, d: np.ndarray) -> np.ndarray:
    x, y = frenet_to_cartesian(reference, s, d)
    return np.column_stack((x, y))


def _s_grid(s_from: float, s_to: float) -> np.ndarray:
    s = np.arange(s_from, s_to + 1e-9, SAMPLE_M)
    if s[-1] < s_to - 1e-9:
        s = np.append(s, s_to)
    return s


def _offset_polyline(reference: ReferencePath, d_m: float, s_from: float = 0.0,
                     s_to: Optional[float] = None) -> np.ndarray:
    s_to = reference.length_m if s_to is None else s_to
    s = _s_grid(s_from, s_to)
    x, y = frenet_to_cartesian(reference, s, np.full_like(s, d_m))
    return np.column_stack((x, y))


def _rotate(points, heading_rad: float) -> np.ndarray:
    """Rotate world points about the origin by a clockwise heading."""
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(heading_rad), math.sin(heading_rad)
    return np.column_stack((p[:, 0] * c - p[:, 1] * s, p[:, 0] * s + p[:, 1] * c))


def _arc(center, start, end, n: int = 12) -> np.ndarray:
    a0 = math.atan2(start[1] - center[1], start[0] - center[0])
    a1 = math.atan2(end[1] - center[1], end[0] - center[0])
    delta = (a1 - a0 + math.pi) % (2.0 * math.pi) - math.pi
    r = math.hypot(start[0] - center[0], start[1] - center[1])
    a = a0 + np.linspace(0.0, delta, n)
    return np.column_stack((center[0] + r * np.cos(a), center[1] + r * np.sin(a)))


# ---------------------------------------------------------------------------
# Multi-lane road
# ---------------------------------------------------------------------------
def road_segment(name: str, reference: ReferencePath, lanes_per_direction: int = 2,
                 lane_width_m: float = LANE_WIDTH_M, *, solid_divider_until_s_m: float = 0.0,
                 lane_drop: Optional[Tuple[float, float]] = None,
                 sidewalk_width_m: float = 0.0, crosswalk_s_m: Sequence[float] = ()) -> RoadNetwork:
    """Divided two-way road: forward lanes F1 (inner) .. Fn (outer) right of the
    reference line, opposing lanes O1..On left of it.

    ``solid_divider_until_s_m``: lane dividers are solid (no lane change)
    before this s and dashed after it.  ``lane_drop=(s_start, taper_m)``: the
    outer lane ends with a taper.  ``sidewalk_width_m``: sidewalks on both
    sides.  ``crosswalk_s_m``: positions of mid-block crosswalks.
    """
    n, w = lanes_per_direction, lane_width_m
    length = reference.length_m
    s_all = _s_grid(0.0, length)
    if lane_drop is not None:
        s_drop0, s_drop1 = lane_drop[0], lane_drop[0] + lane_drop[1]
        if not (0.0 < s_drop0 < s_drop1 <= length) or n < 2:
            raise ValueError("invalid lane_drop")
        shrink = lambda s: w * _smoothstep(s, s_drop0, s_drop1)          # noqa: E731
    else:
        s_drop0 = s_drop1 = None
        shrink = lambda s: np.zeros_like(np.asarray(s, float))            # noqa: E731

    B = BoundaryType

    def divider_spans(k: int, forward: bool, from_dropping_lane: bool):
        end = s_drop0 if (lane_drop is not None and k == n - 1) else length
        split = min(max(solid_divider_until_s_m, 0.0), end) if forward else 0.0
        spans = []
        if split > 0.0:
            spans.append((0.0, split, B.SOLID))
        if split < end:
            spans.append((split, end, B.DASHED))
        if end < length:
            spans.append((end, s_drop1, B.MERGE if (from_dropping_lane and forward) else B.NONE))
            if not from_dropping_lane and s_drop1 < length:
                spans.append((s_drop1, length, B.EDGE))
        return spans

    def boundaries(i: int, forward: bool):
        left = [(0.0, length, B.DOUBLE_YELLOW)] if i == 1 else divider_spans(i - 1, forward, i == n)
        right = [(0.0, length, B.EDGE)] if i == n else divider_spans(i, forward, False)
        return left, right

    def to_lane_s(spans, s_ref_grid, lane_path, reverse):
        lane_s = lane_path.s_m[::-1] if reverse else lane_path.s_m
        out = []
        for a, b, kind in spans:
            a, b = (float(np.interp(v, s_ref_grid, lane_s)) for v in (a, b))
            if abs(b - a) > 1e-9:
                out.append((min(a, b), max(a, b), kind))
        return tuple(sorted(out))

    lanes: Dict[str, Lane] = {}
    for i in range(1, n + 1):
        dropped = lane_drop is not None and i == n
        s_lane = _s_grid(0.0, s_drop1) if dropped else s_all
        d = (i - 0.5) * w - (shrink(s_lane) if dropped else 0.0)
        fwd = _offset_profile(reference, s_lane, d)
        opp = _offset_profile(reference, s_lane, -d)[::-1]
        fwd_path, opp_path = ReferencePath.from_xy(fwd), ReferencePath.from_xy(opp)
        fl, fr = boundaries(i, True)
        ol, or_ = boundaries(i, False)
        lanes[f"{name}:F{i}"] = Lane(
            f"{name}:F{i}", name, fwd_path, w,
            left_lane_id=f"{name}:F{i - 1}" if i > 1 else None,
            right_lane_id=f"{name}:F{i + 1}" if i < n else None,
            left_boundary=to_lane_s(fl, s_lane, fwd_path, False),
            right_boundary=to_lane_s(fr, s_lane, fwd_path, False))
        lanes[f"{name}:O{i}"] = Lane(
            f"{name}:O{i}", name, opp_path, w,
            left_lane_id=f"{name}:O{i - 1}" if i > 1 else None,
            right_lane_id=f"{name}:O{i + 1}" if i < n else None,
            left_boundary=to_lane_s(ol, s_lane, opp_path, True),
            right_boundary=to_lane_s(or_, s_lane, opp_path, True))
    markings = [LaneMarking(MarkingStyle.DOUBLE_YELLOW, _offset_polyline(reference, 0.0))]
    for k in range(1, n):
        divider_end = s_drop0 if (lane_drop is not None and k == n - 1) else length
        for sign in (1.0, -1.0):
            d = sign * k * w
            split = min(max(solid_divider_until_s_m, 0.0), divider_end)
            if sign > 0 and split > 0.0:
                markings.append(LaneMarking(MarkingStyle.SOLID_WHITE, _offset_polyline(reference, d, 0.0, split)))
                if split < divider_end:
                    markings.append(LaneMarking(MarkingStyle.DASHED_WHITE,
                                                _offset_polyline(reference, d, split, divider_end)))
            else:
                markings.append(LaneMarking(MarkingStyle.DASHED_WHITE,
                                            _offset_polyline(reference, d, 0.0, divider_end)))
    edge = n * w - shrink(s_all)
    for sign in (1.0, -1.0):
        markings.append(LaneMarking(MarkingStyle.ROAD_EDGE, _offset_profile(reference, s_all, sign * edge)))
    half = edge + 0.4
    surface = np.vstack((_offset_profile(reference, s_all, -half), _offset_profile(reference, s_all, half)[::-1]))

    sidewalks, crosswalks = [], []
    if sidewalk_width_m > 0.0:
        off = half + 0.3 + sidewalk_width_m / 2.0
        for side, sign in (("R", 1.0), ("L", -1.0)):
            sidewalks.append(Sidewalk(f"{name}:SW{side}", _offset_profile(reference, s_all, sign * off),
                                      sidewalk_width_m))
    for j, s_cw in enumerate(crosswalk_s_m):
        e = n * w - float(shrink(s_cw))
        corners = [(s_cw - 1.5, -e), (s_cw - 1.5, e), (s_cw + 1.5, e), (s_cw + 1.5, -e)]
        xy = np.array([frenet_to_cartesian(reference, a, b) for a, b in corners], dtype=float)
        crosswalks.append(Crosswalk(f"{name}:CW{j}", xy))
    return RoadNetwork(name, name, {name: RoadReference(name, reference)}, lanes,
                       tuple(markings), (surface,), crosswalks=tuple(crosswalks), sidewalks=tuple(sidewalks))


# ---------------------------------------------------------------------------
# Four-way intersection
# ---------------------------------------------------------------------------
def _quarter_turn(span_m: float, sign: float, clothoid_rad: float = 0.7) -> np.ndarray:
    """Clothoid-arc-clothoid 90 deg turn from the origin heading north, ending
    ``span_m`` ahead and ``sign * span_m`` to the right."""
    arc_rad = math.pi / 2.0 - clothoid_rad
    unit = [CurvatureSegment(clothoid_rad, 0.0, sign), CurvatureSegment(arc_rad, sign, sign),
            CurvatureSegment(clothoid_rad, sign, 0.0)]
    x, _, _ = integrate_curvature(unit, step_m=1e-4)
    scale = span_m / x[-1]
    k = sign / scale
    segs = [CurvatureSegment(clothoid_rad * scale, 0.0, k), CurvatureSegment(arc_rad * scale, k, k),
            CurvatureSegment(clothoid_rad * scale, k, 0.0)]
    x, y, _ = integrate_curvature(segs, step_m=0.01)
    pts = _resample(x, y)
    pts[-1] = (span_m, sign * span_m)
    return pts


def junction(name: str, prefix: str, center_xy: Tuple[float, float], heading_rad: float,
             lanes_per_direction: int, arm_extents_m: Sequence[float],
             lane_width_m: float = LANE_WIDTH_M, junction_half_m: Optional[float] = None,
             control: Optional[str] = None, sidewalk_width_m: float = 0.0) -> RoadNetwork:
    """Four-way junction (see ``four_way_intersection``).  ``control``:
    None (uncontrolled), "stop_signs" or "signals"."""
    if control not in (None, "stop_signs", "signals"):
        raise ValueError("control must be None, 'stop_signs' or 'signals'")
    n, w = lanes_per_direction, lane_width_m
    H = junction_half_m if junction_half_m is not None else n * w + 8.5 - 1.5 * (n - 1)
    legacy = prefix == "" and n == 1
    x_stop = -(H + 5.0)

    def lane_in(k, i):
        return f"A{k}:in" if legacy else f"{prefix}A{k}:in{i}"

    def lane_out(k, i):
        return f"A{k}:out" if legacy else f"{prefix}A{k}:out{i}"

    def stop_id(k, i):
        if legacy:
            return f"SL_A{k}"
        return f"{prefix}SL_A{k}" if n == 1 else f"{prefix}SL_A{k}_{i}"

    def to_global(pts, k):
        local = _rotate(pts, k * math.pi / 2.0)
        return _rotate(local, heading_rad) + np.asarray(center_xy, float)

    def line(x_from, x_to, y):
        m = max(2, int(round(abs(x_to - x_from) / SAMPLE_M)) + 1)
        return np.column_stack((np.linspace(x_from, x_to, m), np.full(m, y)))

    lanes: Dict[str, Lane] = {}
    roads: Dict[str, RoadReference] = {}
    markings, stop_lines, crosswalks, signs, signals, sidewalks = [], {}, [], [], {}, []
    exit_arm = {"S": 2, "L": 1, "R": 3}
    movement_of = {"S": "STRAIGHT", "L": "LEFT", "R": "RIGHT"}
    jid = prefix.rstrip(":") or name
    B = BoundaryType
    for k in range(4):
        L = float(arm_extents_m[k])
        arm = f"{prefix}A{k}"
        roads[arm] = RoadReference(arm, ReferencePath.from_xy(to_global(line(-L, -H, 0.0), k)))
        s_stop_in, s_marked_out = x_stop + L, -H - x_stop
        approach = L - H
        for i in range(1, n + 1):
            y = (i - 0.5) * w
            in_left = ((0.0, s_stop_in, B.DOUBLE_YELLOW if i == 1 else B.DASHED), (s_stop_in, approach, B.NONE))
            in_right = ((0.0, approach, B.EDGE),) if i == n else (
                (0.0, s_stop_in, B.DASHED), (s_stop_in, approach, B.NONE))
            out_left = ((0.0, s_marked_out, B.NONE), (s_marked_out, approach, B.DOUBLE_YELLOW if i == 1 else B.DASHED))
            out_right = ((0.0, approach, B.EDGE),) if i == n else (
                (0.0, s_marked_out, B.NONE), (s_marked_out, approach, B.DASHED))
            lanes[lane_in(k, i)] = Lane(
                lane_in(k, i), arm, ReferencePath.from_xy(to_global(line(-L, -H, y), k)), w,
                left_lane_id=lane_in(k, i - 1) if i > 1 else None,
                right_lane_id=lane_in(k, i + 1) if i < n else None,
                left_boundary=in_left, right_boundary=in_right)
            lanes[lane_out(k, i)] = Lane(
                lane_out(k, i), arm, ReferencePath.from_xy(to_global(line(-H, -L, -y), k)), w,
                left_lane_id=lane_out(k, i - 1) if i > 1 else None,
                right_lane_id=lane_out(k, i + 1) if i < n else None,
                left_boundary=out_left, right_boundary=out_right)
            j = (k + 2) % 4
            sid = f"A{k}>A{j}:S" if legacy else f"{prefix}A{k}>A{j}:S{i}"
            path = ReferencePath.from_xy(to_global(line(-H, H, y), k))
            lanes[sid] = Lane(sid, f"{prefix}junction", path, w,
                              left_boundary=((0.0, path.length_m, B.NONE),),
                              right_boundary=((0.0, path.length_m, B.NONE),),
                              junction_id=jid, movement="STRAIGHT")
        y_out = (n - 0.5) * w
        right = _quarter_turn(H - y_out, +1.0) + np.array([-H, y_out])
        left = _quarter_turn(H + 0.5 * w, -1.0) + np.array([-H, 0.5 * w])
        for kind, pts in (("R", right), ("L", left)):
            j = (k + exit_arm[kind]) % 4
            cid = f"{prefix}A{k}>A{j}:{kind}"
            path = ReferencePath.from_xy(to_global(pts, k))
            lanes[cid] = Lane(cid, f"{prefix}junction", path, w,
                              left_boundary=((0.0, path.length_m, B.NONE),),
                              right_boundary=((0.0, path.length_m, B.NONE),),
                              junction_id=jid, movement=movement_of[kind])

        markings.append(LaneMarking(MarkingStyle.DOUBLE_YELLOW, to_global(line(-L, x_stop, 0.0), k)))
        for m in range(1, n):
            for sign in (1.0, -1.0):
                markings.append(LaneMarking(MarkingStyle.DASHED_WHITE, to_global(line(-L, x_stop, sign * m * w), k)))
        for y in (-n * w, n * w):
            markings.append(LaneMarking(MarkingStyle.ROAD_EDGE, to_global(line(-L, -H, y), k)))
        markings.append(LaneMarking(MarkingStyle.ROAD_EDGE, to_global(_arc((-H, H), (-H, n * w), (-n * w, H)), k)))
        for i in range(1, n + 1):
            p0, p1 = to_global([(x_stop, (i - 1) * w), (x_stop, i * w)], k)
            stop_lines[stop_id(k, i)] = StopLine(stop_id(k, i), lane_in(k, i), tuple(p0), tuple(p1), x_stop + L)
        sig_id = f"{prefix}SIG_A{k}"
        crosswalks.append(Crosswalk(f"{prefix}CW_A{k}", to_global([(-(H + 4.0), -n * w), (-(H + 4.0), n * w),
                                                                     (-(H + 1.0), n * w), (-(H + 1.0), -n * w)], k),
                                    signal_id=sig_id if control == "signals" else None))
        if control == "stop_signs":
            signs.append(StopSign(f"{prefix}STOP_A{k}", tuple(to_global([(x_stop - 0.5, n * w + 1.6)], k)[0]),
                                  stop_id(k, 1)))
        elif control == "signals":
            signals[sig_id] = TrafficSignal(sig_id, tuple(to_global([(-(H + 1.0), n * w + 1.6)], k)[0]),
                                            stop_id(k, 1))
        if sidewalk_width_m > 0.0:
            off = n * w + 0.7 + sidewalk_width_m / 2.0
            r = H - n * w
            for side, sign in (("R", 1.0), ("L", -1.0)):
                sidewalks.append(Sidewalk(f"{arm}:SW{side}", to_global(line(-L, -H, sign * off), k), sidewalk_width_m))
            r_sw = max(r - 0.7 - sidewalk_width_m / 2.0, 0.5)
            corner = _arc((-H, H), (-H, H - r_sw), (-(H - r_sw), H))
            sidewalks.append(Sidewalk(f"{arm}:SWC", to_global(corner, k), sidewalk_width_m))

    polygon = []
    for k in (0, 3, 2, 1):
        L = float(arm_extents_m[k])
        polygon.append(np.vstack((
            to_global([(-L, -n * w), (-L, n * w), (-H, n * w)], k),
            to_global(_arc((-H, H), (-H, n * w), (-n * w, H)), k)[1:],
        )))
    surface = np.vstack(polygon)
    box = to_global([(-H, -H), (-H, H), (H, H), (H, -H)], 0)
    return RoadNetwork(name, f"{prefix}A0", roads, lanes, tuple(markings), (surface,), stop_lines,
                       tuple(crosswalks), tuple(signs), signals, sidewalks=tuple(sidewalks),
                       junction_boxes={jid: box})


# ---------------------------------------------------------------------------
# Ready-made roads (used by scenarios through scenarios/scenario_tools.py)
# ---------------------------------------------------------------------------
def straight_reference(length_m: float = 600.0) -> ReferencePath:
    return reference_from_curvature([CurvatureSegment(length_m, 0.0, 0.0)])


def s_curve_reference(radius_m: float = 100.0) -> ReferencePath:
    """Gentle left bend, transition, gentle right bend, then straight."""
    k = 1.0 / radius_m
    return reference_from_curvature([
        CurvatureSegment(40.0, 0.0, 0.0), CurvatureSegment(25.0, 0.0, -k),
        CurvatureSegment(50.0, -k, -k), CurvatureSegment(50.0, -k, k),
        CurvatureSegment(50.0, k, k), CurvatureSegment(25.0, k, 0.0),
        CurvatureSegment(260.0, 0.0, 0.0)])


def four_way_intersection(name: str = "intersection", arm_length_m: float = 150.0,
                          lane_width_m: float = LANE_WIDTH_M, junction_half_m: float = 12.0,
                          control: Optional[str] = None) -> RoadNetwork:
    """Single junction at the origin: arms A0 (south), A1 (west), A2 (north),
    A3 (east), one lane per direction.  ``control``: None, "stop_signs" or
    "signals"."""
    return junction(name, "", (0.0, 0.0), 0.0, 1, (arm_length_m,) * 4, lane_width_m,
                    junction_half_m, control)
