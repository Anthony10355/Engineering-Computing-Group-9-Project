"""OpenCV visualization (display only -- nothing here affects the simulation).

Ported from the original research simulator with the same colours, scale and
layout.  Three views (key ``v`` cycles them):

``local_bev`` (default) -- top-down, EGO-FIXED, forward up, 120 m x 80 m at
    6 px/m.  The ego stays at the same place on screen; the road and other
    road users move around it.  Shows the planned trajectory with a dot every
    0.5 s (dots shrink with time, labels 1s .. 4s) and a red bar where the
    plan comes to a standstill.
``global_map`` -- north-up overview of the whole map (3 px/m); the ego moves.
``both``       -- the two side by side.

``debug`` (key ``d``) adds: the lane centre lines the planner received, the
alternative trajectories the planner reported, a 10 m grid, the pure-pursuit
goal point and the true footprints of all road users.
"""

from __future__ import annotations

import math
from typing import Callable, Optional

import cv2
import numpy as np

from simulator.geometry import box_corners, boxes_overlap_many
from simulator.scenario_types import KIND_TO_CLASS, PEDESTRIAN
from simulator.simulation import DisplayFrame
from simulator.vehicle_model import EGO_GEOMETRY


# ---------------------------------------------------------------------------
# Layout and colours (identical to the original)
# ---------------------------------------------------------------------------
ROWS, COLS = 120, 80                       # local view: 120 m x 80 m
EGO_ROW, EGO_COL = 80, 40                  # ego position in the local view (metres from the top-left)
SCALE = 6                                  # local view: px per metre
TOP, LEFT, GAP = 20, 20, 20
BEV_W, BEV_H = COLS * SCALE, ROWS * SCALE  # 480 x 720
HUD_W = 390
WORLD_SIZE = BEV_H
WORLD_W = WORLD_SIZE
WORLD_SCALE = 3.0                          # global view: px per metre
CANVAS_H = TOP + BEV_H + 20
SHIFT = 3
_SUB = 1 << SHIFT
VIEWS = ("local_bev", "global_map", "both")

C_BACKGROUND = (44, 52, 46)
C_SIDEWALK = (92, 96, 98)
C_ASPHALT = (70, 70, 72)
C_WHITE = (232, 232, 232)
C_EDGE = (200, 200, 200)
C_YELLOW = (40, 200, 245)
C_CROSSWALK = (190, 190, 190)
C_EGO = (40, 180, 255)                      # BGR: bright orange (cars are blue)
C_TRAJ_NEAR = np.array((80, 255, 120), float)      # BGR bright green
C_TRAJ_FAR = np.array((150, 200, 120), float)
C_REFERENCE = (230, 150, 40)
C_LOOKAHEAD = (60, 220, 255)
C_TRUTH = (170, 170, 255)
C_CONFLICT = (40, 40, 255)
C_TEXT = (235, 235, 235)
C_TEXT_DIM = (165, 170, 165)
C_PANEL = (30, 32, 34)
C_GOOD = (120, 210, 120)
C_ALT_VALID = (190, 190, 90)
C_ALT_REJECTED = (60, 110, 255)
SIGNAL_LAMPS = {"RED": (40, 40, 235), "YELLOW": (30, 200, 245), "GREEN": (70, 210, 70)}
DASH_ON_M, DASH_PERIOD_M = 3.0, 9.0

#: object class -> (name, BGR colour)  (original perception colours)
CLASS_STYLE = {1: ("person", (60, 70, 230)), 2: ("bicycle", (60, 135, 230)), 3: ("car", (230, 140, 60)),
               6: ("truck", (160, 95, 30))}
DEFAULT_STYLE = ("unknown", (150, 150, 150))

STATE_COLORS = {
    "KEEP_LANE": (150, 230, 150), "FOLLOW_LEAD": (120, 220, 255), "CHANGE_LEFT": (255, 200, 90),
    "CHANGE_RIGHT": (255, 200, 90), "TURN_LEFT": (230, 190, 255), "TURN_RIGHT": (230, 190, 255),
    "STOP_SIGNAL": (60, 60, 240), "STOP_SIGN": (60, 60, 240), "YIELD": (40, 160, 255),
    "EMERGENCY_STOP": (30, 30, 255), "FALLBACK_BRAKE": (30, 30, 255),
}

ToPx = Callable[[np.ndarray, np.ndarray], np.ndarray]


def ego_to_pixel(x_forward_m, y_right_m) -> np.ndarray:
    """Ego-frame metres -> local-view pixels (u right, v down)."""
    x = np.asarray(x_forward_m, dtype=float)
    y = np.asarray(y_right_m, dtype=float)
    return np.stack(((EGO_COL + y) * SCALE + SCALE / 2.0, (EGO_ROW - x) * SCALE + SCALE / 2.0), axis=-1)


_TEMPLATES: dict = {}


def _filled(h: int, w: int, color) -> np.ndarray:
    key = (h, w, tuple(color))
    if key not in _TEMPLATES:
        img = np.empty((h, w, 3), dtype=np.uint8)
        img[:] = color
        _TEMPLATES[key] = img
    return _TEMPLATES[key].copy()


def _fix(px: np.ndarray) -> np.ndarray:
    return (np.asarray(px) * _SUB).round().astype(np.int32)


def _pt(px) -> tuple:
    return tuple(int(v) for v in np.asarray(px).round())


def _polyline(img, pts_m, to_px: ToPx, color, thickness_px=1) -> None:
    if len(pts_m) < 2:
        return
    cv2.polylines(img, [_fix(to_px(pts_m[:, 0], pts_m[:, 1])).reshape(-1, 1, 2)], False, color,
                  max(1, int(round(thickness_px))), cv2.LINE_AA, SHIFT)


def _offset(pts: np.ndarray, d: float) -> np.ndarray:
    tangent = np.gradient(pts, axis=0)
    norm = np.maximum(np.hypot(tangent[:, 0], tangent[:, 1]), 1e-9)
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0])) / norm[:, None]
    return pts + d * normal


def _draw_marking(img, style: str, pts: np.ndarray, arc, to_px: ToPx, scale: float) -> None:
    thick = 2 if scale >= 5 else 1
    if style == "DOUBLE_YELLOW":
        if scale >= 5:
            _polyline(img, _offset(pts, -0.15), to_px, C_YELLOW, 2)
            _polyline(img, _offset(pts, 0.15), to_px, C_YELLOW, 2)
        else:
            _polyline(img, pts, to_px, C_YELLOW, 1)
    elif style == "DASHED_WHITE":
        arc = np.arange(len(pts)) * 0.5 if arc is None else arc
        idx = np.flatnonzero((arc % DASH_PERIOD_M) < DASH_ON_M)
        if idx.size:
            for run in np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1):
                if run.size >= 2:
                    _polyline(img, pts[run[0]:run[-1] + 1], to_px, C_WHITE, thick)
    elif style == "SOLID_WHITE":
        _polyline(img, pts, to_px, C_WHITE, thick)
    else:                                                  # ROAD_EDGE
        _polyline(img, pts, to_px, C_EDGE, thick)


def _draw_crosswalk(img, poly: np.ndarray, to_px: ToPx) -> None:
    c0, c1, _, c3 = poly
    across, along = c1 - c0, c3 - c0
    width = float(np.hypot(*across))
    for a in np.arange(0.3, width - 0.4, 1.0):
        p0 = c0 + across * (a / width)
        p1 = c0 + across * ((a + 0.55) / width)
        quad = np.array([p0, p1, p1 + along, p0 + along])
        cv2.fillPoly(img, [_fix(to_px(quad[:, 0], quad[:, 1]))], C_CROSSWALK, cv2.LINE_AA, SHIFT)


def _draw_stop_sign(img, px, r_px: float) -> None:
    ang = np.radians(22.5 + 45.0 * np.arange(8))
    octagon = np.column_stack((px[0] + r_px * np.cos(ang), px[1] + r_px * np.sin(ang)))
    pts = _fix(octagon)
    cv2.fillPoly(img, [pts], (40, 40, 210), cv2.LINE_AA, SHIFT)
    cv2.polylines(img, [pts], True, (245, 245, 245), 1, cv2.LINE_AA, SHIFT)


def _draw_signal(img, px, state: str, small: bool = False) -> None:
    u, v = _pt(px)
    w, h, r, step = (6, 15, 2, 5) if small else (9, 24, 3, 8)
    cv2.rectangle(img, (u - w // 2 - 1, v - h // 2 - 1), (u + w // 2 + 1, v + h // 2 + 1), (20, 20, 20), -1)
    for i, name in enumerate(("RED", "YELLOW", "GREEN")):
        color = SIGNAL_LAMPS[name] if state == name else (55, 55, 55)
        cv2.circle(img, (u, v - step + step * i), r, color, -1, cv2.LINE_AA)


def _draw_box(img, to_px: ToPx, x, y, heading, length, width, color, thickness=-1) -> None:
    corners = box_corners(x, y, heading, length, width)
    pts = _fix(to_px(corners[:, 0], corners[:, 1]))
    if thickness < 0:
        cv2.fillPoly(img, [pts], color, cv2.LINE_AA, SHIFT)
    else:
        cv2.polylines(img, [pts], True, color, thickness, cv2.LINE_AA, SHIFT)


def _draw_trajectory(img, x, y, t, to_px: ToPx, scale: float, labels: bool = False,
                     t_max: Optional[float] = None) -> None:
    """Thick near-term, thin and faded toward the horizon; a marker every
    0.5 s that SHRINKS with time (and a 1s / 2s / ... label)."""
    n = len(x)
    if n < 2:
        return
    frac = np.asarray(t) / max(float(t[-1]) if t_max is None else t_max, 1e-9)
    frac = np.clip(frac, 0.0, 1.0)
    colors = (1 - frac)[:, None] * C_TRAJ_NEAR + frac[:, None] * C_TRAJ_FAR
    pts = [tuple(p) for p in _fix(to_px(np.asarray(x), np.asarray(y))).tolist()]
    w0 = 6 if scale >= 5 else 4
    for k in range(n - 1):
        cv2.line(img, pts[k], pts[k + 1], tuple(colors[k].tolist()), max(1, int(round(w0 - (w0 - 2) * frac[k]))),
                 cv2.LINE_AA, SHIFT)
    if scale >= 5:
        last_px = None
        marks = _time_marks(np.asarray(t), 0.5)
        for k in marks:
            r = 4.5 - 2.5 * frac[k]
            cv2.circle(img, pts[k], int((r + 1.0) * _SUB), (30, 30, 30), -1, cv2.LINE_AA, SHIFT)
            cv2.circle(img, pts[k], int(r * _SUB), tuple(colors[k].tolist()), -1, cv2.LINE_AA, SHIFT)
            if labels and abs(t[k] - round(t[k])) < 1e-6 and t[k] > 0.5:
                u, v = pts[k][0] >> SHIFT, pts[k][1] >> SHIFT
                if last_px is None or abs(u - last_px[0]) + abs(v - last_px[1]) > 12:
                    cv2.putText(img, f"{t[k]:.0f}s", (u + 7, v + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                                (235, 255, 235), 1, cv2.LINE_AA)
                    last_px = (u, v)


def _time_marks(t: np.ndarray, every_s: float):
    """Indices of the samples closest to 0.5 s, 1.0 s, 1.5 s, ... (any time step)."""
    out = []
    for tm in np.arange(every_s, float(t[-1]) + 1e-9, every_s):
        k = int(np.argmin(np.abs(t - tm)))
        if abs(t[k] - tm) < 0.06 and k not in out:
            out.append(k)
    return out


def _visible_extent(traj, actors, margin_m: float = 0.1) -> int:
    """Samples to DRAW: up to the first one whose ego footprint would touch an
    actor's CURRENT footprint (the plan may legitimately pass where a moving
    actor is now; drawing that part over the actor is misleading)."""
    n = traj.num_points
    acts = [a for a in actors if math.hypot(a.x_m, a.y_m) < 80.0]
    if not acts:
        return n
    g = EGO_GEOMETRY
    h = np.asarray(traj.heading_rad, float)
    cx = traj.x_m + g.center_offset_m * np.cos(h)
    cy = traj.y_m + g.center_offset_m * np.sin(h)
    for a in acts:
        if np.min(np.hypot(traj.x_m - a.x_m, traj.y_m - a.y_m)) > 0.5 * (g.length_m + a.length_m) + 3.0:
            continue
        touch = np.flatnonzero(boxes_overlap_many(cx, cy, h, g.length_m + 2 * margin_m, g.width_m + 2 * margin_m,
                                                  a.x_m, a.y_m, a.heading_rad, a.length_m, a.width_m))
        if touch.size:
            n = min(n, int(touch[0]))
    return max(n, 1)


def _draw_stop_target(img, traj, to_px: ToPx) -> None:
    """Planned standstill: a red bar across the path where the plan stops."""
    v = np.asarray(traj.speed_mps)
    if v[-1] > 0.2:
        return
    k = int(np.argmax(v <= 0.2))
    x, y, h = float(traj.x_m[k]), float(traj.y_m[k]), float(traj.heading_rad[k])
    front = EGO_GEOMETRY.rear_axle_to_front_m
    fx, fy = x + front * math.cos(h), y + front * math.sin(h)
    nx, ny = -math.sin(h), math.cos(h)
    a = to_px(np.array([fx - 1.2 * nx]), np.array([fy - 1.2 * ny]))[0]
    b = to_px(np.array([fx + 1.2 * nx]), np.array([fy + 1.2 * ny]))[0]
    cv2.line(img, _pt(a), _pt(b), (40, 40, 235), 3, cv2.LINE_AA)


def _draw_agent(img, to_px: ToPx, scale: float, class_id, x, y, heading, length, width,
                vx=0.0, vy=0.0, highlight=False) -> None:
    _, color = CLASS_STYLE.get(class_id, DEFAULT_STYLE)
    p = to_px(np.array([x]), np.array([y]))[0]
    if class_id == PEDESTRIAN:
        r = max(3, int(round(0.45 * scale)))
        cv2.circle(img, _pt(p), r, color, -1, cv2.LINE_AA)
        cv2.circle(img, _pt(p), r, (25, 25, 25), 1, cv2.LINE_AA)
        if highlight:
            cv2.circle(img, _pt(p), r + 4, C_CONFLICT, 2, cv2.LINE_AA)
    else:
        _draw_box(img, to_px, x, y, heading, length, width, color)
        _draw_box(img, to_px, x, y, heading, length, width, (25, 25, 25), 1)
        if highlight:
            _draw_box(img, to_px, x, y, heading, length + 1.0, width + 1.0, C_CONFLICT, 2)
    if math.hypot(vx, vy) > 0.3 and scale >= 5:
        tip = to_px(np.array([x + vx]), np.array([y + vy]))[0]            # 1 s of motion
        cv2.arrowedLine(img, _pt(p), _pt(tip), (240, 240, 240), 1, cv2.LINE_AA, 0, 0.3)


def _dashed(img, pts: np.ndarray, to_px: ToPx, color, thickness=1, on=4, off=4) -> None:
    for k in range(0, len(pts) - 1, on + off):
        _polyline(img, pts[k:k + on + 1], to_px, color, thickness)


def _draw_ego(img, to_px: ToPx, x, y, heading) -> None:
    g = EGO_GEOMETRY
    cx, cy = x + g.center_offset_m * math.cos(heading), y + g.center_offset_m * math.sin(heading)
    _draw_box(img, to_px, cx, cy, heading, g.length_m, g.width_m, C_EGO)
    _draw_box(img, to_px, cx, cy, heading, g.length_m, g.width_m, (20, 20, 20), 1)


def _draw_static_scene(img, scene_like, to_px: ToPx, scale: float) -> None:
    """Sidewalks, asphalt, crosswalks, markings, stop lines."""
    for pts, width in scene_like["sidewalks"]:
        _polyline(img, pts, to_px, C_SIDEWALK, width * scale)
    for poly in scene_like["surfaces"]:
        cv2.fillPoly(img, [to_px(poly[:, 0], poly[:, 1]).round().astype(np.int32)], C_ASPHALT)
    for cw in scene_like["crosswalks"]:
        _draw_crosswalk(img, cw, to_px)
    for style, pts, arc in scene_like["markings"]:
        _draw_marking(img, style, pts, arc, to_px, scale)
    for line in scene_like["stop_lines"]:
        _polyline(img, line, to_px, C_WHITE, max(2, 0.6 * scale))


def _draw_state_label(img, traj, desired_speed_mps: float) -> None:
    """Behaviour label box in the top-left corner of the local view."""
    color = STATE_COLORS.get(traj.behavior, C_TEXT)
    text = traj.behavior.replace("_", " ")
    detail = traj.notes[0][:46] if traj.notes else ""
    target = f"target {desired_speed_mps:.1f} m/s  plan +{traj.t_s[-1]:.0f} s {traj.terminal_speed_mps:.1f} m/s"
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
    (w2, _), _ = cv2.getTextSize(detail, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
    (w3, _), _ = cv2.getTextSize(target, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
    cv2.rectangle(img, (6, 26), (16 + max(w, w2, w3), 30 + h + (40 if detail else 26)), (25, 25, 25), -1)
    cv2.putText(img, text, (11, 28 + h), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
    y = 48 + h
    if detail:
        cv2.putText(img, detail, (11, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_TEXT, 1, cv2.LINE_AA)
        y += 18
    cv2.putText(img, target, (11, y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT_DIM, 1, cv2.LINE_AA)


def _wrap_text(s: str, width: int):
    words, line, out = s.split(), "", []
    for w in words:
        if len(line) + len(w) + 1 > width:
            out.append(line)
            line = w
        else:
            line = f"{line} {w}".strip()
    if line:
        out.append(line)
    return out


# ---------------------------------------------------------------------------
# Global map: static layers drawn once per road network
# ---------------------------------------------------------------------------
class _WorldMap:
    def __init__(self, network) -> None:
        pts = [np.vstack(network.surfaces)] + [sw.centerline_m for sw in network.sidewalks]
        allp = np.vstack(pts)
        margin = WORLD_SIZE / WORLD_SCALE / 2.0 + 20.0
        self.x_max = float(allp[:, 0].max()) + margin
        self.y_min = float(allp[:, 1].min()) - margin
        x_min = float(allp[:, 0].min()) - margin
        y_max = float(allp[:, 1].max()) + margin
        h = int(math.ceil((self.x_max - x_min) * WORLD_SCALE))
        w = int(math.ceil((y_max - self.y_min) * WORLD_SCALE))
        self.image = _filled(h, w, C_BACKGROUND)
        scene = {
            "sidewalks": [(sw.centerline_m, sw.width_m) for sw in network.sidewalks],
            "surfaces": list(network.surfaces),
            "crosswalks": [cw.polygon_m for cw in network.crosswalks],
            "markings": [(m.style.value, m.points_m, m.arc_m) for m in network.markings],
            "stop_lines": [np.array([sl.p0_m, sl.p1_m]) for sl in network.stop_lines.values()],
        }
        _draw_static_scene(self.image, scene, self.to_px, WORLD_SCALE)
        for sign in network.stop_signs:
            _draw_stop_sign(self.image, self.to_px(np.array([sign.position_m[0]]), np.array([sign.position_m[1]]))[0],
                            1.3 * WORLD_SCALE + 1)
        inset_w = 160
        self.inset_scale = inset_w / w
        self.inset = cv2.resize(self.image, (inset_w, max(1, int(h * self.inset_scale))), interpolation=cv2.INTER_AREA)

    def to_px(self, x, y) -> np.ndarray:
        x, y = np.asarray(x, float), np.asarray(y, float)
        return np.stack(((y - self.y_min) * WORLD_SCALE, (self.x_max - x) * WORLD_SCALE), axis=-1)


# ---------------------------------------------------------------------------
# Renderer
# ---------------------------------------------------------------------------
class Renderer:
    def __init__(self, debug: bool = False, view: str = "local_bev") -> None:
        if view not in VIEWS:
            raise ValueError(f"view must be one of {VIEWS}")
        self.view = view
        self.debug = debug
        self._maps: dict = {}

    @property
    def canvas_width(self) -> int:
        panels = {"local_bev": BEV_W, "global_map": WORLD_W, "both": BEV_W + GAP + WORLD_W}[self.view]
        return LEFT + panels + GAP + HUD_W

    # ------------------------------------------------------------------
    def bev_panel(self, frame: DisplayFrame) -> np.ndarray:
        """EGO-centred local view (ego fixed, world moves)."""
        obs, traj, cmd, scene = frame.observation, frame.trajectory, frame.command, frame.scene
        to_px: ToPx = ego_to_pixel
        img = _filled(BEV_H, BEV_W, C_BACKGROUND)
        _draw_static_scene(img, {"sidewalks": scene.sidewalks, "surfaces": scene.surfaces,
                                 "crosswalks": scene.crosswalks, "markings": scene.markings,
                                 "stop_lines": scene.stop_lines}, to_px, SCALE)
        if self.debug:
            for k in range(0, ROWS + 1, 10):
                cv2.line(img, (0, k * SCALE), (BEV_W, k * SCALE), (80, 90, 82), 1)
            for k in range(0, COLS + 1, 10):
                cv2.line(img, (k * SCALE, 0), (k * SCALE, BEV_H), (80, 90, 82), 1)
            for lane in obs.lanes.values():
                _polyline(img, lane.centerline.xy_m, to_px, C_REFERENCE, 2 if lane.name == "current" else 1)
            for a in scene.actors:
                _draw_box(img, to_px, a.x_forward_m, a.y_right_m, a.heading_rad, a.length_m, a.width_m, C_TRUTH, 1)
            for alt in traj.alternatives:
                xy = np.column_stack((alt.x_m, alt.y_m))
                if alt.rejected:
                    _dashed(img, xy, to_px, C_ALT_REJECTED, 1)
                else:
                    _polyline(img, xy, to_px, C_ALT_VALID, 1)

        n_vis = _visible_extent(traj, obs.actors)
        if self.debug and n_vis < traj.num_points:
            _dashed(img, np.column_stack((traj.x_m[n_vis - 1:], traj.y_m[n_vis - 1:])), to_px, (150, 200, 120), 1)
        _draw_trajectory(img, traj.x_m[:n_vis], traj.y_m[:n_vis], traj.t_s[:n_vis], to_px, SCALE,
                         labels=True, t_max=float(traj.t_s[-1]))
        if n_vis == traj.num_points:
            _draw_stop_target(img, traj, to_px)
        for a in obs.actors:
            _draw_agent(img, to_px, SCALE, KIND_TO_CLASS.get(a.kind, 3), a.x_m, a.y_m, a.heading_rad,
                        a.length_m, a.width_m, a.vx_mps, a.vy_mps)
        for x, y in scene.stop_signs:
            _draw_stop_sign(img, to_px(np.array([x]), np.array([y]))[0], 1.4 * SCALE)
        for x, y, state in scene.signals:
            _draw_signal(img, to_px(np.array([x]), np.array([y]))[0], state)
        _draw_ego(img, to_px, 0.0, 0.0, 0.0)
        if self.debug:
            look = to_px(np.array([cmd.lookahead_point_xy_m[0]]), np.array([cmd.lookahead_point_xy_m[1]]))[0]
            cv2.circle(img, _pt(look), 5, C_LOOKAHEAD, 2, cv2.LINE_AA)
            cv2.line(img, _pt(to_px(np.zeros(1), np.zeros(1))[0]), _pt(look), C_LOOKAHEAD, 1, cv2.LINE_AA)
        _draw_state_label(img, traj, obs.desired_speed_mps)
        return img

    def world_panel(self, frame: DisplayFrame) -> np.ndarray:
        """North-up overview (world fixed, ego moves)."""
        view = frame.global_view
        key = id(view.network)
        if key not in self._maps:
            self._maps[key] = _WorldMap(view.network)
        wm = self._maps[key]
        ex, ey, eyaw = view.ego_pose
        cu, cv = wm.to_px(np.array([ex]), np.array([ey]))[0]
        u0, v0 = int(round(cu)) - WORLD_SIZE // 2, int(round(cv)) - WORLD_SIZE // 2
        img = _filled(WORLD_SIZE, WORLD_SIZE, C_BACKGROUND)
        h, w = wm.image.shape[:2]
        su0, sv0 = max(u0, 0), max(v0, 0)
        su1, sv1 = min(u0 + WORLD_SIZE, w), min(v0 + WORLD_SIZE, h)
        if su1 > su0 and sv1 > sv0:
            img[sv0 - v0:sv1 - v0, su0 - u0:su1 - u0] = wm.image[sv0:sv1, su0:su1]
        offset = np.array([u0, v0], float)
        to_px: ToPx = lambda x, y: wm.to_px(x, y) - offset            # noqa: E731

        for x, y, state in view.signals:
            _draw_signal(img, to_px(np.array([x]), np.array([y]))[0], state, small=True)
        if len(view.ego_history_xy) > 1:
            _polyline(img, np.asarray(view.ego_history_xy), to_px, (80, 255, 120), 2)
        for _, class_id, x, y, heading, length, width in view.actors:
            _draw_agent(img, to_px, WORLD_SCALE, class_id, x, y, heading, length, width)
        traj = frame.trajectory
        c, s = math.cos(eyaw), math.sin(eyaw)
        gx = ex + traj.x_m * c - traj.y_m * s
        gy = ey + traj.x_m * s + traj.y_m * c
        n_vis = _visible_extent(traj, frame.observation.actors)
        _draw_trajectory(img, gx[:n_vis], gy[:n_vis], traj.t_s[:n_vis], to_px, WORLD_SCALE, t_max=float(traj.t_s[-1]))
        _draw_ego(img, to_px, ex, ey, eyaw)
        inset = wm.inset.copy()
        ip = wm.to_px(np.array([ex]), np.array([ey]))[0] * wm.inset_scale
        hist = np.asarray(view.ego_history_xy)
        if len(hist) > 1:
            hp = wm.to_px(hist[:, 0], hist[:, 1]) * wm.inset_scale
            cv2.polylines(inset, [hp.round().astype(np.int32).reshape(-1, 1, 2)], False, (80, 255, 120), 1, cv2.LINE_AA)
        cv2.circle(inset, _pt(ip), 3, C_EGO, -1, cv2.LINE_AA)
        ih, iw = min(inset.shape[0], WORLD_SIZE - 40), min(inset.shape[1], WORLD_SIZE - 20)
        img[30:30 + ih, 10:10 + iw] = inset[:ih, :iw]
        cv2.rectangle(img, (9, 29), (10 + iw, 30 + ih), (150, 150, 150), 1)
        cv2.putText(img, "GLOBAL MAP  north up", (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TEXT, 1, cv2.LINE_AA)
        return img

    # ------------------------------------------------------------------
    def render(self, frame: DisplayFrame) -> np.ndarray:
        obs, traj, cmd, result = frame.observation, frame.trajectory, frame.command, frame.result
        canvas = _filled(CANVAS_H, self.canvas_width, C_PANEL)
        x_panel = LEFT
        if self.view in ("local_bev", "both"):
            canvas[TOP:TOP + BEV_H, x_panel:x_panel + BEV_W] = self.bev_panel(frame)
            cv2.rectangle(canvas, (x_panel - 1, TOP - 1), (x_panel + BEV_W, TOP + BEV_H), (110, 110, 110), 1)
            cv2.putText(canvas, "LOCAL BEV  ego-fixed, forward up (120 x 80 m)", (x_panel + 6, TOP + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TEXT, 1, cv2.LINE_AA)
            x_panel += BEV_W + GAP
        if self.view in ("global_map", "both"):
            canvas[TOP:TOP + WORLD_SIZE, x_panel:x_panel + WORLD_SIZE] = self.world_panel(frame)
            cv2.rectangle(canvas, (x_panel - 1, TOP - 1), (x_panel + WORLD_SIZE, TOP + WORLD_SIZE), (110, 110, 110), 1)
            x_panel += WORLD_SIZE + GAP

        x0 = x_panel
        y = TOP + 22

        def text(s, scale=0.5, color=C_TEXT, thick=1, dy=22):
            nonlocal y
            cv2.putText(canvas, s, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)
            y += dy

        text(frame.scenario_name, 0.62, C_TEXT, 2, 22)
        for chunk in _wrap_text(frame.scenario_description, 46)[:3]:
            text(chunk, 0.42, C_TEXT_DIM, 1, 18)
        text(f"planner: {frame.planner_name}", 0.45, (120, 230, 255), 1, 26)
        text(f"BEHAVIOR  {traj.behavior}", 0.6, STATE_COLORS.get(traj.behavior, C_TEXT), 2, 24)
        for note in traj.notes[:2]:
            text(note[:46], 0.45, C_TEXT_DIM, 1, 20)
        y += 6
        speed = obs.ego.speed_mps
        text(f"{speed:4.1f} m/s  ({speed * 3.6:4.1f} km/h)", 0.8, C_TEXT, 2, 30)
        rows = [("speed limit", f"{obs.speed_limit_mps:.1f} m/s ({obs.speed_limit_mps / 0.44704:.0f} mph)"),
                ("desired speed", f"{obs.desired_speed_mps:.1f} m/s"),
                (f"speed at +{traj.t_s[-1]:.0f} s (plan)", f"{traj.terminal_speed_mps:.1f} m/s"),
                ("acceleration cmd", f"{cmd.acceleration_mps2:+.2f} m/s2"),
                ("steering cmd", f"{math.degrees(cmd.steering_angle_rad):+.1f} deg"),
                ("observed actors", f"{len(obs.actors)}"),
                ("planner compute", f"{frame.planner_ms:.1f} ms")]
        if self.debug:
            rows += [("lookahead", f"{cmd.lookahead_distance_m:.1f} m"),
                     ("lane offset (d)", f"{obs.lanes['current'].ego_d_m:+.2f} m"),
                     ("distance to goal", f"{obs.distance_to_goal_m:.0f} m")]
        for label, value in rows:
            cv2.putText(canvas, label, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TEXT_DIM, 1, cv2.LINE_AA)
            cv2.putText(canvas, value, (x0 + 175, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TEXT, 1, cv2.LINE_AA)
            y += 21
        y += 10
        text("EVALUATION (independent of the planner)", 0.45, C_TEXT, 1, 22)
        eval_rows = [("progress", f"{100.0 * result.progress:.0f} %", C_TEXT),
                     ("collision", "YES" if result.collision else "no", C_CONFLICT if result.collision else C_GOOD),
                     ("road departure", "YES" if result.road_departure else "no",
                      C_CONFLICT if result.road_departure else C_GOOD),
                     ("goal reached", "yes" if result.completed else "not yet", C_GOOD if result.completed else C_TEXT),
                     ("planner errors", f"{result.planner_errors}", C_CONFLICT if result.planner_errors else C_TEXT),
                     ("rule violations", result.violation_summary[:26],
                      C_TEXT if result.violation_summary == "none" else C_CONFLICT),
                     ("score so far", f"{result.score:.1f} / 100", C_TEXT)]
        for label, value, color in eval_rows:
            cv2.putText(canvas, label, (x0, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TEXT_DIM, 1, cv2.LINE_AA)
            cv2.putText(canvas, value, (x0 + 175, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            y += 21
        if frame.planner_error:
            for chunk in _wrap_text(frame.planner_error, 50)[:3]:
                text(chunk, 0.38, C_CONFLICT, 1, 16)
        if result.end_reason != "running":
            text(f"END: {result.end_reason}", 0.55, C_CONFLICT if result.score == 0 else C_GOOD, 2, 24)
        if self.debug and traj.notes[2:]:
            for note in traj.notes[2:8]:
                for chunk in _wrap_text(note, 52)[:2]:
                    text(chunk, 0.36, C_TEXT_DIM, 1, 15)
        yb = TOP + BEV_H - 70
        cv2.line(canvas, (x0, yb), (x0 + 40, yb), tuple(int(v) for v in C_TRAJ_NEAR), 5, cv2.LINE_AA)
        cv2.line(canvas, (x0 + 40, yb), (x0 + 80, yb), tuple(int(v) for v in C_TRAJ_FAR), 2, cv2.LINE_AA)
        cv2.putText(canvas, "planned trajectory, dot every 0.5 s", (x0 + 88, yb + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, C_TEXT_DIM, 1, cv2.LINE_AA)
        cv2.putText(canvas, f"t = {frame.time_s:.2f} s   view {self.view}", (x0, yb + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TEXT_DIM, 1, cv2.LINE_AA)
        cv2.putText(canvas, "q/ESC quit  p pause  d debug  v cycle view", (x0, yb + 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_TEXT_DIM, 1, cv2.LINE_AA)
        return canvas
