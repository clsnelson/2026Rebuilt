# pylint: skip-file
"""
Inspired by https://github.com/hammerheads5000/FuelSim v1.0.3
"""

import math
import random
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Callable, ClassVar, Optional

import numpy as np
from pykit.logger import Logger
from wpimath.geometry import (
    Pose2d, Pose3d, Rotation2d, Rotation3d, Transform3d,
    Translation2d, Translation3d,
)
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import (
    kilograms, kilograms_per_cubic_meter, meters,
    meters_per_second, meters_per_second_squared, radians, seconds,
)

# Constants
_PERIOD: seconds = 0.02
_GRAVITY: meters_per_second_squared = -9.81
_AIR_DENSITY: kilograms_per_cubic_meter = 1.2041
_FIELD_COR = math.sqrt(22 / 51.5)
_FUEL_COR = 0.5
_NET_COR = 0.2
_ROBOT_COR = 0.1
_FUEL_RADIUS: meters = 0.075
_FIELD_LENGTH: meters = 16.51
_FIELD_WIDTH: meters = 8.04
_TRENCH_WIDTH: meters = 1.265
_TRENCH_BLOCK_WIDTH: meters = 0.305
_TRENCH_HEIGHT: meters = 0.565
_TRENCH_BAR_HEIGHT: meters = 0.102
_TRENCH_BAR_WIDTH: meters = 0.152
_FRICTION = 0.1
_FUEL_MASS: kilograms = 0.448 * 0.45392
_FUEL_CROSS_AREA = math.pi * _FUEL_RADIUS ** 2
_DRAG_COF = 0.47
_DRAG_FORCE_FACTOR = 0.5 * _AIR_DENSITY * _DRAG_COF * _FUEL_CROSS_AREA

# Pre-computed stuff
_DRAG_OVER_MASS = _DRAG_FORCE_FACTOR / _FUEL_MASS
_FIELD_COR1 = 1.0 + _FIELD_COR
_FUEL_COR1 = 1.0 + _FUEL_COR
_ROBOT_COR1 = 1.0 + _ROBOT_COR
_FUEL_DIAM = _FUEL_RADIUS * 2.0
_FUEL_DIAM_SQ = _FUEL_DIAM ** 2

# Line checks
_FIELD_XZ_LINES: tuple[tuple[Translation3d, Translation3d], ...] = (
    (Translation3d(0, 0, 0), Translation3d(_FIELD_LENGTH, _FIELD_WIDTH, 0)),
    (Translation3d(3.96, 1.57, 0),
     Translation3d(4.61, _FIELD_WIDTH / 2 - 0.60, 0.165)),
    (Translation3d(3.96, _FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(4.61, _FIELD_WIDTH - 1.57, 0.165)),
    (Translation3d(4.61, 1.57, 0.165),
     Translation3d(5.18, _FIELD_WIDTH / 2 - 0.60, 0)),
    (Translation3d(4.61, _FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(5.18, _FIELD_WIDTH - 1.57, 0)),
    (Translation3d(_FIELD_LENGTH - 5.18, 1.57, 0),
     Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2 - 0.60, 0.165)),
    (Translation3d(_FIELD_LENGTH - 5.18, _FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH - 1.57, 0.165)),
    (Translation3d(_FIELD_LENGTH - 4.61, 1.57, 0.165),
     Translation3d(_FIELD_LENGTH - 3.96, _FIELD_WIDTH / 2 - 0.60, 0)),
    (Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(_FIELD_LENGTH - 3.96, _FIELD_WIDTH - 1.57, 0)),
    (Translation3d(3.96, _TRENCH_WIDTH, _TRENCH_HEIGHT),
     Translation3d(5.18, _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT)),
    (Translation3d(3.96, _FIELD_WIDTH - 1.57, _TRENCH_HEIGHT),
     Translation3d(
         5.18,
         _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
         )),
    (Translation3d(_FIELD_LENGTH - 5.18, _TRENCH_WIDTH, _TRENCH_HEIGHT),
     Translation3d(
         _FIELD_LENGTH - 3.96,
         _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
         )),
    (Translation3d(_FIELD_LENGTH - 5.18, _FIELD_WIDTH - 1.57, _TRENCH_HEIGHT),
     Translation3d(
         _FIELD_LENGTH - 3.96,
         _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
         )),
    (Translation3d(
        4.61 - _TRENCH_BAR_WIDTH / 2,
        0,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
        ),
     Translation3d(
         4.61 + _TRENCH_BAR_WIDTH / 2,
         _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
         )),
    (Translation3d(
        4.61 - _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH - 1.57,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
        ),
     Translation3d(
         4.61 + _TRENCH_BAR_WIDTH / 2,
         _FIELD_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
         )),
    (Translation3d(
        _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
        0,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
        ),
     Translation3d(
         _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
         _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
         )),
    (Translation3d(
        _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH - 1.57,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
        ),
     Translation3d(
         _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
         _FIELD_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
         )),
)

# Convert lines to numpy array
_NP_LINES: list[tuple[np.ndarray, np.ndarray]] = [
    (np.array([s.x, s.y, s.z]), np.array([e.x, e.y, e.z]))
    for s, e in _FIELD_XZ_LINES
]

_LINE_BOUNDS: list[tuple[float, float, float, float]] = [
    (min(s.x, e.x), max(s.x, e.x), min(s.y, e.y), max(s.y, e.y))
    for s, e in _FIELD_XZ_LINES
]

# x0, x1, y0, y1, z0, z1
_TRENCH_RECTS: tuple[tuple[float, float, float, float, float, float], ...] = (
    (3.96, 5.18, _TRENCH_WIDTH, _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, 0.0,
     _TRENCH_HEIGHT),
    (3.96, 5.18, _FIELD_WIDTH - 1.57,
     _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH, 0.0, _TRENCH_HEIGHT),
    (_FIELD_LENGTH - 5.18, _FIELD_LENGTH - 3.96, _TRENCH_WIDTH,
     _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, 0.0, _TRENCH_HEIGHT),
    (_FIELD_LENGTH - 5.18, _FIELD_LENGTH - 3.96, _FIELD_WIDTH - 1.57,
     _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH, 0.0, _TRENCH_HEIGHT),
    (4.61 - _TRENCH_BAR_WIDTH / 2, 4.61 + _TRENCH_BAR_WIDTH / 2, 0.0,
     _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT,
     _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
    (4.61 - _TRENCH_BAR_WIDTH / 2, 4.61 + _TRENCH_BAR_WIDTH / 2,
     _FIELD_WIDTH - 1.57, _FIELD_WIDTH, _TRENCH_HEIGHT,
     _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
    (_FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
     _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2, 0.0,
     _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT,
     _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
    (_FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
     _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2, _FIELD_WIDTH - 1.57,
     _FIELD_WIDTH, _TRENCH_HEIGHT, _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
)

# Bounding box for the entire trench region — fast early-out
_TRENCH_X_MIN = 3.96 - _FUEL_RADIUS
_TRENCH_X_MAX = _FIELD_LENGTH - 3.96 + _FUEL_RADIUS
_TRENCH_X_LEFT_MAX = 5.18 + _FUEL_RADIUS
_TRENCH_X_RIGHT_MIN = _FIELD_LENGTH - 5.18 - _FUEL_RADIUS


@dataclass
class Hub:
    """Handles hub interactions with fuel and collisions."""
    center: Translation2d
    exit: Translation3d
    exit_vel_x_mult: int

    _score: int = field(default=0, init=False, repr=False)

    ENTRY_HEIGHT: ClassVar[float] = 1.83
    ENTRY_RADIUS: ClassVar[float] = 0.56
    _SIDE: ClassVar[float] = 1.2
    _NET_HEIGHT_MAX: ClassVar[float] = 3.057
    _NET_HEIGHT_MIN: ClassVar[float] = 1.5
    _NET_OFFSET: ClassVar[float] = _SIDE / 2 + 0.261
    _NET_WIDTH: ClassVar[float] = 1.484

    def __post_init__(self) -> None:
        # Cache plain floats so Hub methods avoid attribute lookups in hot path
        self.cx: float = self.center.x
        self.cy: float = self.center.y
        self.net_x: float = self.cx + self._NET_OFFSET * self.exit_vel_x_mult

    def handle_hub_interaction_np(
        self,
        pos: np.ndarray,  # (N, 3) view of all active fuel
        vel: np.ndarray,  # (N, 3)
        dt: float,
        indices: np.ndarray,  # which rows to check (near-hub subset)
        scored_flags: np.ndarray,  # (N,) bool output — set True when scored
    ) -> None:
        """Vectorized scoring check for a batch of near-hub fuels."""
        sub_pos = pos[indices]
        sub_vel = vel[indices]
        dx = sub_pos[:, 0] - self.cx
        dy = sub_pos[:, 1] - self.cy
        dist2d = np.sqrt(dx * dx + dy * dy)
        prev_z = sub_pos[:, 2] - sub_vel[:, 2] * dt
        scored = (
                (dist2d <= self.ENTRY_RADIUS) &
                (sub_pos[:, 2] <= self.ENTRY_HEIGHT) &
                (prev_z > self.ENTRY_HEIGHT)
        )
        hit = indices[scored]
        for i in hit:
            scored_flags[i] = True
            pos[i] = [self.exit.x, self.exit.y, self.exit.z]
            vel[i] = [
                self.exit_vel_x_mult * (random.random() + 0.1) * 1.5,
                random.uniform(-1, 1),
                0.0,
            ]
        self._score += int(scored.sum())

    def fuel_hit_net_np(self, px: float, py: float, pz: float) -> float:
        """Returns nonzero x-offset when fuel hits the net (matches original
        logic)."""
        if pz > self._NET_HEIGHT_MAX or pz < self._NET_HEIGHT_MIN:
            return 0.0
        if (py > self.cy + self._NET_WIDTH / 2 or py < self.cy -
                self._NET_WIDTH / 2):
            return 0.0
        nx = self.net_x
        if px > nx:
            return max(0.0, nx - (px - _FUEL_RADIUS))
        return min(0.0, nx - (px + _FUEL_RADIUS))

    def fuel_collide_side_np(
        self, pos: np.ndarray, vel: np.ndarray, i: int
    ) -> None:
        """Rectangle collision for hub sides (scalar fallback)."""
        _rect_collide_np(
            pos, vel, i,
            self.cx - self._SIDE / 2, self.cx + self._SIDE / 2,
            self.cy - self._SIDE / 2, self.cy + self._SIDE / 2,
            0.0, self.ENTRY_HEIGHT - 0.1,
        )

    @property
    def score(self) -> int:
        return self._score

    def reset_score(self) -> None:
        self._score = 0

    def increase_score(self) -> None:
        self._score += 1

    @score.setter
    def score(self, value):
        self._score = value


BLUE_HUB = Hub(
    Translation2d(4.61, _FIELD_WIDTH / 2),
    Translation3d(5.3, _FIELD_WIDTH / 2, 0.89),
    1,
)
RED_HUB = Hub(
    Translation2d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2),
    Translation3d(_FIELD_LENGTH - 5.3, _FIELD_WIDTH / 2, 0.89),
    -1,
)

# Pre-cache hub plain float centers
_BLUE_CX, _BLUE_CY = BLUE_HUB.cx, BLUE_HUB.cy
_RED_CX, _RED_CY = RED_HUB.cx, RED_HUB.cy
_HUB_CLOSE_SQ = (Hub.ENTRY_RADIUS + 1.0) ** 2


def _rect_collide_np(
    pos: np.ndarray, vel: np.ndarray, i: int,
    x0: float, x1: float, y0: float, y1: float, z0: float, z1: float,
) -> None:
    """In-place AABB collision for a single fuel ball (index i)."""
    vx, vy, vz = vel[i, 0], vel[i, 1], vel[i, 2]
    speed_sq = vx * vx + vy * vy + vz * vz
    if speed_sq < 1e-12:
        return
    px, py, pz = pos[i, 0], pos[i, 1], pos[i, 2]
    if pz > z1 + _FUEL_RADIUS or pz < z0 - _FUEL_RADIUS:
        return

    d_left = x0 - _FUEL_RADIUS - px  # positive → fuel is to the left
    d_right = px - x1 - _FUEL_RADIUS  # positive → fuel is to the right
    d_top = py - y1 - _FUEL_RADIUS  # positive → fuel is above
    d_bot = y0 - _FUEL_RADIUS - py  # positive → fuel is below

    if d_left > 0 or d_right > 0 or d_top > 0 or d_bot > 0:
        return  # not overlapping

    # Push out along axis of minimum penetration
    if px < x0 or (d_left >= d_right and d_left >= d_top and d_left >= d_bot):
        pos[i, 0] += d_left
        vel[i, 0] += -_FIELD_COR1 * vx
    elif px >= x1 or (
            d_right >= d_left and d_right >= d_top and d_right >= d_bot):
        pos[i, 0] -= d_right
        vel[i, 0] += -_FIELD_COR1 * vx
    elif py > y1 or (d_top >= d_left and d_top >= d_right and d_top >= d_bot):
        pos[i, 1] -= d_top
        vel[i, 1] += -_FIELD_COR1 * vy
    else:
        pos[i, 1] += d_bot
        vel[i, 1] += -_FIELD_COR1 * vy


def _xz_line_collide_np(
    pos: np.ndarray, vel: np.ndarray, i: int,
    start: np.ndarray, end: np.ndarray,
) -> None:
    """In-place XZ-plane line collision for a single fuel ball."""
    py = pos[i, 1]
    if py < start[1] or py > end[1]:
        return

    # Work in the XZ plane
    sx, sz = start[0], start[2]
    ex, ez = end[0], end[2]
    px, pz = pos[i, 0], pos[i, 2]

    lvx, lvz = ex - sx, ez - sz
    norm_sq = lvx * lvx + lvz * lvz
    if norm_sq < 1e-12:
        return

    # Project fuel onto line
    t = ((px - sx) * lvx + (pz - sz) * lvz) / norm_sq
    proj_x = sx + t * lvx
    proj_z = sz + t * lvz

    # Check projection is within segment
    seg_len = math.sqrt(norm_sq)
    d_from_start = math.sqrt((proj_x - sx) ** 2 + (proj_z - sz) ** 2)
    d_from_end = math.sqrt((proj_x - ex) ** 2 + (proj_z - ez) ** 2)
    if d_from_start + d_from_end > seg_len + 1e-9:
        return

    dist = math.sqrt((px - proj_x) ** 2 + (pz - proj_z) ** 2)
    if dist > _FUEL_RADIUS:
        return

    # Normal points away from line (rotate line tangent 90°)
    nx = -lvz / seg_len
    nz = lvx / seg_len

    # Push out
    overlap = _FUEL_RADIUS - dist
    pos[i, 0] += nx * overlap
    pos[i, 2] += nz * overlap

    # Reflect velocity component along normal
    vx, vz = vel[i, 0], vel[i, 2]
    vdotn = vx * nx + vz * nz
    if vdotn >= 0:
        return  # already moving away
    vel[i, 0] -= (1.0 + _FIELD_COR) * vdotn * nx
    vel[i, 2] -= (1.0 + _FIELD_COR) * vdotn * nz


@dataclass
class SimIntake:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    able_to_intake: Callable[[], bool] = field(default=lambda: True)
    callback: Callable[[], None] = field(default=lambda: None)

    def should_intake(self, px: float, py: float, pz: float,
                      robot_pose: Pose2d, bumper_height: float
                      ) -> bool:
        if not self.able_to_intake() or pz > bumper_height:
            return False
        rel = Pose2d(Translation2d(px, py), Rotation2d()).relativeTo(
            robot_pose
        ).translation()
        result = (self.x_min <= rel.x <= self.x_max and
                  self.y_min <= rel.y <= self.y_max)
        if result:
            self.callback()
        return result


# Grid constants
_CELL_SIZE = _FUEL_RADIUS * 2.2
_GRID_COLS = math.ceil(_FIELD_LENGTH / _CELL_SIZE)
_GRID_ROWS = math.ceil(_FIELD_WIDTH / _CELL_SIZE)


def _collide_trench(pos: np.ndarray, vel: np.ndarray, i: int) -> None:
    px = pos[i, 0]
    # Fast bounds check — most balls are nowhere near the trench
    if not (_TRENCH_X_MIN <= px <= _TRENCH_X_LEFT_MAX or
            _TRENCH_X_RIGHT_MIN <= px <= _TRENCH_X_MAX):
        return
    for x0, x1, y0, y1, z0, z1 in _TRENCH_RECTS:
        _rect_collide_np(pos, vel, i, x0, x1, y0, y1, z0, z1)


def _collide_edges(pos: np.ndarray, vel: np.ndarray, i: int) -> None:
    px, py = pos[i, 0], pos[i, 1]
    vx, vy = vel[i, 0], vel[i, 1]
    if px < _FUEL_RADIUS and vx < 0:
        pos[i, 0] = _FUEL_RADIUS
        vel[i, 0] += -(1.0 + _FIELD_COR) * vx
    elif px > _FIELD_LENGTH - _FUEL_RADIUS and vx > 0:
        pos[i, 0] = _FIELD_LENGTH - _FUEL_RADIUS
        vel[i, 0] += -(1.0 + _FIELD_COR) * vx
    if py < _FUEL_RADIUS and vy < 0:
        pos[i, 1] = _FUEL_RADIUS
        vel[i, 1] += -(1.0 + _FIELD_COR) * vy
    elif py > _FIELD_WIDTH - _FUEL_RADIUS and vy > 0:
        pos[i, 1] = _FIELD_WIDTH - _FUEL_RADIUS
        vel[i, 1] += -(1.0 + _FIELD_COR) * vy


class FuelSim:
    """
    Vectorized fuel simulator.

    Internal state:
        _pos : np.ndarray  shape (capacity, 3)  float64  [x, y, z]
        _vel : np.ndarray  shape (capacity, 3)  float64  [vx, vy, vz]
        _alive : np.ndarray  shape (capacity,)  bool
        _sleep : np.ndarray  shape (capacity,)  uint8   sleep counter
        _n : int  — number of allocated slots (may include dead slots)

    Active indices are kept as a cached view; rebuilt when balls are
    added/removed.
    """

    # How many ticks nearly-stationary before sleeping
    _SLEEP_THRESHOLD = 10
    _SLEEP_SPEED_SQ = 1e-6  # (m/s)² — considered "still"

    def __init__(self, table_key: str = "Fuel Simulation/") -> None:
        cap = 512  # initial capacity; doubles as needed
        self._pos = np.zeros((cap, 3), dtype=np.float64)
        self._vel = np.zeros((cap, 3), dtype=np.float64)
        self._alive = np.zeros(cap, dtype=bool)
        self._sleep = np.zeros(cap, dtype=np.uint8)
        self._n = 0  # next free slot
        self._cap = cap

        self.running: bool = False
        self.simulate_air_resistance: bool = False
        self.subticks: int = 5
        self.intakes: list[SimIntake] = []
        self._table_key = table_key

        self.robot_pose_supplier: Optional[Callable[[], Pose2d]] = None
        self.robot_speeds_supplier: Optional[
            Callable[[], ChassisSpeeds]] = None
        self.robot_width: float = 0.0
        self.robot_length: float = 0.0
        self.bumper_height: float = 0.0

    def _ensure_capacity(self, extra: int) -> None:
        needed = self._n + extra
        if needed <= self._cap:
            return
        new_cap = max(needed, self._cap * 2)
        new_pos = np.zeros((new_cap, 3), dtype=np.float64)
        new_pos[:self._cap] = self._pos
        self._pos = new_pos
        new_vel = np.zeros((new_cap, 3), dtype=np.float64)
        new_vel[:self._cap] = self._vel
        self._vel = new_vel
        self._alive = np.resize(self._alive, (new_cap,))
        self._sleep = np.resize(self._sleep, (new_cap,))
        self._pos[self._cap:] = 0.0
        self._vel[self._cap:] = 0.0
        self._alive[self._cap:] = False
        self._sleep[self._cap:] = 0
        self._cap = new_cap

    def _add_ball(self, px: float, py: float, pz: float,
                  vx: float = 0.0, vy: float = 0.0, vz: float = 0.0
                  ) -> int:
        self._ensure_capacity(1)
        i = self._n
        self._pos[i] = [px, py, pz]
        self._vel[i] = [vx, vy, vz]
        self._alive[i] = True
        self._sleep[i] = 0
        self._n += 1
        return i

    @property
    def _active(self) -> np.ndarray:
        """Indices of alive balls."""
        return np.where(self._alive[:self._n])[0]

    def _compact(self) -> None:
        """Remove dead slots (called after intakes remove balls)."""
        alive = self._alive[:self._n]
        idx = np.where(alive)[0]
        k = len(idx)
        self._pos[:k] = self._pos[idx]
        self._vel[:k] = self._vel[idx]
        self._alive[:k] = True
        self._sleep[:k] = self._sleep[idx]
        self._alive[k:self._n] = False
        self._n = k

    def clear_fuel(self) -> None:
        self._alive[:self._n] = False
        self._n = 0

    def spawn_fuel(self, pos: Translation3d, vel: Translation3d) -> None:
        self._add_ball(pos.x, pos.y, pos.z, vel.x, vel.y, vel.z)

    @property
    def fuels(self) -> "FuelListView":
        idx = self._active
        return FuelListView(self._pos, self._vel, idx)

    def spawn_starting_fuel(self) -> None:
        cx = _FIELD_LENGTH / 2
        cy = _FIELD_WIDTH / 2
        balls = [
            (cx + x * (0.076 + 0.152 * j),
             cy + y * (0.0254 + 0.076 + 0.152 * i),
             _FUEL_RADIUS)
            for i in range(15)
            for j in range(6)
            for x, y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        ]
        self._ensure_capacity(len(balls))
        for px, py, pz in balls:
            self._add_ball(px, py, pz)

    def spawn_less_starting_fuel(self) -> None:
        cx = _FIELD_LENGTH / 2
        cy = _FIELD_WIDTH / 2
        balls = [
            (cx + x * (0.076 + 0.152 * j),
             cy + y * (0.0254 + 0.076 + 0.152 * i),
             _FUEL_RADIUS)
            for i in range(3)
            for j in range(2)
            for x, y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        ]
        self._ensure_capacity(len(balls))
        for px, py, pz in balls:
            self._add_ball(px, py, pz)

    def spawn_depot_fuel(self) -> None:
        balls = []
        for i in range(3):
            for j in range(4):
                ox = 0.076 + 0.152 * j
                oy_b = 5.95 + 0.076 + 0.152 * i
                oy_t = 5.95 - 0.076 - 0.152 * i
                rx = _FIELD_LENGTH - 0.076 - 0.152 * j
                ry_b = 2.09 + 0.076 + 0.152 * i
                ry_t = 2.09 - 0.076 - 0.152 * i
                balls += [
                    (ox, oy_b, _FUEL_RADIUS),
                    (ox, oy_t, _FUEL_RADIUS),
                    (rx, ry_b, _FUEL_RADIUS),
                    (rx, ry_t, _FUEL_RADIUS),
                ]
        self._ensure_capacity(len(balls))
        for px, py, pz in balls:
            self._add_ball(px, py, pz)

    def register_robot(
        self,
        width: meters,
        length: meters,
        bumper_height: meters,
        pose_supplier: Callable[[], Pose2d],
        field_speeds_supplier: Callable[[], ChassisSpeeds],
    ) -> None:
        """Register a robot to the FuelSim"""
        self.robot_pose_supplier = pose_supplier
        self.robot_speeds_supplier = field_speeds_supplier
        self.robot_width = width
        self.robot_length = length
        self.bumper_height = bumper_height

    def register_intake(
        self,
        x_min: float, x_max: float,
        y_min: float, y_max: float,
        able_to_intake: Callable[[], bool] = lambda: True,
        callback: Callable[[], None] = lambda: None,
    ) -> None:
        self.intakes.append(
            SimIntake(x_min, x_max, y_min, y_max, able_to_intake, callback)
        )

    def start(self) -> None:
        self.running = True

    def stop(self) -> None:
        self.running = False

    def enable_air_resistance(self) -> None:
        self.simulate_air_resistance = True

    def set_subticks(self, subticks: int) -> None:
        self.subticks = subticks

    def update_sim(self) -> None:
        if self.running:
            self.step_sim()

    def step_sim(self) -> None:
        for _ in range(self.subticks):
            self._physics_step()
            self._collision_step()
            if self.robot_pose_supplier is not None:
                self._handle_robot_collisions()
                self._handle_intakes()
        self._log()


    def launch_fuel(
        self,
        launch_velocity: meters_per_second,
        hood_angle: radians,
        turret_yaw: radians,
        launch_height: meters,
    ) -> None:
        if (self.robot_pose_supplier is None or self.robot_speeds_supplier is
                None):
            raise RuntimeError(
                "Robot must be registered before launching fuel."
            )
        launch_pose = Pose3d(self.robot_pose_supplier()) + Transform3d(
            Translation3d(0, 0, launch_height), Rotation3d()
        )
        field_speeds = self.robot_speeds_supplier()
        horizontal_vel = math.cos(hood_angle) * launch_velocity
        vertical_vel = math.sin(hood_angle) * launch_velocity
        yaw = turret_yaw + launch_pose.rotation().z
        vx = horizontal_vel * math.cos(yaw) + field_speeds.vx
        vy = horizontal_vel * math.sin(yaw) + field_speeds.vy
        lp = launch_pose.translation()
        self._add_ball(lp.x, lp.y, lp.z, vx, vy, vertical_vel)

    def _physics_step(self) -> None:
        """
        Advance all active, non-sleeping balls by dt = _PERIOD / subticks.
        """
        if self._n == 0:
            return
        dt = _PERIOD / self.subticks

        idx = self._active  # shape (M,)
        if len(idx) == 0:
            return

        pos = self._pos  # (N, 3) — we index with idx throughout
        vel = self._vel

        # Split into sleeping and awake
        sleeping = self._sleep[idx] >= self._SLEEP_THRESHOLD
        awake = idx[~sleeping]

        if len(awake) == 0:
            return

        p = pos[awake]  # (M, 3) view-like slice (copy due to fancy index)
        v = vel[awake]

        # --- Integrate position ---
        p = p + v * dt

        # --- Gravity + optional drag ---
        in_air = p[:, 2] > _FUEL_RADIUS  # shape (M,)

        if self.simulate_air_resistance:
            speed_sq = np.einsum('ij,ij->i', v, v)  # (M,)
            speed = np.sqrt(np.maximum(speed_sq, 1e-30))  # avoid /0
            drag_factor = -_DRAG_OVER_MASS * speed  # (M,)
            ax = drag_factor * v[:, 0]
            ay = drag_factor * v[:, 1]
            az = drag_factor * v[:, 2]
            v[:, 0] += ax * dt
            v[:, 1] += ay * dt
            v[:, 2] = np.where(in_air, v[:, 2] + (_GRAVITY + az) * dt, v[:, 2])
        else:
            v[:, 2] = np.where(in_air, v[:, 2] + _GRAVITY * dt, v[:, 2])

        # --- Ground contact and friction ---
        near_ground = (~in_air) & (p[:, 2] <= _FUEL_RADIUS + 0.03)
        slow_z = np.abs(v[:, 2]) < 0.05
        on_ground = near_ground & slow_z

        v[:, 2] = np.where(on_ground, 0.0, v[:, 2])
        fric = 1.0 - _FRICTION * dt
        v[:, 0] = np.where(on_ground, v[:, 0] * fric, v[:, 0])
        v[:, 1] = np.where(on_ground, v[:, 1] * fric, v[:, 1])

        # Write back
        pos[awake] = p
        vel[awake] = v

        # --- Sleep accounting ---
        speed_sq_all = np.einsum('ij,ij->i', vel[awake], vel[awake])
        still = speed_sq_all < self._SLEEP_SPEED_SQ
        grounded = pos[awake, 2] <= _FUEL_RADIUS + 0.01
        can_sleep = still & grounded

        # Increment sleep counter for balls that qualify
        self._sleep[awake] = np.where(
            can_sleep,
            np.minimum(self._sleep[awake] + 1, self._SLEEP_THRESHOLD),
            0,
        ).astype(np.uint8)

    def _collision_step(self) -> None:
        """Field boundary + hub + trench collisions."""
        idx = self._active
        if len(idx) == 0:
            return

        pos = self._pos
        vel = self._vel

        for i in idx:
            # Skip sleeping balls (they're at rest; no new collisions expected)
            if self._sleep[i] >= self._SLEEP_THRESHOLD:
                continue

            vx, vy = vel[i, 0], vel[i, 1]
            vel_sq = vx * vx + vy * vy
            if vel_sq < 1e-12:
                continue

            # -- Field edges --
            _collide_edges(pos, vel, i)

            # -- XZ line ramps --
            px, py = pos[i, 0], pos[i, 1]
            for li, (mn_x, mx_x, mn_y, mx_y) in enumerate(_LINE_BOUNDS):
                if (px + _FUEL_RADIUS >= mn_x and px - _FUEL_RADIUS <= mx_x and
                        py + _FUEL_RADIUS >= mn_y and py - _FUEL_RADIUS <=
                        mx_y):
                    _xz_line_collide_np(
                        pos,
                        vel,
                        i,
                        _NP_LINES[li][0],
                        _NP_LINES[li][1]
                    )

            # -- Hub --
            if pos[i, 0] < _FIELD_LENGTH / 2:
                self._collide_hub(pos, vel, i, BLUE_HUB)
            else:
                self._collide_hub(pos, vel, i, RED_HUB)

            # -- Trench --
            _collide_trench(pos, vel, i)

        # -- Fuel-fuel --
        self._collide_fuel_fuel()

    def _collide_hub(
        self, pos: np.ndarray, vel: np.ndarray, i: int, hub: Hub
    ) -> None:
        px, py = pos[i, 0], pos[i, 1]
        dx, dy = px - hub.cx, py - hub.cy
        if dx * dx + dy * dy > _HUB_CLOSE_SQ:
            return

        # Scoring
        dt = _PERIOD / self.subticks
        pz = pos[i, 2]
        vz = vel[i, 2]
        prev_z = pz - vz * dt
        dist2d = math.sqrt(dx * dx + dy * dy)
        if (dist2d <= hub.ENTRY_RADIUS and
                pz <= hub.ENTRY_HEIGHT < prev_z):
            pos[i] = [hub.exit.x, hub.exit.y, hub.exit.z]
            vel[i] = [
                hub.exit_vel_x_mult * (random.random() + 0.1) * 1.5,
                random.uniform(-1, 1),
                0.0,
            ]
            hub.increase_score()
            self._sleep[i] = 0
            return

        hub.fuel_collide_side_np(pos, vel, i)

        net = hub.fuel_hit_net_np(float(pos[i, 0]), float(pos[i, 1]), float(pos[i, 2]))
        if net != 0.0:
            pos[i, 0] += net
            vel[i, 0] = -vel[i, 0] * _NET_COR
            vel[i, 1] = vel[i, 1] * _NET_COR
            self._sleep[i] = 0

    def _collide_fuel_fuel(self) -> None:
        """Fuel-fuel collision (spatial grid, scalar resolution)"""
        grid: dict[tuple[int, int], list[int]] = defaultdict(list)
        idx = self._active
        pos = self._pos

        for i in idx:
            col = int(pos[i, 0] / _CELL_SIZE)
            row = int(pos[i, 1] / _CELL_SIZE)
            if 0 <= col < _GRID_COLS and 0 <= row < _GRID_ROWS:
                grid[col, row].append(i)

        checked: set[tuple[int, int]] = set()
        for i in idx:
            col = int(pos[i, 0] / _CELL_SIZE)
            row = int(pos[i, 1] / _CELL_SIZE)
            for ci in range(col - 1, col + 2):
                for ri in range(row - 1, row + 2):
                    for j in grid.get((ci, ri), []):
                        if i >= j:
                            continue
                        pair = (i, j)
                        if pair in checked:
                            continue
                        checked.add(pair)
                        dx = pos[i, 0] - pos[j, 0]
                        dy = pos[i, 1] - pos[j, 1]
                        dz = pos[i, 2] - pos[j, 2]
                        dist_sq = dx * dx + dy * dy + dz * dz
                        if dist_sq < _FUEL_DIAM_SQ:
                            self._resolve_fuel_collision(i, j, float(dist_sq))

    def _resolve_fuel_collision(self, a: int, b: int, dist_sq: float) -> None:
        """Fuel-fuel collision handling"""
        pos, vel = self._pos, self._vel
        dist = math.sqrt(dist_sq) if dist_sq > 0 else 1.0
        if dist == 0:
            nx, ny, nz = 1.0, 0.0, 0.0
        else:
            nx = (pos[a, 0] - pos[b, 0]) / dist
            ny = (pos[a, 1] - pos[b, 1]) / dist
            nz = (pos[a, 2] - pos[b, 2]) / dist

        dvx = vel[b, 0] - vel[a, 0]
        dvy = vel[b, 1] - vel[a, 1]
        dvz = vel[b, 2] - vel[a, 2]
        impulse = 0.5 * _FUEL_COR1 * (dvx * nx + dvy * ny + dvz * nz)

        overlap = _FUEL_DIAM - dist
        pos[a, 0] += nx * overlap * 0.5
        pos[a, 1] += ny * overlap * 0.5
        pos[a, 2] += nz * overlap * 0.5
        pos[b, 0] -= nx * overlap * 0.5
        pos[b, 1] -= ny * overlap * 0.5
        pos[b, 2] -= nz * overlap * 0.5

        vel[a, 0] += impulse * nx
        vel[a, 1] += impulse * ny
        vel[a, 2] += impulse * nz
        vel[b, 0] -= impulse * nx
        vel[b, 1] -= impulse * ny
        vel[b, 2] -= impulse * nz

        # Wake both balls
        self._sleep[a] = 0
        self._sleep[b] = 0

    def _handle_robot_collisions(self) -> None:
        """Fuel-robot collision handling"""
        if (self.robot_pose_supplier is None or self.robot_speeds_supplier is
                None):
            return
        robot = self.robot_pose_supplier()
        speeds = self.robot_speeds_supplier()
        rvx, rvy = speeds.vx, speeds.vy
        half_l = self.robot_length / 2
        half_w = self.robot_width / 2
        bh = self.bumper_height
        pos, vel = self._pos, self._vel

        for i in self._active:
            px, py, pz = pos[i, 0], pos[i, 1], pos[i, 2]
            dx = px - robot.translation().x
            dy = py - robot.translation().y
            if dx * dx + dy * dy > self.robot_length ** 2:
                continue
            if pz > bh:
                continue

            rel = Pose2d(Translation2d(float(px), float(py)), Rotation2d()).relativeTo(
                robot
            ).translation()
            rx, ry = rel.x, rel.y

            d_bot = -_FUEL_RADIUS - half_l - rx
            d_top = -_FUEL_RADIUS - half_l + rx
            d_right = -_FUEL_RADIUS - half_w - ry
            d_left = -_FUEL_RADIUS - half_w + ry

            if d_bot > 0 or d_top > 0 or d_right > 0 or d_left > 0:
                continue

            # Minimum penetration axis
            if d_bot >= d_top and d_bot >= d_right and d_bot >= d_left:
                off = Translation2d(d_bot, 0).rotateBy(robot.rotation())
            elif d_top >= d_bot and d_top >= d_right and d_top >= d_left:
                off = Translation2d(-d_top, 0).rotateBy(robot.rotation())
            elif d_right >= d_bot and d_right >= d_top and d_right >= d_left:
                off = Translation2d(0, d_right).rotateBy(robot.rotation())
            else:
                off = Translation2d(0, -d_left).rotateBy(robot.rotation())

            pos[i, 0] += off.x
            pos[i, 1] += off.y
            n = off / off.norm() if off.norm() > 1e-12 else Translation2d(1, 0)
            nx_f, ny_f = n.x, n.y

            vdotn = vel[i, 0] * nx_f + vel[i, 1] * ny_f
            if vdotn < 0:
                vel[i, 0] += -vdotn * _ROBOT_COR1 * nx_f
                vel[i, 1] += -vdotn * _ROBOT_COR1 * ny_f

            rdotn = rvx * nx_f + rvy * ny_f
            if rdotn > 0:
                vel[i, 0] += rdotn * nx_f
                vel[i, 1] += rdotn * ny_f

            self._sleep[i] = 0

    def _handle_intakes(self) -> None:
        if not self.robot_pose_supplier:
            return
        robot = self.robot_pose_supplier()
        bh = self.bumper_height
        pos = self._pos
        removed = False
        for intake in self.intakes:
            for i in range(self._n - 1, -1, -1):
                if not self._alive[i]:
                    continue
                if intake.should_intake(
                        float(pos[i, 0]),
                        float(pos[i, 1]),
                        float(pos[i, 2]),
                        robot,
                        bh
                ):
                    self._alive[i] = False
                    removed = True
        if removed:
            self._compact()

    def _log(self) -> None:
        idx = self._active
        Logger.recordOutput(
            f"{self._table_key}/Fuel",
            [Translation3d(
                float(self._pos[i, 0]),
                float(self._pos[i, 1]),
                float(self._pos[i, 2])
            ) for i in idx],
        )
        Logger.recordOutput(f"{self._table_key}/RedScore", RED_HUB.score)
        Logger.recordOutput(f"{self._table_key}/BlueScore", BLUE_HUB.score)


class FuelListView:
    """Lightweight proxy so `len(sim.fuels)` and iteration still work."""
    __slots__ = ("_pos", "_vel", "_idx")

    def __init__(self, pos: np.ndarray, vel: np.ndarray, idx: np.ndarray):
        self._pos = pos
        self._vel = vel
        self._idx = idx

    def __len__(self) -> int:
        return len(self._idx)

    def __iter__(self):
        for i in self._idx:
            yield FuelProxy(self._pos, self._vel, i)

    def __getitem__(self, n: int):
        return FuelProxy(self._pos, self._vel, int(self._idx[n]))


@dataclass
class FuelProxy:
    """Read-write proxy for a single fuel in the numpy arrays."""
    __slots__ = ("_pos", "_vel", "_i")
    _pos: np.ndarray
    _vel: np.ndarray
    _i: int

    @property
    def pos(self) -> Translation3d:
        r = self._pos[self._i]
        return Translation3d(float(r[0]), float(r[1]), float(r[2]))

    @pos.setter
    def pos(self, v: Translation3d) -> None:
        self._pos[self._i] = [v.x, v.y, v.z]

    @property
    def vel(self) -> Translation3d:
        r = self._vel[self._i]
        return Translation3d(float(r[0]), float(r[1]), float(r[2]))

    @vel.setter
    def vel(self, v: Translation3d) -> None:
        self._vel[self._i] = [v.x, v.y, v.z]
