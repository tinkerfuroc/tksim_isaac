from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from .calibration import BaseCalibration


@dataclass(frozen=True)
class Twist2D:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class WheelCommand:
    left_rad_s: float
    right_rad_s: float
    watchdog_stop: bool


@dataclass(frozen=True)
class OdomEstimate:
    stamp: float
    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float


class BaseParityModel:
    """Safe command and wheel-observation model for the Tracer facade."""

    def __init__(self, calibration: BaseCalibration) -> None:
        calibration.validate()
        self.calibration = calibration
        self._commands: deque[tuple[float, Twist2D]] = deque()
        self._last_command_wall_time: float | None = None
        self._last_applied = Twist2D()
        self._safety_stop = False
        self._last_observation_time: float | None = None
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

    @property
    def safety_stop(self) -> bool:
        return self._safety_stop

    def set_safety_stop(self, stopped: bool) -> None:
        self._safety_stop = bool(stopped)
        if stopped:
            self._commands.clear()
            self._last_applied = Twist2D()

    def reset(self) -> None:
        self._commands.clear()
        self._last_command_wall_time = None
        self._last_applied = Twist2D()
        self._last_observation_time = None
        self._x = self._y = self._yaw = 0.0
        self._safety_stop = False

    def accept_command(self, command: Twist2D, wall_time: float) -> None:
        linear = max(-self.calibration.max_linear_mps, min(self.calibration.max_linear_mps, float(command.linear_x)))
        angular = max(-self.calibration.max_angular_rps, min(self.calibration.max_angular_rps, float(command.angular_z)))
        self._commands.append((wall_time, Twist2D(linear, angular)))
        self._last_command_wall_time = wall_time

    def wheel_command(self, wall_time: float) -> WheelCommand:
        stale = self._last_command_wall_time is None or wall_time - self._last_command_wall_time > self.calibration.command_timeout_s
        if self._safety_stop or stale:
            self._last_applied = Twist2D()
            return WheelCommand(0.0, 0.0, stale)
        ready_at = wall_time - self.calibration.command_latency_s
        while self._commands and self._commands[0][0] <= ready_at:
            _, self._last_applied = self._commands.popleft()
        radius = self.calibration.wheel_radius_m
        half_track = self.calibration.wheel_track_m / 2.0
        left = (self._last_applied.linear_x - half_track * self._last_applied.angular_z) / radius
        right = (self._last_applied.linear_x + half_track * self._last_applied.angular_z) / radius
        return WheelCommand(left, right, False)

    def observe_wheels(self, left_rad_s: float, right_rad_s: float, simulation_time: float) -> OdomEstimate:
        radius = self.calibration.wheel_radius_m
        raw_linear = radius * (left_rad_s + right_rad_s) / 2.0
        raw_angular = radius * (right_rad_s - left_rad_s) / self.calibration.wheel_track_m
        linear = raw_linear * self.calibration.linear_scale + self.calibration.linear_bias_mps
        angular = raw_angular * self.calibration.angular_scale + self.calibration.angular_bias_rps
        if self._last_observation_time is not None:
            dt = simulation_time - self._last_observation_time
            if dt < 0.0:
                raise ValueError("simulation time moved backwards")
            if dt > 0.0:
                midpoint = self._yaw + 0.5 * angular * dt
                self._x += linear * math.cos(midpoint) * dt
                self._y += linear * math.sin(midpoint) * dt
                self._yaw = math.atan2(math.sin(self._yaw + angular * dt), math.cos(self._yaw + angular * dt))
        self._last_observation_time = simulation_time
        return OdomEstimate(simulation_time, self._x, self._y, self._yaw, linear, angular)
