from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EpisodeStatus:
    episode_id: int
    scenario: str
    state: str
    simulation_time: float
    episode_start_time: float
    seed: int

    @property
    def episode_time(self) -> float:
        return self.simulation_time - self.episode_start_time


class EpisodeController:
    """Backend-neutral reset/pause state with monotonic public ROS time."""

    def __init__(self, seed: int = 0) -> None:
        self._time = 0.0
        self._episode_start = 0.0
        self._episode_id = 0
        self._scenario = "empty"
        self._seed = int(seed)
        self._paused = False

    @property
    def simulation_time(self) -> float:
        return self._time

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def seed(self) -> int:
        return self._seed

    def advance(self, dt: float) -> float:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        if not self._paused:
            self._time += dt
        return self._time

    def step_paused(self, steps: int, dt: float) -> float:
        if not self._paused:
            raise RuntimeError("explicit stepping requires a paused simulation")
        if steps < 1:
            raise ValueError("steps must be positive")
        self._time += steps * dt
        return self._time

    def pause(self, paused: bool) -> None:
        self._paused = bool(paused)

    def reset(self, seed: int | None = None) -> None:
        self._episode_id += 1
        self._episode_start = self._time
        if seed is not None:
            self._seed = int(seed)

    def load_scenario(self, scenario: str, seed: int) -> None:
        if not self._paused:
            raise RuntimeError("scenario load requires a paused simulation")
        if not scenario or "/" in scenario or ".." in scenario:
            raise ValueError("invalid scenario identifier")
        self._scenario = scenario
        self._seed = int(seed)
        self.reset(seed)

    def status(self) -> EpisodeStatus:
        return EpisodeStatus(self._episode_id, self._scenario, "paused" if self._paused else "running", self._time, self._episode_start, self._seed)
