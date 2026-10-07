"""Mission behaviour state machine - **pure Python, no ROS**.

The sequencing logic lives here rather than in the node so it can be unit tested
without a ROS installation (``tests/test_mission_fsm.py``).  ``mission_node.py`` is
a thin adapter that turns ROS messages into :class:`Event` values and executes the
returned :class:`Action` values.

Mission shape (see the top-level README workflow)::

    beacon alert -> navigate to the room -> find the flame -> suppress -> verify

Everything that could hang is bounded by a timeout, and an E-STOP from any state
drops straight to ``ABORTED``.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional


class State(Enum):
    IDLE = "idle"                # waiting for a beacon alert
    CONFIRMING = "confirming"    # first alert seen, waiting for confirmation
    NAVIGATING = "navigating"    # driving to the reported room
    SEARCHING = "searching"      # in the room, sweeping for the flame
    APPROACHING = "approaching"  # flame found, closing to suppression range
    SUPPRESSING = "suppressing"  # extinguisher discharging
    VERIFYING = "verifying"      # did the flame actually go out?
    COMPLETE = "complete"        # success, holding before returning to IDLE
    ABORTED = "aborted"          # fault / e-stop; only RESET leaves this state


class Event(Enum):
    BEACON_ALERT = "beacon_alert"
    NAV_GOAL_REACHED = "nav_goal_reached"
    NAV_GOAL_FAILED = "nav_goal_failed"
    FLAME_FOUND = "flame_found"
    FLAME_LOST = "flame_lost"
    IN_RANGE = "in_range"
    FLAME_EXTINGUISHED = "flame_extinguished"
    FLAME_STILL_PRESENT = "flame_still_present"
    ESTOP = "estop"
    RESET = "reset"


class Action(Enum):
    """Side effects the node must perform on the robot."""

    STOP = "stop"
    REQUEST_NAVIGATE = "request_navigate"
    CANCEL_NAVIGATION = "cancel_navigation"
    START_SEARCH = "start_search"
    STOP_SEARCH = "stop_search"
    START_APPROACH = "start_approach"
    START_SUPPRESS = "start_suppress"
    STOP_SUPPRESS = "stop_suppress"


@dataclass(frozen=True)
class MissionConfig:
    #: the beacon transmits every ~2 s, so the confirmation window must exceed it
    confirm_window_s: float = 5.0
    nav_timeout_s: float = 120.0
    search_timeout_s: float = 30.0
    approach_timeout_s: float = 20.0
    suppress_duration_s: float = 10.0
    verify_timeout_s: float = 5.0
    complete_hold_s: float = 3.0
    max_suppress_attempts: int = 3


def _now(now: Optional[float]) -> float:
    return time.monotonic() if now is None else float(now)


class MissionFSM:
    """Deterministic, side-effect-free mission sequencer."""

    def __init__(self, config: Optional[MissionConfig] = None) -> None:
        self.config = config or MissionConfig()
        self.state = State.IDLE
        self.attempts = 0
        self.last_alert_message_number: Optional[int] = None
        self._since = 0.0

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------
    @property
    def is_active(self) -> bool:
        """True while the robot is doing something mission-related."""
        return self.state not in (State.IDLE, State.COMPLETE, State.ABORTED)

    def elapsed(self, now: Optional[float] = None) -> float:
        return _now(now) - self._since

    def snapshot(self) -> dict:
        return {"state": self.state.value, "attempts": self.attempts}

    def __repr__(self) -> str:  # pragma: no cover - debug aid
        return f"MissionFSM(state={self.state.value}, attempts={self.attempts})"

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------
    def handle_event(self, event: Event, now: Optional[float] = None, **data) -> List[Action]:
        """Feed one event in; get the actions to perform back."""
        now = _now(now)

        # Global overrides -------------------------------------------------
        if event is Event.ESTOP:
            if self.state is State.ABORTED:
                return []
            return self._enter(State.ABORTED, now)

        if event is Event.RESET:
            return self.reset(now)

        if self.state is State.ABORTED:
            return []  # nothing but RESET gets us out

        # Per-state handling ----------------------------------------------
        if self.state is State.IDLE:
            if event is Event.BEACON_ALERT:
                self.last_alert_message_number = data.get("message_number")
                return self._enter(State.CONFIRMING, now)

        elif self.state is State.CONFIRMING:
            if event is Event.BEACON_ALERT:
                return self._enter(State.NAVIGATING, now)

        elif self.state is State.NAVIGATING:
            if event is Event.NAV_GOAL_REACHED:
                return self._enter(State.SEARCHING, now)
            if event is Event.NAV_GOAL_FAILED:
                return self._enter(State.ABORTED, now)

        elif self.state is State.SEARCHING:
            if event is Event.FLAME_FOUND:
                return self._enter(State.APPROACHING, now)

        elif self.state is State.APPROACHING:
            if event is Event.IN_RANGE:
                return self._enter(State.SUPPRESSING, now)
            if event is Event.FLAME_LOST:
                return self._enter(State.SEARCHING, now)  # re-acquire

        elif self.state is State.SUPPRESSING:
            # the flame going out early is a success, not a timeout
            if event is Event.FLAME_EXTINGUISHED:
                return self._enter(State.VERIFYING, now)

        elif self.state is State.VERIFYING:
            if event is Event.FLAME_EXTINGUISHED:
                return self._enter(State.COMPLETE, now)
            if event is Event.FLAME_STILL_PRESENT:
                return self._retry_suppression(now)

        return []

    # ------------------------------------------------------------------
    # Timeouts
    # ------------------------------------------------------------------
    def update(self, now: Optional[float] = None) -> List[Action]:
        """Call periodically; handles every state timeout."""
        now = _now(now)
        elapsed = now - self._since
        cfg = self.config

        if self.state is State.CONFIRMING and elapsed >= cfg.confirm_window_s:
            return self._enter(State.IDLE, now)          # false positive
        if self.state is State.NAVIGATING and elapsed >= cfg.nav_timeout_s:
            return self._enter(State.ABORTED, now)
        if self.state is State.SEARCHING and elapsed >= cfg.search_timeout_s:
            return self._enter(State.ABORTED, now)
        if self.state is State.APPROACHING and elapsed >= cfg.approach_timeout_s:
            return self._enter(State.ABORTED, now)
        if self.state is State.SUPPRESSING and elapsed >= cfg.suppress_duration_s:
            return self._enter(State.VERIFYING, now)
        if self.state is State.VERIFYING and elapsed >= cfg.verify_timeout_s:
            return self._retry_suppression(now)          # assume still burning
        if self.state is State.COMPLETE and elapsed >= cfg.complete_hold_s:
            return self._enter(State.IDLE, now)

        return []

    # ------------------------------------------------------------------
    def reset(self, now: Optional[float] = None) -> List[Action]:
        now = _now(now)
        self.attempts = 0
        self.last_alert_message_number = None
        return self._enter(State.IDLE, now)

    def _retry_suppression(self, now: float) -> List[Action]:
        if self.attempts < self.config.max_suppress_attempts:
            return self._enter(State.SUPPRESSING, now)
        return self._enter(State.ABORTED, now)

    def _enter(self, state: State, now: float) -> List[Action]:
        """Transition, returning the actions triggered on entry."""
        self.state = state
        self._since = now

        if state is State.IDLE:
            return [Action.STOP]

        if state is State.CONFIRMING:
            return [Action.STOP]

        if state is State.NAVIGATING:
            return [Action.REQUEST_NAVIGATE]

        if state is State.SEARCHING:
            return [Action.START_SEARCH]

        if state is State.APPROACHING:
            return [Action.STOP_SEARCH, Action.START_APPROACH]

        if state is State.SUPPRESSING:
            self.attempts += 1
            return [Action.START_SUPPRESS]

        if state is State.VERIFYING:
            return [Action.STOP_SUPPRESS, Action.STOP]

        if state is State.COMPLETE:
            return [Action.STOP, Action.CANCEL_NAVIGATION]

        if state is State.ABORTED:
            return [
                Action.STOP,
                Action.STOP_SEARCH,
                Action.STOP_SUPPRESS,
                Action.CANCEL_NAVIGATION,
            ]

        return []
