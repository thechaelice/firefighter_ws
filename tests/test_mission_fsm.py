"""Mission FSM tests - pure Python, no ROS, no rclpy.

    uv run --group test pytest tests/test_mission_fsm.py
    python3 -m pytest tests/test_mission_fsm.py

Every transition is driven with an explicit clock so the tests are deterministic
and never sleep.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(
    0, str(Path(__file__).resolve().parents[1] / "src" / "firefighter_mission")
)

from firefighter_mission.mission_fsm import (  # noqa: E402
    Action,
    Event,
    MissionConfig,
    MissionFSM,
    State,
)


class Clock:
    """Manually advanced clock (seconds)."""

    def __init__(self, start: float = 1000.0) -> None:
        self.t = start

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t


@pytest.fixture()
def clock():
    return Clock()


@pytest.fixture()
def fsm():
    return MissionFSM(MissionConfig())


# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------
def send(fsm, clock, event, **data):
    return fsm.handle_event(event, now=clock(), **data)


def tick(fsm, clock):
    return fsm.update(now=clock())


def goto_navigating(fsm, clock):
    send(fsm, clock, Event.BEACON_ALERT, message_number=1)
    assert fsm.state is State.CONFIRMING
    clock.advance(2.0)
    send(fsm, clock, Event.BEACON_ALERT, message_number=2)
    assert fsm.state is State.NAVIGATING


def goto_searching(fsm, clock):
    goto_navigating(fsm, clock)
    clock.advance(1.0)
    send(fsm, clock, Event.NAV_GOAL_REACHED)
    assert fsm.state is State.SEARCHING


def goto_approaching(fsm, clock):
    goto_searching(fsm, clock)
    clock.advance(1.0)
    send(fsm, clock, Event.FLAME_FOUND)
    assert fsm.state is State.APPROACHING


def goto_suppressing(fsm, clock):
    goto_approaching(fsm, clock)
    clock.advance(1.0)
    send(fsm, clock, Event.IN_RANGE)
    assert fsm.state is State.SUPPRESSING


def goto_verifying(fsm, clock):
    goto_suppressing(fsm, clock)
    clock.advance(1.0)
    send(fsm, clock, Event.FLAME_EXTINGUISHED)
    assert fsm.state is State.VERIFYING


# ------------------------------------------------------------
# Initial state
# ------------------------------------------------------------
def test_starts_idle(fsm):
    assert fsm.state is State.IDLE
    assert fsm.attempts == 0
    assert not fsm.is_active
    assert fsm.snapshot() == {"state": "idle", "attempts": 0}


def test_idle_has_no_timeout(fsm, clock):
    clock.advance(10_000.0)
    assert tick(fsm, clock) == []
    assert fsm.state is State.IDLE


# ------------------------------------------------------------
# Alert confirmation
# ------------------------------------------------------------
def test_single_alert_enters_confirming_and_stops(fsm, clock):
    actions = send(fsm, clock, Event.BEACON_ALERT, message_number=1)
    assert fsm.state is State.CONFIRMING
    assert Action.STOP in actions
    assert fsm.last_alert_message_number == 1
    assert fsm.is_active


def test_unconfirmed_alert_times_out_back_to_idle(fsm, clock):
    send(fsm, clock, Event.BEACON_ALERT, message_number=1)
    clock.advance(1.0)
    assert tick(fsm, clock) == []  # still inside the window
    clock.advance(fsm.config.confirm_window_s)
    actions = tick(fsm, clock)
    assert fsm.state is State.IDLE
    assert Action.STOP in actions


def test_two_alerts_start_navigation(fsm, clock):
    send(fsm, clock, Event.BEACON_ALERT, message_number=1)
    clock.advance(2.0)
    actions = send(fsm, clock, Event.BEACON_ALERT, message_number=2)
    assert fsm.state is State.NAVIGATING
    assert Action.REQUEST_NAVIGATE in actions


def test_confirmation_window_exceeds_beacon_period():
    """The beacon transmits every ~2 s, so the window must be longer."""
    assert MissionConfig().confirm_window_s > 2.0


# ------------------------------------------------------------
# Happy path
# ------------------------------------------------------------
def test_full_happy_path_returns_to_idle(fsm, clock):
    goto_navigating(fsm, clock)

    clock.advance(5.0)
    assert Action.START_SEARCH in send(fsm, clock, Event.NAV_GOAL_REACHED)
    assert fsm.state is State.SEARCHING

    clock.advance(1.0)
    actions = send(fsm, clock, Event.FLAME_FOUND)
    assert fsm.state is State.APPROACHING
    assert Action.STOP_SEARCH in actions
    assert Action.START_APPROACH in actions

    clock.advance(1.0)
    assert Action.START_SUPPRESS in send(fsm, clock, Event.IN_RANGE)
    assert fsm.state is State.SUPPRESSING
    assert fsm.attempts == 1

    clock.advance(2.0)
    actions = send(fsm, clock, Event.FLAME_EXTINGUISHED)
    assert fsm.state is State.VERIFYING
    assert Action.STOP_SUPPRESS in actions
    assert Action.STOP in actions

    clock.advance(1.0)
    actions = send(fsm, clock, Event.FLAME_EXTINGUISHED)
    assert fsm.state is State.COMPLETE
    assert Action.STOP in actions
    assert Action.CANCEL_NAVIGATION in actions
    assert not fsm.is_active

    clock.advance(fsm.config.complete_hold_s)
    tick(fsm, clock)
    assert fsm.state is State.IDLE
    assert fsm.attempts == 1


# ------------------------------------------------------------
# Failure branches
# ------------------------------------------------------------
def test_nav_goal_failed_aborts(fsm, clock):
    goto_navigating(fsm, clock)
    actions = send(fsm, clock, Event.NAV_GOAL_FAILED)
    assert fsm.state is State.ABORTED
    assert Action.STOP in actions
    assert Action.CANCEL_NAVIGATION in actions


def test_nav_timeout_aborts(fsm, clock):
    goto_navigating(fsm, clock)
    clock.advance(fsm.config.nav_timeout_s)
    actions = tick(fsm, clock)
    assert fsm.state is State.ABORTED
    assert Action.CANCEL_NAVIGATION in actions
    assert Action.STOP in actions


def test_search_timeout_aborts(fsm, clock):
    goto_searching(fsm, clock)
    clock.advance(fsm.config.search_timeout_s)
    tick(fsm, clock)
    assert fsm.state is State.ABORTED


def test_approach_timeout_aborts(fsm, clock):
    goto_approaching(fsm, clock)
    clock.advance(fsm.config.approach_timeout_s)
    tick(fsm, clock)
    assert fsm.state is State.ABORTED


def test_flame_lost_while_approaching_resumes_search(fsm, clock):
    goto_approaching(fsm, clock)
    clock.advance(1.0)
    actions = send(fsm, clock, Event.FLAME_LOST)
    assert fsm.state is State.SEARCHING
    assert Action.START_SEARCH in actions


def test_suppress_duration_expiry_moves_to_verifying(fsm, clock):
    goto_suppressing(fsm, clock)
    clock.advance(fsm.config.suppress_duration_s)
    actions = tick(fsm, clock)
    assert fsm.state is State.VERIFYING
    assert Action.STOP_SUPPRESS in actions


# ------------------------------------------------------------
# Suppression retries
# ------------------------------------------------------------
def test_verification_failure_retries_suppression(fsm, clock):
    goto_verifying(fsm, clock)
    assert fsm.attempts == 1
    clock.advance(1.0)
    actions = send(fsm, clock, Event.FLAME_STILL_PRESENT)
    assert fsm.state is State.SUPPRESSING
    assert fsm.attempts == 2
    assert Action.START_SUPPRESS in actions


def test_verification_timeout_counts_as_still_present(fsm, clock):
    goto_verifying(fsm, clock)
    clock.advance(fsm.config.verify_timeout_s)
    actions = tick(fsm, clock)
    assert fsm.state is State.SUPPRESSING
    assert fsm.attempts == 2
    assert Action.START_SUPPRESS in actions


def test_gives_up_after_max_suppress_attempts(fsm, clock):
    goto_verifying(fsm, clock)  # attempts == 1
    cfg = fsm.config

    for expected in range(2, cfg.max_suppress_attempts + 1):
        clock.advance(1.0)
        send(fsm, clock, Event.FLAME_STILL_PRESENT)
        assert fsm.state is State.SUPPRESSING
        assert fsm.attempts == expected
        clock.advance(1.0)
        send(fsm, clock, Event.FLAME_EXTINGUISHED)
        assert fsm.state is State.VERIFYING

    clock.advance(1.0)
    actions = send(fsm, clock, Event.FLAME_STILL_PRESENT)
    assert fsm.state is State.ABORTED
    assert fsm.attempts == cfg.max_suppress_attempts
    assert Action.STOP_SUPPRESS in actions


# ------------------------------------------------------------
# E-stop / reset
# ------------------------------------------------------------
@pytest.mark.parametrize(
    "setup",
    [goto_navigating, goto_searching, goto_approaching, goto_suppressing, goto_verifying],
    ids=["navigating", "searching", "approaching", "suppressing", "verifying"],
)
def test_estop_from_every_active_state(fsm, clock, setup):
    setup(fsm, clock)
    actions = send(fsm, clock, Event.ESTOP)
    assert fsm.state is State.ABORTED
    for expected in (Action.STOP, Action.STOP_SEARCH, Action.STOP_SUPPRESS,
                     Action.CANCEL_NAVIGATION):
        assert expected in actions


def test_aborted_only_leaves_via_reset(fsm, clock):
    goto_suppressing(fsm, clock)
    send(fsm, clock, Event.ESTOP)
    assert fsm.state is State.ABORTED

    for event in (Event.BEACON_ALERT, Event.NAV_GOAL_REACHED, Event.FLAME_FOUND,
                  Event.IN_RANGE, Event.FLAME_EXTINGUISHED):
        assert send(fsm, clock, event) == []
        assert fsm.state is State.ABORTED

    clock.advance(10_000.0)
    assert tick(fsm, clock) == []

    actions = send(fsm, clock, Event.RESET)
    assert fsm.state is State.IDLE
    assert fsm.attempts == 0
    assert fsm.last_alert_message_number is None
    assert Action.STOP in actions


def test_repeated_estop_is_idempotent(fsm, clock):
    send(fsm, clock, Event.ESTOP)
    assert send(fsm, clock, Event.ESTOP) == []
    assert fsm.state is State.ABORTED


# ------------------------------------------------------------
# Event hygiene
# ------------------------------------------------------------
def test_alert_ignored_outside_idle_and_confirming(fsm, clock):
    goto_navigating(fsm, clock)
    assert send(fsm, clock, Event.BEACON_ALERT, message_number=9) == []
    assert fsm.state is State.NAVIGATING


def test_irrelevant_events_are_no_ops(fsm, clock):
    goto_searching(fsm, clock)
    for event in (Event.IN_RANGE, Event.FLAME_EXTINGUISHED, Event.NAV_GOAL_FAILED):
        assert send(fsm, clock, event) == []
        assert fsm.state is State.SEARCHING


def test_active_flag_tracks_state(fsm, clock):
    assert not fsm.is_active                     # IDLE
    goto_navigating(fsm, clock)
    assert fsm.is_active
    send(fsm, clock, Event.NAV_GOAL_FAILED)
    assert not fsm.is_active                     # ABORTED
