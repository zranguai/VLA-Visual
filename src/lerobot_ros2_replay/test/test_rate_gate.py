import pytest

from lerobot_ros2_replay.replay_node import RateGate


def test_rate_gate_drops_missed_intervals_without_bursting():
    gate = RateGate(10.0)
    assert gate.due(0.0)
    assert not gate.due(0.05)
    assert gate.due(0.10)
    assert gate.due(0.36)
    assert not gate.due(0.36)
    assert gate.next_due_s == pytest.approx(0.4)


def test_zero_rate_is_disabled():
    gate = RateGate(0.0)
    assert not gate.due(100.0)
