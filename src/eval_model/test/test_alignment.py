import numpy as np

from eval_model.alignment import ActionAligner


def _vector(value: float, left_gripper: float = 0.0, right_gripper: float = 0.0):
    result = np.full(16, value, dtype=np.float64)
    result[7] = left_gripper
    result[15] = right_gripper
    return result


def test_joint_interpolation_and_gripper_zero_order_hold():
    aligner = ActionAligner(max_gap_ms=100)
    aligner.add_reference(1_000_000_000, _vector(0.0, 0.0, 100.0))
    assert aligner.add_prediction(1_050_000_000, _vector(1.0)) == []
    matches = aligner.add_reference(1_100_000_000, _vector(2.0, 100.0, 0.0))

    assert len(matches) == 1
    np.testing.assert_allclose(matches[0].reference[:7], 1.0)
    assert matches[0].reference[7] == 0.0
    assert matches[0].reference[15] == 100.0


def test_unmatched_prediction_is_counted_after_gap_expires():
    aligner = ActionAligner(max_gap_ms=10)
    aligner.add_prediction(1_000_000_000, _vector(0.0))
    aligner.add_reference(1_100_000_000, _vector(0.0))
    aligner.finalize()
    assert aligner.unmatched_prediction_count == 1
    assert not aligner.matches


def test_finalize_uses_nearest_endpoint_within_gap():
    aligner = ActionAligner(max_gap_ms=100)
    aligner.add_reference(1_000_000_000, _vector(2.0))
    aligner.add_prediction(1_020_000_000, _vector(2.0))
    matches = aligner.finalize()
    assert len(matches) == 1
    np.testing.assert_allclose(matches[0].reference, matches[0].prediction)


def test_sparse_reference_uses_nearest_sample_within_gap():
    aligner = ActionAligner(max_gap_ms=100)
    aligner.add_reference(1_000_000_000, _vector(2.0))
    assert aligner.add_prediction(1_005_000_000, _vector(2.0)) == []

    matches = aligner.add_reference(1_150_000_000, _vector(4.0))

    assert len(matches) == 1
    np.testing.assert_allclose(matches[0].reference, _vector(2.0))
