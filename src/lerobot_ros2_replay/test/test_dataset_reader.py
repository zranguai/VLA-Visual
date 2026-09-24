from pathlib import Path

import numpy as np
import pytest

from lerobot_ros2_replay.dataset_reader import LeRobotEpisodeReader


DATASET_ROOT = Path("/home/zme/datasets/lerobot_test/episode_2026-09-11_10-46-33")


@pytest.fixture(scope="module")
def reader():
    if not DATASET_ROOT.is_dir():
        pytest.skip(f"sample dataset unavailable: {DATASET_ROOT}")
    return LeRobotEpisodeReader(DATASET_ROOT, episode_index=0, video_backend="pyav")


def test_sample_metadata_and_left_right_reorder(reader):
    assert reader.length == 9581
    assert reader.fps == 30.0
    state, action = reader.low_dimensional(0)
    assert state.shape == (16,)
    assert action.shape == (16,)
    # Source metadata is right-first; the replay API must expose left-first.
    assert state[0] == pytest.approx(-2.1882064342)
    assert state[7] == pytest.approx(99.8125)
    assert state[8] == pytest.approx(-2.1883428097)
    assert state[15] == pytest.approx(100.9609375)
    assert action[0] == pytest.approx(-2.1884822845)
    assert action[7] == pytest.approx(100.0)


def test_sample_image_decode(reader):
    images = reader.images(0, ["observation.images.top"])
    image = images["observation.images.top"]
    assert image.shape == (480, 640, 3)
    assert image.dtype == np.uint8
    assert image.flags.c_contiguous


def test_source_time_lookup(reader):
    assert reader.source_index_at(0.0) == 0
    assert reader.source_index_at(float(reader.timestamps[1])) == 1
    assert reader.source_index_at(10_000.0) == reader.length - 1
