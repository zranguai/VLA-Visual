import json

import numpy as np
import yaml

from eval_model.metadata import load_dataset_metadata


def test_metadata_reorders_right_first_dataset_to_ros_order(tmp_path):
    root = tmp_path / "episode"
    meta = root / "meta"
    meta.mkdir(parents=True)
    names = [
        *(f"right_joint{i}" for i in range(1, 8)),
        "right_gripper",
        *(f"left_joint{i}" for i in range(1, 8)),
        "left_gripper",
    ]
    info = {
        "codebase_version": "v3.0",
        "fps": 30,
        "total_episodes": 1,
        "splits": {"train": "0:1"},
        "features": {"action": {"names": [names]}},
    }
    stats = {"action": {"q01": list(range(16)), "q99": list(range(20, 36))}}
    (meta / "info.json").write_text(json.dumps(info), encoding="utf-8")
    (meta / "stats.json").write_text(json.dumps(stats), encoding="utf-8")
    replay = tmp_path / "replay.yaml"
    replay.write_text(
        yaml.safe_dump({"dataset": {"root": str(root), "episode_index": 0}}),
        encoding="utf-8",
    )

    metadata = load_dataset_metadata(replay)

    np.testing.assert_array_equal(metadata.action_q01[:8], np.arange(8, 16))
    np.testing.assert_array_equal(metadata.action_q01[8:], np.arange(0, 8))
