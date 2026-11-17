from __future__ import annotations

import csv
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from .alignment import MatchedSample
from .constants import SIGNAL_NAMES
from .metrics import ActionNormalizer


def make_run_directory(output_root: Path, run_label: str) -> Path:
    safe_label = re.sub(r"[^A-Za-z0-9_.-]+", "-", run_label.strip()).strip("-.")
    if not safe_label:
        safe_label = "model"
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    return output_root / f"{timestamp}-{safe_label}"


def write_report(
    output_directory: Path,
    summary: dict[str, Any],
    matches: list[MatchedSample],
    normalizer: ActionNormalizer,
) -> tuple[Path, Path]:
    output_directory.mkdir(parents=True, exist_ok=True)
    summary_path = output_directory / "summary.json"
    samples_path = output_directory / "aligned_samples.csv"

    temporary_summary = output_directory / ".summary.json.tmp"
    temporary_summary.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary_summary.replace(summary_path)

    header = ["timestamp_ns", "timestamp_s"]
    for prefix in ("reference", "prediction", "abs_normalized_error"):
        header.extend(f"{prefix}_{name}" for name in SIGNAL_NAMES)
    with samples_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        if matches:
            origin = matches[0].timestamp_ns
            for match in matches:
                reference = normalizer.canonicalize(match.reference)
                prediction = normalizer.canonicalize(match.prediction)
                error = np.abs(normalizer.normalized_error(prediction, reference))
                writer.writerow(
                    [
                        match.timestamp_ns,
                        (match.timestamp_ns - origin) / 1e9,
                        *reference.tolist(),
                        *prediction.tolist(),
                        *error.tolist(),
                    ]
                )
    return summary_path, samples_path


def summary_markdown(summary: dict[str, Any]) -> str:
    coverage = summary["coverage"]
    lines = [
        "# Model action evaluation",
        "",
        f"- Status: **{summary['status']}**",
        f"- Run: `{summary['run']['label']}`",
        f"- Matched samples: {coverage['matched_samples']} / {coverage['prediction_messages']}",
        f"- Valid for model ranking: **{summary['run']['valid_for_model_ranking']}**",
    ]
    metrics = summary.get("metrics", {})
    joint = metrics.get("joint", {}).get("all")
    if joint:
        lines.extend(
            [
                f"- Joint NMAE: **{joint['mean']:.6f}**",
                f"- Joint error P95: **{joint['p95']:.6f}**",
            ]
        )
    temporal = metrics.get("temporal_alignment")
    if temporal:
        lines.extend(
            [
                f"- Estimated delay: **{temporal['estimated_delay_ms']:.1f} ms**",
                f"- Delay-compensated joint NMAE: **{temporal['compensated_joint_nmae']:.6f}**",
            ]
        )
    warnings = summary.get("warnings", [])
    if warnings:
        lines.extend(["", "## Warnings", ""])
        lines.extend(f"- {warning}" for warning in warnings)
    return "\n".join(lines)
