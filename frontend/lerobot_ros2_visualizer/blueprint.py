from __future__ import annotations

import rerun as rr
import rerun.blueprint as rrb

from .config import VisualizationConfig


CAMERA_LABELS = {
    "top": "Head",
    "left": "Left wrist",
    "right": "Right wrist",
}
SOURCE_STYLES = {
    "state": ("State", [56, 189, 248], 1.5),
    "dataset_action": ("Dataset action", [245, 158, 11], 2.0),
    "model_action": ("Model action", [232, 121, 249], 2.0),
}
EVALUATION_OVERVIEW_PATHS = {
    "joint_nmae": "/evaluation/overview/joint_nmae",
    "rolling_joint_nmae": "/evaluation/overview/rolling_joint_nmae",
    "left_joint_nmae": "/evaluation/overview/left_joint_nmae",
    "right_joint_nmae": "/evaluation/overview/right_joint_nmae",
    "left_gripper_match": "/evaluation/overview/left_gripper_match",
    "right_gripper_match": "/evaluation/overview/right_gripper_match",
}
EVALUATION_STYLES = {
    "joint_nmae": ("Joint NMAE", [239, 68, 68], 1.5),
    "rolling_joint_nmae": ("Joint NMAE (1s mean)", [250, 204, 21], 2.5),
    "left_joint_nmae": ("Left arm NMAE", [34, 211, 238], 1.5),
    "right_joint_nmae": ("Right arm NMAE", [232, 121, 249], 1.5),
    "left_gripper_match": ("Left gripper match", [74, 222, 128], 1.5),
    "right_gripper_match": ("Right gripper match", [96, 165, 250], 1.5),
}


def camera_entity_path(role: str) -> str:
    return f"/cameras/{role}"


def signal_entity_path(signal: str, source: str) -> str:
    return f"/signals/{signal}/{source}"


def evaluation_error_path(signal: str) -> str:
    return f"/evaluation/signals/{signal}/abs_normalized_error"


def _camera_view(role: str, name_suffix: str = "") -> rrb.Spatial2DView:
    label = CAMERA_LABELS[role]
    return rrb.Spatial2DView(
        name=f"{label}{name_suffix}",
        origin=camera_entity_path(role),
        contents=[f"+ {camera_entity_path(role)}"],
    )


def _camera_tabs(config: VisualizationConfig) -> rrb.Tabs:
    enabled = config.cameras.enabled
    all_cameras = rrb.Grid(
        contents=[_camera_view(role, " overview") for role in enabled],
        grid_columns=2,
        name="All cameras",
    )
    individual = [_camera_view(role) for role in enabled]
    active_tab = 0
    if config.cameras.default_view != "all":
        active_tab = enabled.index(config.cameras.default_view) + 1
    return rrb.Tabs(
        contents=[all_cameras, *individual],
        active_tab=active_tab,
        name="Cameras",
    )


def _signal_view(config: VisualizationConfig, signal: str) -> rrb.TimeSeriesView:
    paths = [signal_entity_path(signal, source) for source in config.signals.sources]
    overrides = {
        path: rrb.EntityBehavior(visible=source in config.signals.default_visible_sources)
        for path, source in zip(paths, config.signals.sources, strict=True)
    }
    time_range = rrb.VisibleTimeRange(
        config.viewer.timeline,
        start=rrb.TimeRangeBoundary.cursor_relative(
            seconds=-config.viewer.history_window_s
        ),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )
    return rrb.TimeSeriesView(
        name=signal,
        origin=f"/signals/{signal}",
        contents=[f"+ {path}" for path in paths],
        overrides=overrides,
        plot_legend=rrb.PlotLegend(visible=True),
        time_ranges=time_range,
    )


def _evaluation_tabs(config: VisualizationConfig) -> rrb.Tabs:
    time_range = rrb.VisibleTimeRange(
        config.viewer.timeline,
        start=rrb.TimeRangeBoundary.cursor_relative(seconds=-config.viewer.history_window_s),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )
    overview = rrb.TimeSeriesView(
        name="Overview",
        origin="/evaluation/overview",
        contents=["+ /evaluation/overview/**"],
        plot_legend=rrb.PlotLegend(visible=True),
        time_ranges=time_range,
    )
    error_views = [
        rrb.TimeSeriesView(
            name=signal,
            origin=f"/evaluation/signals/{signal}",
            contents=[f"+ {evaluation_error_path(signal)}"],
            plot_legend=rrb.PlotLegend(visible=True),
            time_ranges=time_range,
        )
        for signal in config.signals.enabled
    ]
    per_signal = rrb.Tabs(contents=error_views, name="Per-signal error")
    summary = rrb.TextDocumentView(
        name="Summary",
        origin="/evaluation/summary",
        contents=["+ /evaluation/summary"],
    )
    return rrb.Tabs(
        contents=[overview, per_signal, summary],
        active_tab=0,
        name="Evaluation",
    )


def build_blueprint(
    config: VisualizationConfig, evaluation_enabled: bool = False
) -> rrb.Blueprint:
    signal_views = [_signal_view(config, signal) for signal in config.signals.enabled]
    signal_tabs = rrb.Tabs(
        contents=signal_views,
        active_tab=config.signals.enabled.index(config.signals.default_signal),
        name="Joint and gripper signals",
    )
    right_content: rrb.ContainerLike = signal_tabs
    if evaluation_enabled:
        right_content = rrb.Tabs(
            contents=[signal_tabs, _evaluation_tabs(config)],
            active_tab=0,
            name="Signals and evaluation",
        )
    viewport = rrb.Horizontal(
        _camera_tabs(config),
        right_content,
        column_shares=[3, 2],
        name="LeRobot ROS 2 replay",
    )
    return rrb.Blueprint(
        viewport,
        rrb.BlueprintPanel(state="expanded"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded", timeline=config.viewer.timeline),
        auto_layout=False,
        auto_views=False,
    )


def log_series_styles(
    config: VisualizationConfig, evaluation_enabled: bool = False
) -> None:
    for signal in config.signals.enabled:
        for source in config.signals.sources:
            name, color, width = SOURCE_STYLES[source]
            rr.log(
                signal_entity_path(signal, source),
                rr.SeriesLines(colors=color, names=name, widths=width),
                static=True,
            )
    if not evaluation_enabled:
        return
    for key, path in EVALUATION_OVERVIEW_PATHS.items():
        name, color, width = EVALUATION_STYLES[key]
        rr.log(path, rr.SeriesLines(colors=color, names=name, widths=width), static=True)
    for signal in config.signals.enabled:
        rr.log(
            evaluation_error_path(signal),
            rr.SeriesLines(colors=[239, 68, 68], names="Absolute normalized error", widths=2.0),
            static=True,
        )
