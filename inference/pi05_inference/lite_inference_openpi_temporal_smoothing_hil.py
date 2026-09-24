"""
ROS2 时序平滑推理客户端 — Human-in-the-Loop (HIL) 版本
基于 lite_inference_openpi_temporal_smoothing.py，增加 HIL 接管与 VLA 授权机制：

HIL 逻辑：
  - --start-armed 保留一次启动授权；运行中只由 left_button[15] 上升沿切换控制权
  - VLA 授权和 AUTONOMY_JOINT/指定二级模式同时有效时，才允许推理 + 下发
  - 外部任务/模式变化撤销授权，清空动作；任务结束后仍须按左摇杆恢复
  - /hil/request_vla_autonomy_grant 保留兼容接口，但不再授予控制权

数据维度与顺序与 lite_inference_openpi_temporal_smoothing.py 完全一致，不做修改：
  16D layout: [left_j0..6, l_grip(归一化0~1), right_j0..6, r_grip(归一化0~1)]
"""
from __future__ import annotations

import argparse
import os
import signal
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from arm_interfaces.msg import ArmStatus, AutonomyArmCommand

try:
    from arm_interfaces.msg import MasterControllerCommand
    from arm_interfaces.srv import TrajFollowMode, TrajFollowSecondaryMode
    from std_srvs.srv import Trigger
except ImportError:
    MasterControllerCommand = None
    TrajFollowMode = None
    TrajFollowSecondaryMode = None
    Trigger = None
from sensor_msgs.msg import CompressedImage, Image

from openpi_client import image_tools, websocket_client_policy

# ── 关节限位与原文件完全一致 ──────────────────────────────────────────────────
JOINT_MIN = np.asarray([-2.94, -1.62, -2.94, -3.00, -2.94, -0.35, -1.70], dtype=np.float32)
JOINT_MAX = np.asarray([ 2.94,  1.62,  2.94,  0.01,  2.94,  1.4,  1.70], dtype=np.float32)
GRIP_MIN, GRIP_MAX = 0.0, 100.0

# ── 默认初始位 ────────────────────────────────────────────────────────────────
DEFAULT_INIT_JOINT_RAD = np.asarray(
    [-2.188486, 0.993911, 1.780427, -2.382301, 0.598698, 0.170221, 0.283382],
    dtype=np.float32,
)
DEFAULT_INIT_GRIPPER = np.asarray([100.0, 100.0], dtype=np.float32)

# ── 16D 动作/状态字段布局（与训练及原文件完全一致）────────────────────────────
# state / action_chunk[:, 0:7]  左臂关节   [:, 7]  左夹爪  (归一化 [0,1])
#                    [:, 8:15] 右臂关节   [:, 15] 右夹爪  (归一化 [0,1])

MAX_DEQUE = 1000

# 关节死区阈值（与原文件一致）
JOINT_DEADBAND = 0.005

# ── HIL 模式常量 ──────────────────────────────────────────────────────────────
AUTONOMY_JOINT_MODE   = 1   # VLA 自主关节指令模式
TELEOPERATION_VR_MODE = 5   # 人类 VR 遥操作模式（HIL 接管）

# ── 全局事件 ──────────────────────────────────────────────────────────────────
shutdown_event = threading.Event()   # 进程退出信号


def _on_sigint(signum, frame):
    try:
        shutdown_event.set()
    except Exception:
        pass


# ── HIL 辅助函数 ──────────────────────────────────────────────────────────────

def _primary_mode(arm_status: ArmStatus) -> int:
    return int(arm_status.other_status[0]) if len(arm_status.other_status) > 0 else -1


def _secondary_mode(arm_status: ArmStatus) -> int:
    return int(arm_status.other_status[1]) if len(arm_status.other_status) > 1 else -1


def wait_for_vla_authorization(ros: "RosOperator", sleep_s: float = 0.05) -> ArmStatus | None:
    """只等待显式授权和模式确认，不主动修复或切换底层模式。"""
    last_modes = None
    while not shutdown_event.is_set():
        arm = ros.get_latest_arm_status()
        if arm is not None and ros.vla_command_allowed(arm):
            if last_modes is not None:
                print("[HIL] 模式已就绪（AUTONOMY_JOINT），恢复下发")
            return arm
        modes = None if arm is None else (_primary_mode(arm), _secondary_mode(arm))
        if modes != last_modes:
            current = "暂无 arm_status" if modes is None else f"current={modes[0]}/{modes[1]}"
            print(
                f"[HIL] 等待左摇杆授权及 AUTONOMY_JOINT "
                f"secondary={ros.args.autonomy_secondary_mode}（{current}）..."
            )
            last_modes = modes
        time.sleep(sleep_s)
    return None


# ── 参数 ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--host", required=True)
    p.add_argument("--port", type=int, default=8008)
    p.add_argument("--task", required=True)
    p.add_argument("--hz",   type=float, default=30.0, help="主循环（下发）频率")
    p.add_argument("--resize", type=int, nargs=2, default=[224, 224], metavar=("H", "W"))
    p.add_argument("--arm-topic",   default="/arm_status")
    p.add_argument("--top-topic",   default="/camera_dcw2/custom_cam_color_1M")
    p.add_argument("--left-topic",  default="/camera_dcl_left/custom_cam_color_1M")
    p.add_argument("--right-topic", default="/camera_dcl_right/custom_cam_color_1M")
    p.add_argument("--cam-type",    default="custom_image1m",
                   choices=("custom_image1m", "custom_image4m", "sensor_image", "compressed_image"))
    p.add_argument("--cmd-topic", default=None,
                   help="动作发布话题；replay 模式默认 /execute_arm_cmd_inference_dummy")
    p.add_argument("--mode-service",   default="/switch_arm_executer_mode")
    p.add_argument("--secondary-mode-service", default="/switch_arm_secondary_mode")
    p.add_argument("--autonomy-secondary-mode", type=int, default=1, choices=(1, 2),
                   help="/execute_arm_cmd 配套的二级模式：1=关节指令模式 2=EEF指令模式")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--replay-mode", action="store_true",
                   help="离线回放模式：跳过 HIL 授权和模式服务，只允许发布到安全话题")
    p.add_argument("--init-pose-hold-steps", type=int, default=20)
    p.add_argument("--shutdown-hold-steps",  type=int, default=4)

    # ── 时序平滑参数（与原文件一致）──────────────────────────────────────────
    p.add_argument("--inference-rate", type=float, default=3.0,
                   help="后台非阻塞推理线程的目标频率（Hz）")
    p.add_argument("--latency-k", type=int, default=8,
                   help="新 chunk 到达时，按已下发步数 k 做延迟裁剪的上限步数")
    p.add_argument("--min-smooth-steps", type=int, default=8,
                   help="新旧 chunk 重叠平滑区间的最小长度 m")
    p.add_argument("--buffer-max-chunks", type=int, default=10)
    p.add_argument("--exp-decay-alpha", type=float, default=0.25)

    # ── HIL 参数 ──────────────────────────────────────────────────────────────
    p.add_argument("--vla-grant-service", default="/hil/request_vla_autonomy_grant",
                   help="兼容 Trigger 服务名；调用会提示使用左摇杆，不授予控制权")
    p.add_argument("--start-armed", action="store_true",
                   help="启动时自动请求一次 VLA 控制权；运行期恢复仍须按左摇杆")
    p.add_argument("--grant-activation-grace-s", type=float, default=1.0,
                   help="旧参数，仅为命令行兼容保留；不再以宽限期自动授权")

    # ── /tele_vr_cmd 模式切换（left_button[15] 的上升沿触发）────────────────────
    p.add_argument("--tele-vr-cmd-topic", default="/tele_vr_cmd",
                   help="Pico 遥操话题；left_button[15] 的上升沿触发一次模式切换")
    p.add_argument("--mode-switch-service-timeout-s", type=float, default=0.10,
                   help="左摇杆触发的模式切换服务发现超时（秒）")
    p.add_argument("--mode-switch-confirm-timeout-s", type=float, default=2.0,
                   help="等待模式服务成功和新 arm_status 确认的总超时（秒）")
    args = p.parse_args()
    if args.mode_switch_confirm_timeout_s <= 0:
        p.error("--mode-switch-confirm-timeout-s must be positive")
    if args.cmd_topic is None:
        args.cmd_topic = "/execute_arm_cmd_inference_dummy" if args.replay_mode else "/execute_arm_cmd"
    if args.replay_mode:
        if args.dry_run:
            p.error("--replay-mode and --dry-run are mutually exclusive")
        if args.start_armed:
            p.error("--start-armed is not used in --replay-mode")
        if args.cmd_topic == "/execute_arm_cmd":
            p.error("--replay-mode refuses to publish to the real /execute_arm_cmd topic")
        if os.environ.get("ROS_DOMAIN_ID") != "100":
            p.error("--replay-mode requires ROS_DOMAIN_ID=100")
    return args


# ── 图像解码（与原文件完全一致）──────────────────────────────────────────────

def _decode_img(msg, cam_type: str) -> np.ndarray:
    if cam_type == "compressed_image":
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    if cam_type == "sensor_image":
        h, w, step = int(msg.height), int(msg.width), int(msg.step)
        rows = np.frombuffer(msg.data, np.uint8)[:h * step].reshape(h, step)
        img  = rows[:, :w * 3].reshape(h, w, 3)
        return img.copy() if msg.encoding.lower() == "rgb8" else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    rows  = np.asarray(msg.data[:h * step], dtype=np.uint8).reshape(h, step)
    enc_b = bytes(int(x) for x in msg.encoding.data)
    zero  = enc_b.find(b"\x00")
    enc   = (enc_b[:zero] if zero >= 0 else enc_b).decode("utf-8", errors="ignore").lower()
    img   = rows[:, :w * 3].reshape(h, w, 3)
    return img.copy() if enc == "rgb8" else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def _cam_cls(cam_type: str):
    if cam_type == "sensor_image":     return Image
    if cam_type == "compressed_image": return CompressedImage
    if cam_type == "custom_image4m":
        from custom_image_msg.msg import Image4m; return Image4m
    from custom_image_msg.msg import Image1m; return Image1m


# ── RosOperator（含 HIL 授权逻辑）────────────────────────────────────────────

class RosOperator(Node):
    """
    在 lite_inference_openpi_temporal_smoothing.py 的 RosOperator 基础上增加：
      - 左摇杆独占运行期控制权切换，grant 服务仅保留兼容响应
      - _vla_authorized / _vla_authority_epoch 授权状态管理
      - 模式确认、外部中断撤权、旧推理/动作失效
    数据维度与顺序与原文件完全一致。
    """

    def __init__(self, args):
        node_name = "fastwam_temporal_smoothing_replay_client" if args.replay_mode \
            else "fastwam_temporal_smoothing_hil_client"
        super().__init__(node_name)
        self.args = args

        self.arm_dq:   deque = deque()
        self.top_dq:   deque = deque()
        self.left_dq:  deque = deque()
        self.right_dq: deque = deque()
        self.lock = threading.Lock()

        # ── HIL 授权状态 ──────────────────────────────────────────────────────
        # 锁顺序：authority -> arm deque / action buffer。下发也在 authority 锁内复核。
        self._vla_authority_lock   = threading.RLock()
        self._vla_authorized       = False
        self._vla_authority_epoch  = 0
        self._control_modes       = None
        self._arm_status_seq      = 0
        self._mode_transition     = None
        self._startup_autonomy_pending = bool(args.start_armed)
        self._action_buffer       = None

        cam_cls = _cam_cls(args.cam_type)
        self.create_subscription(ArmStatus, args.arm_topic,   self._arm_cb,                                    10)
        self.create_subscription(cam_cls,   args.top_topic,   lambda m: self._img_cb(self.top_dq,   m), qos_profile_sensor_data)
        self.create_subscription(cam_cls,   args.left_topic,  lambda m: self._img_cb(self.left_dq,  m), qos_profile_sensor_data)
        self.create_subscription(cam_cls,   args.right_topic, lambda m: self._img_cb(self.right_dq, m), qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(AutonomyArmCommand, args.cmd_topic, 10)
        self.mode_cli = None
        self.secondary_mode_cli = None
        self._mode_button_active = None
        self._tele_vr_cmd_discovery_timer = None
        self._mode_switch_timer = None

        if args.replay_mode:
            self.get_logger().warning(
                f"REPLAY MODE: HIL services disabled; predictions -> {args.cmd_topic}")
        else:
            missing = [
                name for name, value in (
                    ("MasterControllerCommand", MasterControllerCommand),
                    ("TrajFollowMode", TrajFollowMode),
                    ("Trigger", Trigger),
                ) if value is None
            ]
            if missing:
                raise RuntimeError(
                    "full arm_interfaces installation is required outside replay mode; "
                    f"missing: {', '.join(missing)}"
                )
            self.mode_cli = self.create_client(TrajFollowMode, args.mode_service)
            if TrajFollowSecondaryMode is not None:
                self.secondary_mode_cli = self.create_client(
                    TrajFollowSecondaryMode, args.secondary_mode_service)
            else:
                self.get_logger().warning(
                    "TrajFollowSecondaryMode unavailable; secondary mode switch disabled.")

            # 兼容服务不再允许外部调用绕过左摇杆。
            self.create_service(Trigger, args.vla_grant_service, self._on_vla_autonomy_grant)
            self.get_logger().info(
                f"HIL grant service ready: {args.vla_grant_service} | start_armed={args.start_armed}")

            # /tele_vr_cmd 持续高频发布，因此只在 left_button[15] 的上升沿切换一次。
            # 首帧为 true 时不能证明发生了按下，先观察一次 false 才接受上升沿。
            self.create_subscription(
                MasterControllerCommand, args.tele_vr_cmd_topic, self._on_tele_vr_cmd,
                qos_profile_sensor_data)
            self.get_logger().info(
                f"HIL mode command topic ready: {args.tele_vr_cmd_topic} | left_button[15]")
            self._tele_vr_cmd_discovery_timer = self.create_timer(
                2.0, self._check_tele_vr_cmd_publisher)
            self._mode_switch_timer = self.create_timer(0.05, self._check_mode_switch_timeout)

    # ── HIL 授权管理 ──────────────────────────────────────────────────────────

    def _on_vla_autonomy_grant(self, request: Trigger.Request, response: Trigger.Response):
        response.success = False
        response.message = "VLA control requires a left_button[15] rising edge; external grant disabled"
        self.get_logger().warning("[HIL] 外部 grant 不授予控制权，请按左摇杆恢复模型")
        return response

    def _set_authority_locked(self, authorized: bool) -> None:
        """调用方持有 authority 锁；每次交接都使旧推理/动作失效。"""
        self._vla_authorized = authorized
        self._vla_authority_epoch += 1
        if self._action_buffer is not None:
            self._action_buffer.clear()

    @property
    def vla_authority_epoch(self) -> int:
        with self._vla_authority_lock:
            return self._vla_authority_epoch

    def _vla_allowed_locked(self) -> bool:
        return (
            self._vla_authorized and self._mode_transition is None and
            self._control_modes == (AUTONOMY_JOINT_MODE, int(self.args.autonomy_secondary_mode))
        )

    def vla_command_allowed(self, arm_status: ArmStatus) -> bool:
        with self._vla_authority_lock:
            return self._vla_allowed_locked() and (
                _primary_mode(arm_status), _secondary_mode(arm_status)
            ) == self._control_modes

    def inference_epoch(self) -> int | None:
        """无控制权时不推理；dry-run/replay 不申请真实底层控制权。"""
        with self._vla_authority_lock:
            if self.args.dry_run or self.args.replay_mode or self._vla_allowed_locked():
                return self._vla_authority_epoch
            return None

    def attach_action_buffer(self, buffer: "StreamActionBuffer") -> None:
        with self._vla_authority_lock:
            self._action_buffer = buffer
            buffer.clear()

    def accept_inference_result(self, actions, epoch: int, buffer: "StreamActionBuffer") -> bool:
        # 与撤销授权、清空动作使用同一把锁，避免检查通过后才写入过时 chunk。
        with self._vla_authority_lock:
            if self.inference_epoch() != epoch:
                return False
            buffer.integrate_new_chunk(
                actions, max_k=self.args.latency_k, min_m=self.args.min_smooth_steps)
            return True

    def pop_authorized_action(self, buffer: "StreamActionBuffer"):
        with self._vla_authority_lock:
            epoch = self.inference_epoch()
            if epoch is None:
                return None, None
            return buffer.pop_next_action(), epoch

    # ── 左摇杆：唯一运行期控制权请求入口 ────────────────────────────────────

    def _on_tele_vr_cmd(self, msg: MasterControllerCommand) -> None:
        if len(msg.left_button) <= 15:
            return  # 无效帧不能当作松开，否则会制造假上升沿。
        button_active = bool(msg.left_button[15])
        rising_edge = button_active and self._mode_button_active is False
        self._mode_button_active = button_active
        if rising_edge:
            self.get_logger().info("[HIL] 左摇杆按下：left_button[15] false -> true")
            self.toggle_ctrl_mode()

    def _check_tele_vr_cmd_publisher(self) -> None:
        infos = self.get_publishers_info_by_topic(self.args.tele_vr_cmd_topic)
        if infos:
            self.get_logger().info(
                f"[HIL] 已发现 {self.args.tele_vr_cmd_topic} 发布者，左摇杆模式切换可用")
        else:
            self.get_logger().warning(
                f"[HIL] 未发现 {self.args.tele_vr_cmd_topic} 发布者；"
                "请启动 Pico 站遥操终端/robot_communication，并确认双方 ROS_DOMAIN_ID 与 DDS 配置一致")
        self.destroy_timer(self._tele_vr_cmd_discovery_timer)

    def start_autonomy_once(self) -> None:
        with self._vla_authority_lock:
            if not self._startup_autonomy_pending:
                return
            self._startup_autonomy_pending = False
            transition = self._begin_mode_transition_locked(True, "--start-armed 启动")
        self._send_mode_transition(transition)

    def toggle_ctrl_mode(self) -> None:
        with self._vla_authority_lock:
            # 用户已按过摇杆时，不允许稍后的启动流程覆盖其选择。
            self._startup_autonomy_pending = False
            transition = self._begin_mode_transition_locked(
                not self._vla_allowed_locked(), "左摇杆 left_button[15]")
        self._send_mode_transition(transition)

    def _begin_mode_transition_locked(self, to_vla: bool, source: str):
        if shutdown_event.is_set():
            return None
        if self.args.dry_run or self.args.replay_mode:
            self.get_logger().info(f"[HIL] dry-run/replay：忽略底层模式请求（{source}）")
            return None
        if self._mode_transition is not None:
            self.get_logger().warning("[HIL] 切换确认中，忽略重复按下；完成后请松开再按")
            return None
        if self._control_modes is None or self._control_modes[0] not in (0, 1, 5):
            self.get_logger().warning(f"[HIL] 跳过切换：模式未知或不支持 {self._control_modes}")
            return None
        self._set_authority_locked(False)
        transition = {
            "target": (AUTONOMY_JOINT_MODE, int(self.args.autonomy_secondary_mode))
                      if to_vla else (TELEOPERATION_VR_MODE, 0),
            "source": source,
            "deadline": time.monotonic() + float(self.args.mode_switch_confirm_timeout_s),
            "sent": False,
            "status_seq": self._arm_status_seq,
            "matched": False,
            "response_ok": False,
        }
        self._mode_transition = transition
        self.get_logger().info(
            f"[HIL] 请求控制权切换：{self._control_modes} -> {transition['target']} | {source}")
        return transition

    def _send_mode_transition(self, transition) -> None:
        if transition is None:
            return
        primary, secondary = transition["target"]
        if primary == AUTONOMY_JOINT_MODE:
            client = self.secondary_mode_cli
            req = TrajFollowSecondaryMode.Request() if client is not None else None
            if req is not None:
                req.trajectory_following_secondary_mode = secondary
        else:
            client = self.mode_cli
            req = TrajFollowMode.Request()
        try:
            if client is None or not client.wait_for_service(
                    timeout_sec=float(self.args.mode_switch_service_timeout_s)):
                with self._vla_authority_lock:
                    self._fail_mode_transition_locked(transition, "模式服务不可用")
                return
            req.trajectory_following_mode = primary
            with self._vla_authority_lock:
                if self._mode_transition is not transition:
                    return
                if time.monotonic() >= transition["deadline"]:
                    self._fail_mode_transition_locked(transition, "模式请求超时")
                    return
                transition["sent"] = True
                transition["status_seq"] = self._arm_status_seq
                future = client.call_async(req)
            future.add_done_callback(lambda f: self._on_mode_service_done(transition, f))
        except Exception as exc:
            with self._vla_authority_lock:
                self._fail_mode_transition_locked(transition, f"模式服务异常：{exc}")

    def _on_mode_service_done(self, transition, future) -> None:
        with self._vla_authority_lock:
            if self._mode_transition is not transition:
                return  # 超时或失效的请求即使迟到成功，也不能重新授权。
            try:
                response = future.result()
                ok = bool(getattr(response, "result", False))
            except Exception as exc:
                self._fail_mode_transition_locked(transition, f"模式服务异常：{exc}")
                return
            if not ok:
                self._fail_mode_transition_locked(transition, "模式服务返回失败")
                return
            transition["response_ok"] = True
            self._complete_mode_transition_locked()

    def _complete_mode_transition_locked(self) -> None:
        transition = self._mode_transition
        if transition is None:
            return
        if shutdown_event.is_set():
            self._fail_mode_transition_locked(transition, "程序退出，取消授权确认")
            return
        if time.monotonic() >= transition["deadline"]:
            self._fail_mode_transition_locked(transition, "等待服务成功和 arm_status 确认超时")
            return
        if not (transition["response_ok"] and transition["matched"] and
                self._control_modes == transition["target"]):
            return
        self._mode_transition = None
        to_vla = transition["target"][0] == AUTONOMY_JOINT_MODE
        self._set_authority_locked(to_vla)
        state = "模型授权已生效" if to_vla else "已进入人工控制，模型保持暂停"
        self.get_logger().info(f"[HIL] {state} | {transition['source']}")

    def _fail_mode_transition_locked(self, transition, reason: str) -> None:
        if self._mode_transition is not transition:
            return
        self._mode_transition = None
        self._set_authority_locked(False)
        self.get_logger().warning(f"[HIL] {reason}；模型保持暂停，请重新按左摇杆")

    def _check_mode_switch_timeout(self) -> None:
        with self._vla_authority_lock:
            self._complete_mode_transition_locked()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _arm_cb(self, msg: ArmStatus) -> None:
        # 在 ROS 回调中立即捕获外部中断，避免主循环漏掉短暂的 1/0 或 IDLE。
        with self._vla_authority_lock:
            self._control_modes = (_primary_mode(msg), _secondary_mode(msg))
            self._arm_status_seq += 1
            with self.lock:
                if len(self.arm_dq) >= MAX_DEQUE:
                    self.arm_dq.popleft()
                self.arm_dq.append(msg)
            transition = self._mode_transition
            if transition is not None:
                if transition["sent"] and self._arm_status_seq > transition["status_seq"]:
                    if self._control_modes == transition["target"]:
                        transition["matched"] = True
                    elif transition["matched"]:
                        self._fail_mode_transition_locked(transition, "确认期间底层模式再次改变")
                self._complete_mode_transition_locked()
            elif self._vla_authorized and not self._vla_allowed_locked():
                self._set_authority_locked(False)
                self.get_logger().warning(
                    f"[HIL] 外部任务/模式变化中断模型：{self._control_modes}；"
                    "任务结束不会自动恢复，请按左摇杆")

    def _img_cb(self, dq: deque, msg) -> None:
        try:
            img = _decode_img(msg, self.args.cam_type)
        except Exception as e:
            self.get_logger().warning(f"img decode: {e}")
            return
        with self.lock:
            if len(dq) >= MAX_DEQUE: dq.popleft()
            dq.append((_stamp(msg), img))

    # ── get_frame（与原文件完全一致）──────────────────────────────────────────

    def get_frame(self):
        with self.lock:
            if not self.arm_dq or not self.top_dq or not self.left_dq or not self.right_dq:
                return None
            frame_time = min(self.top_dq[-1][0], self.left_dq[-1][0], self.right_dq[-1][0])
            if _stamp(self.arm_dq[-1]) < frame_time: return None
            if self.top_dq[-1][0]      < frame_time: return None
            if self.left_dq[-1][0]     < frame_time: return None
            if self.right_dq[-1][0]    < frame_time: return None
            while len(self.arm_dq)   > 1 and _stamp(self.arm_dq[0])  < frame_time: self.arm_dq.popleft()
            while len(self.top_dq)   > 1 and self.top_dq[0][0]       < frame_time: self.top_dq.popleft()
            while len(self.left_dq)  > 1 and self.left_dq[0][0]      < frame_time: self.left_dq.popleft()
            while len(self.right_dq) > 1 and self.right_dq[0][0]     < frame_time: self.right_dq.popleft()
            arm          = self.arm_dq.popleft()
            _, img_top   = self.top_dq.popleft()
            _, img_left  = self.left_dq.popleft()
            _, img_right = self.right_dq.popleft()

        # 16D state，布局与训练一致：[left_j0..6, l_grip, right_j0..6, r_grip]
        lj = [float(j.joint_status[0]) for j in arm.left_arm]
        rj = [float(j.joint_status[0]) for j in arm.right_arm]
        lg_raw = float(arm.gripper[0].joint_status[0]) if len(arm.gripper) > 0 else 0.0
        rg_raw = float(arm.gripper[1].joint_status[0]) if len(arm.gripper) > 1 else 0.0
        lg = float(np.clip(lg_raw, 0.0, 100.0)) / 100.0
        rg = float(np.clip(rg_raw, 0.0, 100.0)) / 100.0
        vec = (lj[:7] + [lg] + rj[:7] + [rg]) if (len(lj) >= 7 and len(rj) >= 7) \
              else (lj[:6] + [0.0, lg] + rj[:6] + [0.0, rg])
        state = np.asarray(vec, dtype=np.float32)
        return img_top, img_left, img_right, state

    # ── publish（与原文件一致，关节越限 clip，夹爪 <3 置 0）──────────────────

    def _publish_authorized(self, msg: AutonomyArmCommand, epoch: int) -> bool:
        # 与 _arm_cb 撤权互斥；即使 action 取出后发生中断，也不能继续下发。
        with self._vla_authority_lock:
            if self.args.dry_run or epoch != self._vla_authority_epoch:
                return False
            if not self.args.replay_mode and not self._vla_allowed_locked():
                return False
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(msg)
            return True

    def publish(self, action: np.ndarray, epoch: int) -> bool:
        lj = np.array(action[0:7],  dtype=np.float32)
        rj = np.array(action[8:15], dtype=np.float32)
        lg = float(np.clip(float(action[7])  * 100.0, GRIP_MIN, GRIP_MAX))
        rg = float(np.clip(float(action[15]) * 100.0, GRIP_MIN, GRIP_MAX))
        # if lg < 15.0: lg = 0.0
        # if rg < 15.0: rg = 0.0

        for side, joints in (("left", lj), ("right", rj)):
            for i, (v, lo, hi) in enumerate(zip(joints, JOINT_MIN, JOINT_MAX)):
                if v < lo - 1e-6 or v > hi + 1e-6:
                    clipped = float(np.clip(v, lo, hi))
                    self.get_logger().warning(
                        f"[LIMIT] {side}_j{i}={v:.4f} ∉ [{lo:.4f},{hi:.4f}] → clipped to {clipped:.4f}"
                    )
                    joints[i] = clipped

        msg = AutonomyArmCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        lf = "left_joint_command"  if hasattr(msg, "left_joint_command")  else "left_command"
        rf = "right_joint_command" if hasattr(msg, "right_joint_command") else "right_command"
        for i in range(7):
            getattr(msg, lf)[i] = float(lj[i])
            getattr(msg, rf)[i] = float(rj[i])
        msg.gripper_command[0] = lg
        msg.gripper_command[1] = rg
        for i in range(2, 7):
            msg.gripper_command[i] = 0.0

        published = self._publish_authorized(msg, epoch)
        if published:
            print(f"[send]  left_joint={np.array2string(lj, precision=4)} "
                  f"right_joint={np.array2string(rj, precision=4)} "
                  f"gripper=[{lg:.2f}, {rg:.2f}]")
        return published

    def publish_init_pose(self, hold_steps: int, dt: float) -> None:
        if hold_steps <= 0:
            self.get_logger().info("skip init pose (--init-pose-hold-steps=0)")
            return
        epoch = self.inference_epoch()
        if epoch is None:
            return
        msg = AutonomyArmCommand()
        lf = "left_joint_command"  if hasattr(msg, "left_joint_command")  else "left_command"
        rf = "right_joint_command" if hasattr(msg, "right_joint_command") else "right_command"
        for i in range(7):
            getattr(msg, lf)[i] = float(DEFAULT_INIT_JOINT_RAD[i])
            getattr(msg, rf)[i] = float(DEFAULT_INIT_JOINT_RAD[i])
        msg.gripper_command[0] = float(DEFAULT_INIT_GRIPPER[0])
        msg.gripper_command[1] = float(DEFAULT_INIT_GRIPPER[1])
        for i in range(2, 7):
            msg.gripper_command[i] = 0.0
        self.get_logger().info(f"Publishing init pose for {hold_steps} steps @ {1/dt:.0f}Hz ...")
        for _ in range(hold_steps):
            if shutdown_event.is_set() or not self._publish_authorized(msg, epoch):
                self.get_logger().warning("[HIL] 初始位下发中断，等待左摇杆重新授权")
                return
            time.sleep(dt)
        self.get_logger().info("Init pose done.")

    def publish_shutdown_hold(self, hold_steps: int, dt: float) -> None:
        if hold_steps <= 0:
            return
        epoch = self.inference_epoch()
        if epoch is None:
            return
        with self.lock:
            if not self.arm_dq:
                self.get_logger().warning("No arm status for shutdown hold, skip.")
                return
            arm = self.arm_dq[-1]
        lj = [float(j.joint_status[0]) for j in arm.left_arm]
        rj = [float(j.joint_status[0]) for j in arm.right_arm]
        if len(lj) < 7 or len(rj) < 7:
            self.get_logger().warning("Arm status has <7 joints, skip shutdown hold.")
            return
        lg_raw = float(arm.gripper[0].joint_status[0]) if len(arm.gripper) > 0 else 0.0
        rg_raw = float(arm.gripper[1].joint_status[0]) if len(arm.gripper) > 1 else 0.0
        lg = float(np.clip(lg_raw, GRIP_MIN, GRIP_MAX))
        rg = float(np.clip(rg_raw, GRIP_MIN, GRIP_MAX))
        msg = AutonomyArmCommand()
        lf = "left_joint_command"  if hasattr(msg, "left_joint_command")  else "left_command"
        rf = "right_joint_command" if hasattr(msg, "right_joint_command") else "right_command"
        for i in range(7):
            getattr(msg, lf)[i] = float(lj[i])
            getattr(msg, rf)[i] = float(rj[i])
        msg.gripper_command[0] = lg
        msg.gripper_command[1] = rg
        for i in range(2, 7):
            msg.gripper_command[i] = 0.0
        self.get_logger().warning(f"Shutdown hold: publishing {hold_steps} steps @ {1/dt:.0f}Hz ...")
        for idx in range(hold_steps):
            if not self._publish_authorized(msg, epoch):
                return
            if idx + 1 < hold_steps:
                time.sleep(dt)
        self.get_logger().warning("Shutdown hold done.")

    def get_latest_arm_status(self) -> ArmStatus | None:
        with self.lock:
            return self.arm_dq[-1] if self.arm_dq else None

# ── 工具 ──────────────────────────────────────────────────────────────────────

def _stamp(msg) -> float:
    try:
        s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return s if s > 1.0 else time.monotonic()
    except Exception:
        return time.monotonic()


def build_observation(ros: RosOperator, task: str, resize: tuple) -> dict | None:
    frame = ros.get_frame()
    if frame is None:
        return None
    img_top, img_left, img_right, state = frame
    h, w = resize
    return {
        "images": {
            "top_head":   image_tools.resize_with_pad(img_top,   h, w).transpose(2, 0, 1),
            "hand_right": image_tools.resize_with_pad(img_right, h, w).transpose(2, 0, 1),
            "hand_left":  image_tools.resize_with_pad(img_left,  h, w).transpose(2, 0, 1),
        },
        "state":  state,
        "prompt": task,
    }


# ── StreamActionBuffer（在原版基础上新增 clear() 方法）──────────────────────

class StreamActionBuffer:
    """
    与原文件完全一致，新增 clear() 用于 HIL 接管结束后丢弃过时 chunk。
    """
    def __init__(self, max_chunks=10, decay_alpha=0.25, state_dim=16, smooth_method="temporal"):
        self.chunks      = deque()
        self.max_chunks  = max_chunks
        self.lock        = threading.Lock()
        self.decay_alpha = float(decay_alpha)
        self.state_dim   = state_dim
        self.smooth_method = smooth_method
        self.cur_chunk   = deque()
        self.k           = 0
        self.last_action = None

    def integrate_new_chunk(self, actions_chunk: np.ndarray, max_k: int, min_m: int = 8):
        with self.lock:
            if actions_chunk is None or len(actions_chunk) == 0:
                return
            max_k  = max(0, int(max_k))
            min_m  = max(1, int(min_m))
            drop_n = min(self.k, max_k)
            if drop_n >= len(actions_chunk):
                return
            new_chunk = [a.copy() for a in actions_chunk[drop_n:]]
            if len(self.cur_chunk) == 0 and self.last_action is not None:
                old_list = [np.asarray(self.last_action, dtype=float).copy() for _ in range(min_m)]
                self.last_action = None
            else:
                old_list = list(self.cur_chunk)
                if len(old_list) > 0 and len(old_list) < min_m:
                    tail = np.asarray(old_list[-1], dtype=float).copy()
                    old_list.extend([tail.copy() for _ in range(min_m - len(old_list))])
                elif len(old_list) == 0:
                    self.cur_chunk = deque(new_chunk, maxlen=None)
                    self.k = 0
                    return
            new_list = list(new_chunk)
            overlap_len = min(len(old_list), len(new_list))
            if overlap_len <= 0:
                self.cur_chunk = deque(new_list, maxlen=None)
                self.k = 0
                return
            if len(old_list) > len(new_list):
                old_list    = old_list[:len(new_list)]
                overlap_len = len(new_list)
            if overlap_len == 1:
                w_old = np.array([1.0], dtype=float)
            else:
                w_old = np.linspace(1.0, 0.0, overlap_len, dtype=float)
            w_new = 1.0 - w_old
            smoothed = [
                (w_old[i] * np.asarray(old_list[i], dtype=float) +
                 w_new[i] * np.asarray(new_list[i], dtype=float))
                for i in range(overlap_len)
            ]
            combined = smoothed + new_list[overlap_len:]
            self.cur_chunk = deque([a.copy() for a in combined], maxlen=None)
            self.k = 0

    def has_any(self) -> bool:
        with self.lock:
            return len(self.cur_chunk) > 0

    def pop_next_action(self) -> np.ndarray | None:
        with self.lock:
            if len(self.cur_chunk) == 0:
                return None
            if len(self.cur_chunk) == 1:
                self.last_action = np.asarray(self.cur_chunk[0], dtype=float).copy()
            act = np.asarray(self.cur_chunk.popleft(), dtype=float)
            self.k += 1
            return act

    def clear(self) -> None:
        """丢弃所有待执行动作。HIL 接管结束或授权变化后调用，防止下发过时 chunk。"""
        with self.lock:
            self.cur_chunk.clear()
            self.k           = 0
            self.last_action = None


# ── 后台推理线程（HIL 版：接管期间暂停推理）──────────────────────────────────

def inference_fn_non_blocking_fast(
    args, policy, ros: RosOperator, resize: tuple, stream_buffer: StreamActionBuffer
):
    """
    推理前记录控制权版本；结果返回时再次核对授权和版本。
    即使推理期间发生中断并重新授权，旧结果也不能混入新动作。
    """
    period = 1.0 / max(args.inference_rate, 1e-6)
    while not shutdown_event.is_set():
        t0 = time.time()
        try:
            epoch = ros.inference_epoch()
            if epoch is None:
                time.sleep(0.1)
                continue

            obs = None
            while obs is None and not shutdown_event.is_set():
                if ros.inference_epoch() != epoch:
                    break
                obs = build_observation(ros, args.task, resize)
                if obs is None:
                    time.sleep(0.005)
            if obs is None:
                continue
            if shutdown_event.is_set() or ros.inference_epoch() != epoch:
                continue

            print(f"[obs-bg]   state={np.array2string(obs['state'], precision=3)}")

            result   = policy.infer({"images": obs["images"], "state": obs["state"], "prompt": obs["prompt"]})
            actions  = np.asarray(result["actions"], dtype=np.float32)
            infer_ms = result.get('server_timing', {}).get('infer_ms', 0)
            rtt_ms   = result.get('client_timing', {}).get('rtt_ms', 0)
            print(f"[infer-bg] chunk={len(actions)} infer={infer_ms:.0f}ms rtt={rtt_ms:.0f}ms")

            if actions is not None and len(actions) > 0:
                if shutdown_event.is_set() or not ros.accept_inference_result(actions, epoch, stream_buffer):
                    print("[infer-bg] 控制权已改变，丢弃过时推理结果")
            else:
                print("[infer-bg] empty actions, skip")
        except Exception as e:
            print(f"[infer-bg][WARN] {e}")
            time.sleep(0.05)
            continue

        elapsed = time.time() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


def start_inference_thread(args, policy, ros, resize, stream_buffer) -> threading.Thread:
    t = threading.Thread(
        target=inference_fn_non_blocking_fast,
        args=(args, policy, ros, resize, stream_buffer),
        daemon=True,
    )
    t.start()
    return t


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    args   = parse_args()
    resize = tuple(args.resize)
    dt     = 1.0 / args.hz

    shutdown_event.clear()
    signal.signal(signal.SIGINT, _on_sigint)

    rclpy.init()
    ros = RosOperator(args)
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros,), daemon=True)
    spin_thread.start()

    print(f"Connecting to policy server {args.host}:{args.port} ...")
    policy = websocket_client_policy.WebsocketClientPolicy(args.host, args.port)
    print(f"Server metadata: {policy.get_server_metadata()}")

    print("Waiting for ROS observations ...")
    while (not shutdown_event.is_set() and
           not (ros.arm_dq and ros.top_dq and ros.left_dq and ros.right_dq)):
        time.sleep(0.05)
    if shutdown_event.is_set():
        try:
            policy._ws.close()
        except Exception:
            pass
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        return
    print("Ready.")

    if not args.dry_run and not args.replay_mode:
        if args.start_armed:
            ros.start_autonomy_once()
        arm_status = wait_for_vla_authorization(ros)
        if arm_status is None:
            rclpy.shutdown()
            spin_thread.join(timeout=1.0)
            return
        ros.publish_init_pose(args.init_pose_hold_steps, dt)

    if args.replay_mode:
        print("[REPLAY] 观测已就绪，自动开始推理；预测动作仅发布到安全 Dummy 话题。")
    else:
        print("\n" + "─" * 60)
        print("  初始化流程结束（temporal smoothing + HIL 模式）。")
        if args.start_armed:
            print("  [start-armed] 已请求一次启动授权，输入 'start' 并回车开始：")
        else:
            print("  运行期授权只接受左摇杆按下；输入 'start' 开始：")
        print("─" * 60)
        while True:
            try:
                cmd = input("> ").strip().lower()
            except EOFError:
                cmd = "start"
            if cmd == "start":
                print("Start!\n")
                break
            print(f"  未识别的输入 '{cmd}'，请输入 'start' 继续。")

    # Warmup
    try:
        obs = None
        while obs is None and not shutdown_event.is_set():
            epoch = ros.inference_epoch()
            if epoch is None:
                time.sleep(0.05)
                continue
            obs = build_observation(ros, args.task, resize)
            if obs is None:
                time.sleep(0.005)
        if obs is not None and not shutdown_event.is_set() and ros.inference_epoch() == epoch:
            _ = policy.infer({"images": obs["images"], "state": obs["state"], "prompt": obs["prompt"]})
    except Exception as e:
        print(f"[startup_warmup_infer][WARN] {e}")

    # ── 启动后台推理线程 ──────────────────────────────────────────────────────
    stream_buffer = StreamActionBuffer(
        max_chunks=args.buffer_max_chunks,
        decay_alpha=args.exp_decay_alpha,
        state_dim=16,
        smooth_method="temporal",
    )
    ros.attach_action_buffer(stream_buffer)
    bg_thread = start_inference_thread(args, policy, ros, resize, stream_buffer)

    last_published_act: np.ndarray | None = None
    last_action_epoch = None
    waiting_for_authority = False
    published_real_commands = 0

    # ── 主控制循环（HIL 版） ──────────────────────────────────────────────────
    #
    # 底层模式变化仅能撤权；主循环不申请模式切换，也不自动恢复授权。

    try:
        while True:
            step_start = time.time()

            if shutdown_event.is_set():
                break

            act, action_epoch = ros.pop_authorized_action(stream_buffer)
            if action_epoch is None:
                if not waiting_for_authority:
                    print("[HIL] 模型已暂停，等待左摇杆授权及模式确认 ...")
                    waiting_for_authority = True
                last_published_act = None
                last_action_epoch = None
                time.sleep(0.05)
                continue
            if waiting_for_authority:
                print("[HIL] 左摇杆授权和模式已确认，恢复模型")
                waiting_for_authority = False
            if action_epoch != last_action_epoch:
                last_published_act = None
                last_action_epoch = action_epoch
            if act is None:
                time.sleep(0.001)
                continue

            # 关节死区（与原文件一致）
            if last_published_act is not None:
                hold_mask = np.abs(act - last_published_act) < JOINT_DEADBAND
                act = act.copy()
                act[hold_mask] = last_published_act[hold_mask]
            last_published_act = act.copy()

            if not args.dry_run:
                published = ros.publish(act, action_epoch)
                if published:
                    published_real_commands += 1
                tag = "replay" if published and args.replay_mode else ("real" if published else "skip")
            else:
                tag = "dry"
            print(f"[{tag}]  action={np.array2string(act, precision=3)}")

            elapsed = time.time() - step_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        shutdown_event.set()
        bg_thread.join(timeout=2.0)
        if not args.dry_run and not args.replay_mode and published_real_commands > 0:
            ros.publish_shutdown_hold(args.shutdown_hold_steps, dt)
        try:
            policy._ws.close()
        except Exception:
            pass
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
