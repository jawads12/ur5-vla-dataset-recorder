"""ROS 2 node for synchronized UR5 demonstration recording."""

from __future__ import annotations

import json
import shutil
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, Joy
from std_msgs.msg import Float64, Float64MultiArray, Int8
from std_srvs.srv import Trigger

from .core import (
    UR5_JOINT_ORDER,
    ValidationError,
    reorder_command,
    reorder_joint_positions,
)
from .episode_writer import EpisodeWriter


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def message_stamp_ns(message) -> int | None:
    header = getattr(message, "header", None)
    if header is None:
        return None
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class DatasetRecorder(Node):
    def __init__(self) -> None:
        super().__init__("dataset_recorder")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest: dict[str, tuple[object, float]] = {}
        self.episode_active = False
        self.episode_stopped = False
        self.writer: EpisodeWriter | None = None
        self.episode_name = ""
        self.episode_start_monotonic = 0.0
        self.episode_start_utc = ""
        self.frame_index = 0
        self.dropped_frames = 0
        self.last_warning_time = 0.0
        self.next_disk_check = 0.0
        self.output_root: Path | None = None
        self.previous_joy_buttons: list[int] = []
        self.joystick_control_busy = False
        self.joystick_control_lock = threading.Lock()

        self._create_subscriptions()
        self._create_services()
        rate = float(self.get_parameter("recording_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.sample_frame)
        self.get_logger().info(
            f"Recorder ready at {rate:.1f} Hz. Set 'task', then call ~/start_episode."
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "output_root": "~/ur5_vla_dataset",
            "task": "",
            "operator_note": "",
            "recording_rate_hz": 20.0,
            "base_image_topic": "/base/camera/color/image_raw",
            "wrist_image_topic": "/wrist/camera/color/image_raw",
            "enable_depth": True,
            "base_depth_topic": "/base/camera/aligned_depth_to_color/image_raw",
            "wrist_depth_topic": "/wrist/camera/aligned_depth_to_color/image_raw",
            "depth_unit_m": 0.001,
            "joint_state_topic": "/joint_states",
            "arm_command_topic": "/forward_position_controller/commands",
            "servo_status_topic": "/servo_node/status",
            "enable_joystick_episode_controls": True,
            "joy_topic": "/joy",
            # Physical button labels are one-based; Joy array indexes are zero-based.
            "start_episode_button_index": 8,
            "abort_episode_button_index": 9,
            "save_success_button_index": 10,
            "enable_gripper": False,
            "gripper_state_topic": "/gripper/joint_states",
            "gripper_command_topic": "/gripper_controller/command",
            "gripper_joint_name": "gripper_joint",
            "require_servo_status_zero": True,
            "max_joint_state_age_s": 0.10,
            "max_action_age_s": 0.10,
            "max_image_age_s": 0.10,
            "max_depth_age_s": 0.10,
            "max_gripper_age_s": 0.20,
            "minimum_free_disk_gb": 20.0,
            "jpeg_quality": 90,
            "writer_queue_size": 60,
            "command_joint_order": list(UR5_JOINT_ORDER),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _create_subscriptions(self) -> None:
        def cache(name):
            return lambda msg: self._cache(name, msg)

        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            cache("joint_state"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("arm_command_topic").value,
            cache("arm_command"),
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("base_image_topic").value,
            cache("base_image"),
            qos_profile_sensor_data,
        )
        if self.get_parameter("enable_depth").value:
            self.create_subscription(
                Image,
                self.get_parameter("base_depth_topic").value,
                cache("base_depth"),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                self.get_parameter("wrist_depth_topic").value,
                cache("wrist_depth"),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Image,
            self.get_parameter("wrist_image_topic").value,
            cache("wrist_image"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Int8,
            self.get_parameter("servo_status_topic").value,
            cache("servo_status"),
            10,
        )
        if self.get_parameter("enable_joystick_episode_controls").value:
            self.create_subscription(
                Joy,
                self.get_parameter("joy_topic").value,
                self._joy_callback,
                qos_profile_sensor_data,
            )
        if self.get_parameter("enable_gripper").value:
            self.create_subscription(
                JointState,
                self.get_parameter("gripper_state_topic").value,
                cache("gripper_state"),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Float64,
                self.get_parameter("gripper_command_topic").value,
                cache("gripper_command"),
                10,
            )

    def _create_services(self) -> None:
        self.create_service(Trigger, "~/start_episode", self.start_episode)
        self.create_service(Trigger, "~/stop_episode", self.stop_episode)
        self.create_service(Trigger, "~/save_success", self.save_success)
        self.create_service(Trigger, "~/save_failure", self.save_failure)
        self.create_service(Trigger, "~/discard_episode", self.discard_episode)
        self.create_service(Trigger, "~/status", self.status)

    def _cache(self, name: str, message: object) -> None:
        with self.lock:
            self.latest[name] = (message, time.monotonic())

    def _joy_callback(self, message: Joy) -> None:
        """Dispatch one episode action on each button rising edge."""
        buttons = [int(value) for value in message.buttons]
        previous = self.previous_joy_buttons
        self.previous_joy_buttons = buttons

        actions = (
            ("start", int(self.get_parameter("start_episode_button_index").value)),
            ("abort", int(self.get_parameter("abort_episode_button_index").value)),
            ("save_success", int(self.get_parameter("save_success_button_index").value)),
        )
        for action, index in actions:
            pressed = index < len(buttons) and buttons[index] == 1
            was_pressed = index < len(previous) and previous[index] == 1
            if pressed and not was_pressed:
                self._dispatch_joystick_action(action)
                break

    def _dispatch_joystick_action(self, action: str) -> None:
        with self.joystick_control_lock:
            if self.joystick_control_busy:
                self.get_logger().warning(
                    f"Ignoring joystick {action}: another episode action is running"
                )
                return
            self.joystick_control_busy = True
        threading.Thread(
            target=self._run_joystick_action,
            args=(action,),
            daemon=True,
        ).start()

    def _run_joystick_action(self, action: str) -> None:
        try:
            if action == "start":
                result = self.start_episode(None, Trigger.Response())
            elif action == "abort":
                if self.episode_active:
                    stopped = self.stop_episode(None, Trigger.Response())
                    if not stopped.success:
                        result = stopped
                    else:
                        result = self.discard_episode(None, Trigger.Response())
                elif self.episode_stopped:
                    result = self.discard_episode(None, Trigger.Response())
                else:
                    result = Trigger.Response(success=False, message="no episode to abort")
            elif action == "save_success":
                if self.episode_active:
                    stopped = self.stop_episode(None, Trigger.Response())
                    if not stopped.success:
                        result = stopped
                    else:
                        result = self.save_success(None, Trigger.Response())
                elif self.episode_stopped:
                    result = self.save_success(None, Trigger.Response())
                else:
                    result = Trigger.Response(success=False, message="no episode to save")
            else:
                result = Trigger.Response(success=False, message=f"unknown action {action}")

            log = self.get_logger().info if result.success else self.get_logger().warning
            log(f"Joystick {action}: {result.message}")
        except Exception as exc:
            self.get_logger().error(f"Joystick {action} failed: {exc}")
        finally:
            with self.joystick_control_lock:
                self.joystick_control_busy = False

    def required_streams(self) -> list[str]:
        names = ["joint_state", "arm_command", "base_image", "wrist_image"]
        if self.get_parameter("enable_depth").value:
            names += ["base_depth", "wrist_depth"]
        if self.get_parameter("require_servo_status_zero").value:
            names.append("servo_status")
        if self.get_parameter("enable_gripper").value:
            names += ["gripper_state", "gripper_command"]
        return names

    def readiness_errors(self, snapshot, now: float) -> list[str]:
        limits = {
            "joint_state": float(self.get_parameter("max_joint_state_age_s").value),
            "arm_command": float(self.get_parameter("max_action_age_s").value),
            "base_image": float(self.get_parameter("max_image_age_s").value),
            "wrist_image": float(self.get_parameter("max_image_age_s").value),
            "base_depth": float(self.get_parameter("max_depth_age_s").value),
            "wrist_depth": float(self.get_parameter("max_depth_age_s").value),
            "servo_status": float(self.get_parameter("max_action_age_s").value),
            "gripper_state": float(self.get_parameter("max_gripper_age_s").value),
            "gripper_command": float(self.get_parameter("max_gripper_age_s").value),
        }
        errors = []
        for name in self.required_streams():
            if name not in snapshot:
                errors.append(f"missing {name}")
            else:
                age = now - snapshot[name][1]
                if age > limits[name]:
                    errors.append(f"stale {name} ({age:.3f}s)")
        if "servo_status" in snapshot and self.get_parameter(
            "require_servo_status_zero"
        ).value:
            code = int(snapshot["servo_status"][0].data)
            if code != 0:
                errors.append(f"Servo status is {code}, expected 0")
        return errors

    def start_episode(self, _request, response):
        if self.episode_active or self.episode_stopped:
            response.success = False
            response.message = "finish or discard the current episode first"
            return response
        task = str(self.get_parameter("task").value).strip()
        if not task:
            response.success = False
            response.message = "set the task parameter before starting"
            return response
        with self.lock:
            snapshot = dict(self.latest)
        errors = self.readiness_errors(snapshot, time.monotonic())
        if errors:
            response.success = False
            response.message = "; ".join(errors)
            return response

        output_root = Path(str(self.get_parameter("output_root").value)).expanduser()
        output_root.mkdir(parents=True, exist_ok=True)
        free_gb = shutil.disk_usage(output_root).free / (1024**3)
        required_gb = float(self.get_parameter("minimum_free_disk_gb").value)
        if free_gb < required_gb:
            response.success = False
            response.message = f"only {free_gb:.1f} GB free; require {required_gb:.1f} GB"
            return response

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.episode_name = f"episode_{timestamp}"
        metadata = {
            "schema_version": 1,
            "episode_name": self.episode_name,
            "task": task,
            "operator_note": str(self.get_parameter("operator_note").value),
            "start_time_utc": utc_now(),
            "robot": "ur5",
            "robot_ip": "10.0.1.38",
            "ros_distribution": "humble",
            "recording_rate_hz": float(self.get_parameter("recording_rate_hz").value),
            "joint_order": list(UR5_JOINT_ORDER),
            "command_joint_order": list(self.get_parameter("command_joint_order").value),
            "base_image_topic": self.get_parameter("base_image_topic").value,
            "wrist_image_topic": self.get_parameter("wrist_image_topic").value,
            "depth_enabled": bool(self.get_parameter("enable_depth").value),
            "base_depth_topic": self.get_parameter("base_depth_topic").value,
            "wrist_depth_topic": self.get_parameter("wrist_depth_topic").value,
            "depth_unit_m": float(self.get_parameter("depth_unit_m").value),
            "depth_storage": "lossless 16-bit PNG aligned to color",
            "joint_state_topic": self.get_parameter("joint_state_topic").value,
            "arm_command_topic": self.get_parameter("arm_command_topic").value,
            "gripper_enabled": bool(self.get_parameter("enable_gripper").value),
            "action_semantics": "absolute_joint_position_target",
            "joint_units": "radian",
        }
        self.writer = EpisodeWriter(
            output_root,
            int(self.get_parameter("jpeg_quality").value),
            int(self.get_parameter("writer_queue_size").value),
            bool(self.get_parameter("enable_depth").value),
        )
        try:
            path = self.writer.start(self.episode_name, metadata)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response

        self.episode_start_monotonic = time.monotonic()
        self.output_root = output_root
        self.next_disk_check = self.episode_start_monotonic + 5.0
        self.episode_start_utc = metadata["start_time_utc"]
        self.frame_index = 0
        self.dropped_frames = 0
        self.episode_active = True
        response.success = True
        response.message = f"recording {path}"
        return response

    def sample_frame(self) -> None:
        if not self.episode_active or self.writer is None:
            return
        now = time.monotonic()
        if now >= self.next_disk_check:
            self.next_disk_check = now + 5.0
            if self.output_root is not None:
                free_gb = shutil.disk_usage(self.output_root).free / (1024**3)
                required_gb = float(self.get_parameter("minimum_free_disk_gb").value)
                if free_gb < required_gb:
                    self.get_logger().error(
                        f"Stopping recording: only {free_gb:.1f} GB free"
                    )
                    self.episode_active = False
                    self.episode_stopped = True
                    try:
                        self.writer.stop()
                    except Exception as exc:
                        self.get_logger().error(str(exc))
                    return
        with self.lock:
            snapshot = dict(self.latest)
        errors = self.readiness_errors(snapshot, now)
        if errors:
            self._drop("; ".join(errors))
            return
        if self.writer.error:
            self._drop(f"writer error: {self.writer.error}")
            self.episode_active = False
            self.episode_stopped = True
            return

        try:
            joint_msg = snapshot["joint_state"][0]
            command_msg = snapshot["arm_command"][0]
            actual = reorder_joint_positions(joint_msg.name, joint_msg.position)
            command = reorder_command(
                command_msg.data,
                list(self.get_parameter("command_joint_order").value),
            )
            base_msg = snapshot["base_image"][0]
            wrist_msg = snapshot["wrist_image"][0]
            base_bgr = self.bridge.imgmsg_to_cv2(base_msg, desired_encoding="bgr8")
            wrist_bgr = self.bridge.imgmsg_to_cv2(wrist_msg, desired_encoding="bgr8")
            base_depth = None
            wrist_depth = None
            base_depth_msg = None
            wrist_depth_msg = None
            if self.get_parameter("enable_depth").value:
                base_depth_msg = snapshot["base_depth"][0]
                wrist_depth_msg = snapshot["wrist_depth"][0]
                base_depth = self.bridge.imgmsg_to_cv2(
                    base_depth_msg, desired_encoding="passthrough"
                )
                wrist_depth = self.bridge.imgmsg_to_cv2(
                    wrist_depth_msg, desired_encoding="passthrough"
                )
                if base_depth.dtype != np.uint16 or wrist_depth.dtype != np.uint16:
                    raise ValidationError(
                        "depth images must use 16UC1/uint16 for lossless PNG storage"
                    )

            observation_state = list(actual)
            action = list(command)
            if self.get_parameter("enable_gripper").value:
                gripper_msg = snapshot["gripper_state"][0]
                gripper_name = str(self.get_parameter("gripper_joint_name").value)
                gripper_map = dict(zip(gripper_msg.name, gripper_msg.position))
                if gripper_name not in gripper_map:
                    raise ValidationError(f"missing gripper joint {gripper_name}")
                observation_state.append(float(gripper_map[gripper_name]))
                action.append(float(snapshot["gripper_command"][0].data))

            receive_ages = {
                name: round(now - snapshot[name][1], 6)
                for name in self.required_streams()
            }
            record = {
                "episode_time_s": round(now - self.episode_start_monotonic, 9),
                "record_time_utc": utc_now(),
                "frame_index": self.frame_index,
                "task": str(self.get_parameter("task").value),
                "observation.state": observation_state,
                "action": action,
                "joint_state_source_stamp_ns": message_stamp_ns(joint_msg),
                "base_image_source_stamp_ns": message_stamp_ns(base_msg),
                "wrist_image_source_stamp_ns": message_stamp_ns(wrist_msg),
                "base_depth_source_stamp_ns": message_stamp_ns(base_depth_msg)
                if base_depth_msg is not None
                else None,
                "wrist_depth_source_stamp_ns": message_stamp_ns(wrist_depth_msg)
                if wrist_depth_msg is not None
                else None,
                "base_depth_encoding": base_depth_msg.encoding
                if base_depth_msg is not None
                else None,
                "wrist_depth_encoding": wrist_depth_msg.encoding
                if wrist_depth_msg is not None
                else None,
                "receive_age_s": receive_ages,
                "servo_status": int(snapshot["servo_status"][0].data)
                if "servo_status" in snapshot
                else None,
                "valid": True,
            }
            self.writer.enqueue(
                record,
                base_bgr.copy(),
                wrist_bgr.copy(),
                base_depth.copy() if base_depth is not None else None,
                wrist_depth.copy() if wrist_depth is not None else None,
            )
            self.frame_index += 1
        except Exception as exc:
            self._drop(str(exc))

    def _drop(self, reason: str) -> None:
        self.dropped_frames += 1
        now = time.monotonic()
        if now - self.last_warning_time > 1.0:
            self.get_logger().warning(f"Dropped frame: {reason}")
            self.last_warning_time = now

    def stop_episode(self, _request, response):
        if not self.episode_active or self.writer is None:
            response.success = False
            response.message = "no active episode"
            return response
        self.episode_active = False
        try:
            self.writer.stop()
        except Exception as exc:
            self.episode_stopped = True
            response.success = False
            response.message = str(exc)
            return response
        self.episode_stopped = True
        response.success = True
        response.message = (
            f"stopped with {self.writer.frames_written} frames; "
            f"{self.dropped_frames} dropped. Save or discard it."
        )
        return response

    def _save(self, outcome: str, response):
        if not self.episode_stopped or self.writer is None:
            response.success = False
            response.message = "stop an episode before saving"
            return response
        try:
            final_path = self.writer.finalize(
                outcome,
                {
                    "end_time_utc": utc_now(),
                    "dropped_frames": self.dropped_frames,
                },
            )
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        self._reset_episode()
        response.success = True
        response.message = f"saved {outcome} episode to {final_path}"
        return response

    def save_success(self, _request, response):
        return self._save("success", response)

    def save_failure(self, _request, response):
        return self._save("failure", response)

    def discard_episode(self, _request, response):
        if self.episode_active:
            response.success = False
            response.message = "stop the episode before discarding"
            return response
        if self.writer is None:
            response.success = False
            response.message = "no episode to discard"
            return response
        self.writer.discard()
        self._reset_episode()
        response.success = True
        response.message = "incomplete episode discarded"
        return response

    def status(self, _request, response):
        with self.lock:
            snapshot = dict(self.latest)
        errors = self.readiness_errors(snapshot, time.monotonic())
        data = {
            "active": self.episode_active,
            "stopped_pending_label": self.episode_stopped,
            "episode": self.episode_name or None,
            "frames_sampled": self.frame_index,
            "frames_written": self.writer.frames_written if self.writer else 0,
            "dropped_frames": self.dropped_frames,
            "ready": not errors,
            "errors": errors,
        }
        response.success = not errors
        response.message = json.dumps(data)
        return response

    def _reset_episode(self) -> None:
        self.episode_active = False
        self.episode_stopped = False
        self.writer = None
        self.episode_name = ""
        self.output_root = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.episode_active and node.writer:
            node.episode_active = False
            try:
                node.writer.stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
