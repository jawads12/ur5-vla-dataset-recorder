"""Logitech Extreme 3D Pro teleoperation and pneumatic-gripper control."""

from __future__ import annotations

from collections.abc import Callable

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from ur_msgs.srv import SetIO


DEADMAN_BUTTON = 0
GRIPPER_CLOSE_BUTTON = 1
GRIPPER_OPEN_BUTTON = 2
Z_DOWN_BUTTON = 4
Z_UP_BUTTON = 5

AXIS_LEFT_RIGHT = 0
AXIS_FORWARD_BACKWARD = 1
AXIS_HANDLE_TWIST = 2

# Conservative collection defaults. Increase only after workspace validation.
LINEAR_SPEED = 0.10
ANGULAR_SPEED = 0.40

IO_FUNCTION_SET_DIGITAL_OUT = 1
GRIPPER_CLOSE_PIN = 0
GRIPPER_OPEN_PIN = 1
OUTPUT_ON = 1.0
OUTPUT_OFF = 0.0

GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 1.0


class LogitechServo(Node):
    def __init__(self) -> None:
        super().__init__("logitech_servo_mapper")

        self.twist_publisher = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )
        gripper_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gripper_command_publisher = self.create_publisher(
            Float64,
            "/gripper/command",
            gripper_qos,
        )
        self.gripper_state_publisher = self.create_publisher(
            Float64,
            "/gripper/state",
            gripper_qos,
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            qos_profile_sensor_data,
        )
        self.io_client = self.create_client(
            SetIO,
            "/io_and_status_controller/set_io",
        )

        self.previous_buttons: list[int] = []
        self.gripper_busy = False
        self.last_gripper_value: float | None = None
        self.gripper_heartbeat = self.create_timer(
            0.05,
            self.publish_gripper_heartbeat,
        )

        self.get_logger().info("Logitech UR5 teleoperation started.")
        self.get_logger().info("Hold trigger to move the arm.")
        self.get_logger().info("Joy index 1: CLOSE | Joy index 2: OPEN")
        self.get_logger().info(
            "Operate the gripper once before recording to initialize its state."
        )

        if self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("UR SetIO service is available.")
        else:
            self.get_logger().warning("UR SetIO service is not currently available.")

    @staticmethod
    def axis(message: Joy, index: int) -> float:
        return float(message.axes[index]) if 0 <= index < len(message.axes) else 0.0

    @staticmethod
    def button(message: Joy, index: int) -> int:
        return int(message.buttons[index]) if 0 <= index < len(message.buttons) else 0

    def send_io(
        self,
        pin: int,
        state: float,
        description: str,
        on_success: Callable[[], None] | None = None,
        on_failure: Callable[[], None] | None = None,
    ) -> None:
        if not self.io_client.service_is_ready():
            self.get_logger().warning(
                "Cannot control gripper: SetIO service unavailable."
            )
            if on_failure is not None:
                on_failure()
            return

        request = SetIO.Request()
        request.fun = IO_FUNCTION_SET_DIGITAL_OUT
        request.pin = pin
        request.state = float(state)
        self.get_logger().info(f"{description}: pin={pin}, state={state}")

        future = self.io_client.call_async(request)
        future.add_done_callback(
            lambda completed: self.io_response_callback(
                completed,
                description,
                on_success,
                on_failure,
            )
        )

    def io_response_callback(
        self,
        future,
        description: str,
        on_success: Callable[[], None] | None,
        on_failure: Callable[[], None] | None,
    ) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{description} completed successfully.")
                if on_success is not None:
                    on_success()
                return
            self.get_logger().warning(
                f"{description} was rejected by the UR controller."
            )
        except Exception as error:
            self.get_logger().error(f"{description} failed: {error}")

        if on_failure is not None:
            on_failure()

    def publish_gripper_value(self, value: float) -> None:
        self.last_gripper_value = float(value)
        message = Float64(data=self.last_gripper_value)
        self.gripper_command_publisher.publish(message)
        self.gripper_state_publisher.publish(message)

    def publish_gripper_heartbeat(self) -> None:
        if self.last_gripper_value is not None:
            self.publish_gripper_value(self.last_gripper_value)

    def finish_gripper_command(self, value: float, description: str) -> None:
        self.publish_gripper_value(value)
        self.gripper_busy = False
        self.get_logger().info(f"{description}; dataset value={value:.1f}")

    def fail_gripper_command(self) -> None:
        self.gripper_busy = False
        self.get_logger().error("Gripper command failed; dataset value was not changed.")

    def open_gripper(self) -> None:
        if self.gripper_busy:
            self.get_logger().warning(
                "Ignoring OPEN: another gripper command is running."
            )
            return
        self.gripper_busy = True
        self.send_io(
            GRIPPER_CLOSE_PIN,
            OUTPUT_OFF,
            "Disable CLOSE output",
            on_success=lambda: self.send_io(
                GRIPPER_OPEN_PIN,
                OUTPUT_ON,
                "OPEN gripper",
                on_success=lambda: self.finish_gripper_command(
                    GRIPPER_OPEN,
                    "OPEN confirmed",
                ),
                on_failure=self.fail_gripper_command,
            ),
            on_failure=self.fail_gripper_command,
        )

    def close_gripper(self) -> None:
        if self.gripper_busy:
            self.get_logger().warning(
                "Ignoring CLOSE: another gripper command is running."
            )
            return
        self.gripper_busy = True
        self.send_io(
            GRIPPER_OPEN_PIN,
            OUTPUT_OFF,
            "Disable OPEN output",
            on_success=lambda: self.send_io(
                GRIPPER_CLOSE_PIN,
                OUTPUT_ON,
                "CLOSE gripper",
                on_success=lambda: self.finish_gripper_command(
                    GRIPPER_CLOSED,
                    "CLOSE confirmed",
                ),
                on_failure=self.fail_gripper_command,
            ),
            on_failure=self.fail_gripper_command,
        )

    def process_gripper_buttons(self, joy: Joy) -> None:
        if len(self.previous_buttons) != len(joy.buttons):
            self.previous_buttons = [0] * len(joy.buttons)

        close_pressed = self.button(joy, GRIPPER_CLOSE_BUTTON) == 1
        open_pressed = self.button(joy, GRIPPER_OPEN_BUTTON) == 1
        close_was_pressed = self.previous_buttons[GRIPPER_CLOSE_BUTTON] == 1
        open_was_pressed = self.previous_buttons[GRIPPER_OPEN_BUTTON] == 1

        close_rising = close_pressed and not close_was_pressed
        open_rising = open_pressed and not open_was_pressed
        if close_rising and open_rising:
            self.get_logger().warning(
                "Ignoring simultaneous OPEN and CLOSE buttons."
            )
        elif close_rising:
            self.close_gripper()
        elif open_rising:
            self.open_gripper()

        self.previous_buttons = list(joy.buttons)

    def joy_callback(self, joy: Joy) -> None:
        self.process_gripper_buttons(joy)

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "tool0"

        if self.button(joy, DEADMAN_BUTTON) == 1:
            command.twist.linear.x = (
                -self.axis(joy, AXIS_FORWARD_BACKWARD) * LINEAR_SPEED
            )
            command.twist.linear.y = self.axis(joy, AXIS_LEFT_RIGHT) * LINEAR_SPEED
            command.twist.linear.z = (
                self.button(joy, Z_UP_BUTTON) - self.button(joy, Z_DOWN_BUTTON)
            ) * LINEAR_SPEED
            command.twist.angular.z = (
                self.axis(joy, AXIS_HANDLE_TWIST) * ANGULAR_SPEED
            )

        self.twist_publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LogitechServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Logitech UR5 teleoperation.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
