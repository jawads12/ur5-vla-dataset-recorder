"""Original Logitech teleoperation behavior plus dataset gripper topics."""

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


# Logitech joystick mapping
DEADMAN_BUTTON = 0
GRIPPER_CLOSE_BUTTON = 1
GRIPPER_OPEN_BUTTON = 2
Z_DOWN_BUTTON = 4
Z_UP_BUTTON = 5

AXIS_LEFT_RIGHT = 0
AXIS_FORWARD_BACKWARD = 1
AXIS_HANDLE_TWIST = 2

# Original working motion speeds
LINEAR_SPEED = 0.35
ANGULAR_SPEED = 0.60

# UR digital outputs
IO_FUNCTION_SET_DIGITAL_OUT = 1
GRIPPER_CLOSE_PIN = 0
GRIPPER_OPEN_PIN = 1
OUTPUT_ON = 1.0
OUTPUT_OFF = 0.0

# Dataset convention
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = 1.0


class LogitechServo(Node):
    def __init__(self):
        super().__init__("logitech_servo_mapper")

        self.twist_publisher = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
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

        # Added only for dataset recording. Transient-local QoS preserves the
        # latest value for a recorder that starts after the mapper.
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

        self.previous_buttons = []
        self.last_gripper_value = None
        self.gripper_heartbeat = self.create_timer(
            0.05,
            self.publish_gripper_heartbeat,
        )

        self.get_logger().info("Logitech UR5 teleoperation started.")
        self.get_logger().info("Hold trigger to move the arm.")
        self.get_logger().info(
            "Button index 1: CLOSE gripper | Button index 2: OPEN gripper"
        )
        self.get_logger().info(
            "Operate the gripper once before recording to initialize its value."
        )

        if self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("UR SetIO service is available.")
        else:
            self.get_logger().warning(
                "UR SetIO service is not currently available."
            )

    @staticmethod
    def axis(message: Joy, index: int) -> float:
        if 0 <= index < len(message.axes):
            return float(message.axes[index])
        return 0.0

    @staticmethod
    def button(message: Joy, index: int) -> int:
        if 0 <= index < len(message.buttons):
            return int(message.buttons[index])
        return 0

    def publish_gripper_value(self, value: float) -> None:
        self.last_gripper_value = float(value)
        message = Float64()
        message.data = self.last_gripper_value
        self.gripper_command_publisher.publish(message)
        self.gripper_state_publisher.publish(message)

    def publish_gripper_heartbeat(self) -> None:
        if self.last_gripper_value is not None:
            self.publish_gripper_value(self.last_gripper_value)

    def send_io(
        self,
        pin: int,
        state: float,
        description: str,
        dataset_value=None,
    ) -> None:
        if not self.io_client.service_is_ready():
            self.get_logger().warning(
                "Cannot control gripper: SetIO service unavailable."
            )
            return

        request = SetIO.Request()
        request.fun = IO_FUNCTION_SET_DIGITAL_OUT
        request.pin = pin
        request.state = float(state)

        self.get_logger().info(f"{description}: pin={pin}, state={state}")

        future = self.io_client.call_async(request)
        future.add_done_callback(
            lambda completed_future: self.io_response_callback(
                completed_future,
                description,
                dataset_value,
            )
        )

    def io_response_callback(
        self,
        future,
        description: str,
        dataset_value,
    ) -> None:
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    f"{description} completed successfully."
                )
                if dataset_value is not None:
                    self.publish_gripper_value(dataset_value)
                    self.get_logger().info(
                        f"Dataset gripper value={dataset_value:.1f}"
                    )
            else:
                self.get_logger().warning(
                    f"{description} was rejected by the UR controller."
                )

        except Exception as error:
            self.get_logger().error(f"{description} failed: {error}")

    def open_gripper(self) -> None:
        # Original I/O behavior: disable CLOSE, then activate OPEN.
        self.send_io(
            GRIPPER_CLOSE_PIN,
            OUTPUT_OFF,
            "Disable CLOSE output",
        )
        self.send_io(
            GRIPPER_OPEN_PIN,
            OUTPUT_ON,
            "OPEN gripper",
            dataset_value=GRIPPER_OPEN,
        )

    def close_gripper(self) -> None:
        # Original I/O behavior: disable OPEN, then activate CLOSE.
        self.send_io(
            GRIPPER_OPEN_PIN,
            OUTPUT_OFF,
            "Disable OPEN output",
        )
        self.send_io(
            GRIPPER_CLOSE_PIN,
            OUTPUT_ON,
            "CLOSE gripper",
            dataset_value=GRIPPER_CLOSED,
        )

    def process_gripper_buttons(self, joy: Joy) -> None:
        if len(self.previous_buttons) != len(joy.buttons):
            self.previous_buttons = [0] * len(joy.buttons)

        close_pressed = self.button(joy, GRIPPER_CLOSE_BUTTON) == 1
        open_pressed = self.button(joy, GRIPPER_OPEN_BUTTON) == 1

        close_was_pressed = (
            self.previous_buttons[GRIPPER_CLOSE_BUTTON] == 1
            if GRIPPER_CLOSE_BUTTON < len(self.previous_buttons)
            else False
        )
        open_was_pressed = (
            self.previous_buttons[GRIPPER_OPEN_BUTTON] == 1
            if GRIPPER_OPEN_BUTTON < len(self.previous_buttons)
            else False
        )

        if close_pressed and not close_was_pressed:
            self.close_gripper()

        if open_pressed and not open_was_pressed:
            self.open_gripper()

        self.previous_buttons = list(joy.buttons)

    def joy_callback(self, joy: Joy) -> None:
        self.process_gripper_buttons(joy)

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "tool0"

        deadman_pressed = self.button(joy, DEADMAN_BUTTON) == 1

        if deadman_pressed:
            command.twist.linear.x = (
                -self.axis(joy, AXIS_FORWARD_BACKWARD) * LINEAR_SPEED
            )
            command.twist.linear.y = (
                self.axis(joy, AXIS_LEFT_RIGHT) * LINEAR_SPEED
            )
            command.twist.linear.z = (
                self.button(joy, Z_UP_BUTTON)
                - self.button(joy, Z_DOWN_BUTTON)
            ) * LINEAR_SPEED
            command.twist.angular.z = (
                self.axis(joy, AXIS_HANDLE_TWIST) * ANGULAR_SPEED
            )

        # Releasing the trigger publishes zero motion.
        self.twist_publisher.publish(command)


def main(args=None):
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
